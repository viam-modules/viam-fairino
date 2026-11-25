package viamfairino

import (
	"context"
	"errors"
	"fmt"
	"math"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	rdkutils "go.viam.com/rdk/utils"
)

const (
	// The vendor SDK's default ServoJ cmdT; longer gaps are split into steps no longer than this.
	maxServoInterval = 8 * time.Millisecond
	// The bottom of ServoJ's documented cmdT range.
	minServoInterval = time.Millisecond
	// No FR joint is rated faster (Cobots manual Table 2.1-1); a faster step is a jump, not a trajectory.
	maxServoJointSpeedDegPerSec = 180.0
	streamSettleTimeout         = 2 * time.Second
	// StreamStartAtKey in extra is the wall-clock start (Unix microseconds) for the trajectory's t=0.
	// Callers moving several arms together pass the same value to each.
	StreamStartAtKey = "start_at_unix_us"
	// How late past start_at the trajectory may still begin; later than this it would run out of step.
	maxStreamStartLate = 2 * time.Millisecond
)

// streamStartAt reads extra's start time; the zero Time means "start as soon as the trajectory arrives".
func streamStartAt(extra map[string]interface{}) (time.Time, error) {
	v, ok := extra[StreamStartAtKey]
	if !ok {
		return time.Time{}, nil
	}
	var us float64
	switch n := v.(type) {
	case float64:
		us = n
	case int64:
		us = float64(n)
	case int:
		us = float64(n)
	default:
		return time.Time{}, fmt.Errorf("%s must be a number of Unix microseconds, got %T", StreamStartAtKey, v)
	}
	return time.UnixMicro(int64(us)), nil
}

// MoveThroughJointPositionsStreamed plays the trajectory as ServoJ setpoints.
// Each setpoint is sent at the previous point's time with the gap as its
// interval, so the arm reaches it at its own time.
func (s *viamFairinoFairino) MoveThroughJointPositionsStreamed(
	ctx context.Context,
	batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response,
	extra map[string]interface{},
) error {
	ctx, done := s.opMgr.New(ctx)
	defer done()

	startAt, err := streamStartAt(extra)
	if err != nil {
		return err
	}

	var (
		prev    arm.TrajectoryPoint
		started bool // the t=0 point has been accepted
		moving  bool // a ServoJ may have gone out
		anchor  time.Time
		tr      *streamTrace
	)
	fail := func(err error) error {
		if moving {
			s.stopAfterFailure(ctx)
		}
		return err
	}

	for {
		var batch []arm.TrajectoryPoint
		select {
		case <-ctx.Done():
			return fail(ctx.Err())
		case b, ok := <-batches:
			if !ok {
				if !moving {
					return nil
				}
				if err := s.waitForStreamSettle(ctx, prev); err != nil {
					return fail(err)
				}
				return nil
			}
			batch = b
		}

		for _, p := range batch {
			if err := s.checkStreamPoint(ctx, started, prev, p); err != nil {
				return fail(err)
			}
			if started {
				if anchor.IsZero() {
					anchor = time.Now()
					if !startAt.IsZero() {
						if late := anchor.Sub(startAt); late > maxStreamStartLate {
							return fail(fmt.Errorf("trajectory was ready %v after its %s start; not starting out of step",
								late, StreamStartAtKey))
						}
						if err := sleepUntil(ctx, startAt); err != nil {
							return fail(err)
						}
						anchor = startAt
					}
					tr = newStreamTrace(s.logger, anchor, inputsToDeg(prev.Positions))
				}
				moving = true
				if err := s.servoBetween(ctx, anchor, prev, p, tr); err != nil {
					return fail(err)
				}
			}
			prev, started = p, true
		}

		select {
		case responses <- arm.Response{}:
		case <-ctx.Done():
			return fail(ctx.Err())
		}
	}
}

func (s *viamFairinoFairino) checkStreamPoint(ctx context.Context, started bool, prev, p arm.TrajectoryPoint) error {
	if len(p.Positions) != 6 {
		return fmt.Errorf("trajectory point at %v has %d joint positions, arm has 6", p.Time, len(p.Positions))
	}
	for i, lim := range s.model.DoF() {
		if p.Positions[i] < lim.Min || p.Positions[i] > lim.Max {
			return fmt.Errorf("trajectory point at %v puts joint %d at %.2f°, outside [%.2f°, %.2f°]", p.Time, i+1,
				rdkutils.RadToDeg(p.Positions[i]), rdkutils.RadToDeg(lim.Min), rdkutils.RadToDeg(lim.Max))
		}
	}

	if !started {
		if p.Time != 0 {
			return fmt.Errorf("first trajectory point must be at time zero, got %v", p.Time)
		}
		if p.Constraints != nil {
			for i, v := range p.Constraints.Velocities {
				if v != 0 {
					return fmt.Errorf("first trajectory point must be at rest, but joint %d has velocity %v", i+1, v)
				}
			}
		}
		fr, err := s.latestFrame(ctx)
		if err != nil {
			return fmt.Errorf("reading current joints: %w", err)
		}
		for i, deg := range inputsToDeg(p.Positions) {
			if math.Abs(deg-fr.JointDegs[i]) > jointToleranceDeg {
				return fmt.Errorf("first trajectory point is not where the arm is: joint %d is at %.3f°, trajectory starts at %.3f°",
					i+1, fr.JointDegs[i], deg)
			}
		}
		return nil
	}

	dt := p.Time - prev.Time
	if dt <= 0 {
		return fmt.Errorf("trajectory point times must strictly increase, got %v after %v", p.Time, prev.Time)
	}
	from, to := inputsToDeg(prev.Positions), inputsToDeg(p.Positions)
	for i := range to {
		if speed := math.Abs(to[i]-from[i]) / dt.Seconds(); speed > maxServoJointSpeedDegPerSec {
			return fmt.Errorf("trajectory point at %v moves joint %d at %.0f°/s, faster than %.0f°/s",
				p.Time, i+1, speed, maxServoJointSpeedDegPerSec)
		}
	}
	return nil
}

func (s *viamFairinoFairino) servoBetween(ctx context.Context, anchor time.Time, from, to arm.TrajectoryPoint, tr *streamTrace) error {
	dt := to.Time - from.Time
	steps := int(math.Ceil(float64(dt) / float64(maxServoInterval)))
	step := dt / time.Duration(steps)
	interval := max(step, minServoInterval)
	fromDeg, toDeg := inputsToDeg(from.Positions), inputsToDeg(to.Positions)

	for k := 1; k <= steps; k++ {
		if fr, ok, _ := s.status.latest(); ok {
			if err := motionFault(fr); err != nil {
				return err
			}
			tr.observe(fr)
		}
		frac := float64(k) / float64(steps)
		var target [6]float64
		for i := range target {
			target[i] = fromDeg[i] + (toDeg[i]-fromDeg[i])*frac
		}
		slot := anchor.Add(from.Time + step*time.Duration(k-1))
		sentAt := time.Now()
		if err := s.cmd.servoJ(ctx, target, interval); err != nil {
			return fmt.Errorf("ServoJ toward the point at %v: %w", to.Time, err)
		}
		tr.sent(from.Time+step*time.Duration(k), target, sentAt.Sub(slot), time.Since(sentAt))
		if err := sleepUntil(ctx, anchor.Add(from.Time+step*time.Duration(k))); err != nil {
			return err
		}
	}
	return nil
}

func (s *viamFairinoFairino) waitForStreamSettle(ctx context.Context, last arm.TrajectoryPoint) error {
	settleCtx, cancel := context.WithTimeout(ctx, streamSettleTimeout)
	defer cancel()
	err := s.waitForMotionComplete(settleCtx, inputsToDeg(last.Positions))
	if errors.Is(err, context.DeadlineExceeded) && ctx.Err() == nil {
		return fmt.Errorf("arm did not settle at the trajectory's last point within %v", streamSettleTimeout)
	}
	return err
}

func inputsToDeg(in []referenceframe.Input) [6]float64 {
	var out [6]float64
	for i := range out {
		out[i] = rdkutils.RadToDeg(in[i])
	}
	return out
}

func sleepUntil(ctx context.Context, t time.Time) error {
	timer := time.NewTimer(time.Until(t))
	defer timer.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-timer.C:
		return nil
	}
}
