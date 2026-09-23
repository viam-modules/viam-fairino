package viamfairino

import (
	"context"
	"errors"
	"fmt"
	"time"

	arm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	rdkutils "go.viam.com/rdk/utils"
)

// maxServoIntervalSec is the longest gap between streamed points. ServoJ moves
// to each setpoint over the given interval, and the vendor SDK recommends
// command cycles of at most 16 ms; points further apart than that would be
// executed as a jump rather than a trajectory.
const maxServoIntervalSec = 0.016

// MoveThroughJointPositionsStreamed executes a trajectory that arrives
// incrementally, one ServoJ setpoint per point, each sent at the point's Time
// and told to take the time since the previous point. Same pacing scheme as
// viam-ufactory-xarm's servo-mode streaming: wall-clock is anchored when the
// first moving point is sent, and a point that arrives late is sent at once.
//
// It is one opMgr operation, so Stop or a new move cancels it; on any early
// exit the arm is sent STOP, since the controller would otherwise hold (or
// finish) its last setpoint. When the client closes the stream, it waits for
// the arm to settle at the final point.
func (s *viamFairinoFairino) MoveThroughJointPositionsStreamed(
	ctx context.Context,
	batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response,
	extra map[string]interface{},
) (err error) {
	ctx, done := s.opMgr.New(ctx)
	defer done()
	if err := s.checkReadyToMove(ctx); err != nil {
		return err
	}
	return s.runServoStream(ctx, batches, responses)
}

// runServoStream is MoveThroughJointPositionsStreamed's executor, shared with
// the trajectory-generator path (trajgen.go). The caller owns the operation.
func (s *viamFairinoFairino) runServoStream(
	ctx context.Context,
	batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response,
) (err error) {
	var (
		anchor  time.Time
		sent    bool
		lastDeg [6]float64
		v       trajectoryStreamValidator
	)
	defer func() {
		if err != nil && sent {
			s.stopDetached()
		}
	}()

	for {
		var batch []arm.TrajectoryPoint
		select {
		case <-ctx.Done():
			return ctx.Err()
		case b, ok := <-batches:
			if !ok {
				if !sent {
					return nil
				}
				return s.waitForMotionComplete(ctx, lastDeg, 5*time.Second)
			}
			batch = b
		}

		for _, p := range batch {
			prev := v.lastTime
			if err := v.validate(p); err != nil {
				return err
			}
			if err := s.checkJointTargets(ctx, [][]referenceframe.Input{p.Positions}); err != nil {
				return err
			}
			// The first point is where the arm already is; there's nothing to send.
			if p.Time == 0 {
				continue
			}
			interval := (p.Time - prev).Seconds()
			if interval > maxServoIntervalSec {
				return fmt.Errorf("trajectory points are %v apart; ServoJ needs them at most %v apart",
					p.Time-prev, time.Duration(maxServoIntervalSec*float64(time.Second)))
			}

			for i := range lastDeg {
				lastDeg[i] = rdkutils.RadToDeg(p.Positions[i])
			}
			if anchor.IsZero() {
				anchor = time.Now()
			}
			if err := s.cmd.servoJ(ctx, lastDeg, interval); err != nil {
				return fmt.Errorf("ServoJ at %v: %w", p.Time, err)
			}
			sent = true
			if !sleepCtx(ctx, time.Until(anchor.Add(p.Time))) {
				return ctx.Err()
			}
		}

		select {
		case responses <- arm.Response{}:
		case <-ctx.Done():
			return ctx.Err()
		}
	}
}

// trajectoryStreamValidator checks the shape a streamed trajectory must have,
// one point at a time: the first at time zero and at rest, later ones strictly
// later, all with the same number of joints. The same checks as
// viam-ufactory-xarm's, which are meant to move into RDK (RSDK-14275).
type trajectoryStreamValidator struct {
	seen     bool
	lastTime time.Duration
	dof      int
}

func (v *trajectoryStreamValidator) validate(p arm.TrajectoryPoint) error {
	if len(p.Positions) == 0 {
		return errors.New("trajectory point has no joint positions")
	}
	if !v.seen {
		if p.Time != 0 {
			return fmt.Errorf("first trajectory point must be at time zero, got %v", p.Time)
		}
		if p.Constraints != nil {
			for i, vel := range p.Constraints.Velocities {
				if vel != 0 {
					return fmt.Errorf("first trajectory point must be at rest, but joint %d has velocity %v", i, vel)
				}
			}
		}
		v.seen, v.dof = true, len(p.Positions)
		return nil
	}
	if p.Time <= v.lastTime {
		return fmt.Errorf("trajectory point times must strictly increase, got %v after %v", p.Time, v.lastTime)
	}
	if len(p.Positions) != v.dof {
		return fmt.Errorf("trajectory points must all have %d joint positions, got %d", v.dof, len(p.Positions))
	}
	v.lastTime = p.Time
	return nil
}
