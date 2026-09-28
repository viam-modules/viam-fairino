package viamfairino

import (
	"context"
	"errors"
	"fmt"
	"time"

	arm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
)

const (
	// maxServoIntervalSec is the longest gap between streamed points. ServoJ
	// moves to each setpoint over the given interval, and the vendor SDK
	// recommends command cycles of at most 16 ms; points further apart than
	// that would be executed as a jump rather than a trajectory.
	maxServoIntervalSec = 0.016

	// minServoIntervalSec is the shortest gap between streamed points: the
	// vendor's minimum ServoJ command cycle (cmdT, 0.001-0.016 s). ServoJ's
	// cmdT goes out with four decimals, so a shorter gap would reach the
	// controller out of range, or as 0.
	minServoIntervalSec = 0.001

	// servoPaceSlack is how late a point may be sent and still count as on
	// schedule. Timers always wake a little late, and the checks before each
	// ServoJ take time too; re-anchoring on every such delay would stretch
	// every interval by it, sending setpoints slower than the cmdT they
	// carry, so the arm would stop and start at every one. Only lateness
	// beyond this (a slow client, a stalled connection) moves the schedule.
	servoPaceSlack = 2 * time.Millisecond

	// streamStartToleranceRad is how far a stream's first point may be from
	// where the arm actually is, per joint. ServoJ has no approach move of its
	// own: the first setpoint after it goes out over one interval, so a start
	// that isn't the arm's position would be a jump. Same value as RDK's
	// motion service uses before executing a plan (defaultExecuteEpsilon).
	streamStartToleranceRad = 0.01

	// maxServoStepSpeedDegsPerSec caps the joint speed any one step of a
	// stream may imply: the arm's top speed, plus 5% for sampling noise in a
	// trajectory planned right at the limit. Anything faster is a
	// discontinuity, not a trajectory.
	maxServoStepSpeedDegsPerSec = maxJointSpeedDegsPerSec * 1.05

	// servoSettleTimeout bounds the wait for the arm to settle at a stream's
	// final point once the last setpoint has gone out.
	servoSettleTimeout = 5 * time.Second
)

// MoveThroughJointPositionsStreamed executes a trajectory that arrives
// incrementally, one ServoJ setpoint per point (see servoStreamer), each sent
// at the point's Time and told to take the time since the previous point.
//
// It is one opMgr operation, so Stop or a new move cancels it; on any early
// exit after a setpoint went out, the arm is sent STOP, since the controller
// would otherwise hold (or finish) its last setpoint. When the client closes
// the stream, it waits for the arm to settle at the final point.
func (s *viamFairinoFairino) MoveThroughJointPositionsStreamed(
	ctx context.Context,
	batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response,
	extra map[string]interface{},
) error {
	ctx, done := s.opMgr.New(ctx)
	defer done()
	if err := s.checkReadyToMove(ctx); err != nil {
		return err
	}
	st := &servoStreamer{s: s}
	err := s.runServoStream(ctx, st, batches, responses)
	if err != nil && st.sent {
		s.stopDetached()
	}
	return err
}

// runServoStream feeds every batch through st, answering each with a
// response, until the client closes the stream; then it waits for the arm to
// settle.
func (s *viamFairinoFairino) runServoStream(
	ctx context.Context,
	st *servoStreamer,
	batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response,
) error {
	for {
		var batch []arm.TrajectoryPoint
		select {
		case <-ctx.Done():
			return ctx.Err()
		case b, ok := <-batches:
			if !ok {
				return st.finish(ctx)
			}
			batch = b
		}
		for _, p := range batch {
			if err := st.send(ctx, p); err != nil {
				return err
			}
		}
		select {
		case responses <- arm.Response{}:
		case <-ctx.Done():
			return ctx.Err()
		}
	}
}

// servoStreamer executes a trajectory as ServoJ setpoints, one point at a
// time, checking each before it goes out: shape and timing
// (trajectoryStreamValidator), finite positions within the joint limits, a
// first point where the arm actually is, and no step faster than the arm can
// move. The caller owns the operation, and sends STOP when a stream that has
// sent a setpoint (sent) ends in an error.
//
// Pacing follows viam-ufactory-xarm's servo-mode streaming: the point after
// prevTime is sent at anchor+prevTime and told to take the interval to the
// next. A point that arrives late (a slow client, a stalled command
// connection) shifts the anchor by its lateness beyond servoPaceSlack instead
// of being sent at once, so a backlog is replayed at the planned pace rather
// than bunched up — which would drive the arm several times faster than the
// trajectory. Lateness within the slack leaves the schedule alone, so the
// next point is sent on time again.
type servoStreamer struct {
	s *viamFairinoFairino

	v        trajectoryStreamValidator
	prevDeg  [6]float64
	started  bool // the first (t=0) point has been checked
	anchor   time.Time
	sent     bool
	lastDeg  [6]float64
	prevTime time.Duration
}

// check validates p as the next point without sending anything.
func (st *servoStreamer) check(ctx context.Context, p arm.TrajectoryPoint) error {
	prevTime := st.v.lastTime
	if err := st.v.validate(p); err != nil {
		return err
	}
	if err := st.s.checkJointTarget(ctx, p.Positions); err != nil {
		return fmt.Errorf("trajectory point at %v: %w", p.Time, err)
	}
	deg := inputsToDeg(p.Positions)
	if !st.started {
		current, err := st.s.JointPositions(ctx, nil)
		if err != nil {
			return err
		}
		if d := referenceframe.InputsLinfDistance(current, p.Positions); d > streamStartToleranceRad {
			return fmt.Errorf("trajectory starts %.4f rad from the arm's current position (at most %v): "+
				"its first point must be where the arm is", d, streamStartToleranceRad)
		}
		st.started, st.prevDeg = true, deg
		return nil
	}
	interval := (p.Time - prevTime).Seconds()
	if interval > maxServoIntervalSec {
		return fmt.Errorf("trajectory points are %v apart; ServoJ needs them at most %v apart",
			p.Time-prevTime, time.Duration(maxServoIntervalSec*float64(time.Second)))
	}
	if interval < minServoIntervalSec {
		return fmt.Errorf("trajectory points are %v apart; ServoJ needs them at least %v apart",
			p.Time-prevTime, time.Duration(minServoIntervalSec*float64(time.Second)))
	}
	if speed := maxJointTravelDeg(st.prevDeg, deg) / interval; speed > maxServoStepSpeedDegsPerSec {
		return fmt.Errorf("trajectory point at %v needs %.0f°/s, beyond the arm's %v°/s", p.Time, speed, maxJointSpeedDegsPerSec)
	}
	st.prevDeg = deg
	return nil
}

// send checks p and, unless it is the first point (where the arm already is),
// sends it as a ServoJ and waits until the next point is due.
func (st *servoStreamer) send(ctx context.Context, p arm.TrajectoryPoint) error {
	if err := st.check(ctx, p); err != nil {
		return err
	}
	if p.Time == 0 {
		return nil
	}
	// The status feed is the only way to see a fault or an e-stop mid-stream,
	// so no feed means no more setpoints.
	fr, err := st.s.status.current()
	if err != nil {
		return err
	}
	if age := time.Since(fr.ReceivedAt); age > statusFeedLostTimeout {
		return fmt.Errorf("status stream stalled: no new frame for %v", age.Round(time.Millisecond))
	}
	if fr.EmergencyStop {
		return errors.New("Fairino controller e-stopped during a streamed trajectory")
	}
	if fr.ErrorCode != 0 {
		return fmt.Errorf("Fairino controller reported a fault during a streamed trajectory: %s", faultCodeString(fr.ErrorCode))
	}

	interval := p.Time - st.prevTime
	due := st.anchor.Add(st.prevTime)
	switch now := time.Now(); {
	case st.anchor.IsZero():
		st.anchor = now.Add(-st.prevTime)
	case now.Sub(due) > servoPaceSlack:
		late := now.Sub(due)
		st.anchor = st.anchor.Add(late - servoPaceSlack)
		if late > interval {
			st.s.logger.Warnw("streamed trajectory fell behind; replaying at its planned pace", "late", late, "point_time", p.Time)
		}
	}
	st.lastDeg = inputsToDeg(p.Positions)
	st.sent = true // from here the controller may be moving, even if the ack fails
	if err := st.s.cmd.servoJ(ctx, st.lastDeg, interval.Seconds()); err != nil {
		return fmt.Errorf("ServoJ at %v: %w", p.Time, err)
	}
	st.prevTime = p.Time
	if !sleepCtx(ctx, time.Until(st.anchor.Add(p.Time))) {
		return ctx.Err()
	}
	return nil
}

// finish waits for the arm to settle at the last setpoint sent, if any.
func (st *servoStreamer) finish(ctx context.Context) error {
	if !st.sent {
		return nil
	}
	return st.s.waitForMotionComplete(ctx, st.lastDeg, servoSettleTimeout)
}

// trajectoryStreamValidator checks the shape a streamed trajectory must have,
// one point at a time: the first at time zero and at rest, later ones strictly
// later. The same checks as viam-ufactory-xarm's, which are meant to move into
// RDK (RSDK-14275); the joint count is checkJointTarget's.
type trajectoryStreamValidator struct {
	seen     bool
	lastTime time.Duration
}

func (v *trajectoryStreamValidator) validate(p arm.TrajectoryPoint) error {
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
		v.seen = true
		return nil
	}
	if p.Time <= v.lastTime {
		return fmt.Errorf("trajectory point times must strictly increase, got %v after %v", p.Time, v.lastTime)
	}
	v.lastTime = p.Time
	return nil
}
