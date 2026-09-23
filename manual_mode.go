package viamfairino

import (
	"context"
	"sync"
	"time"

	arm "go.viam.com/rdk/components/arm"
)

// Properties reports optional arm features. Manual (drag-teach) mode is
// supported over the command port (see dragTeachSwitch, CMD_ID 333); cartesian
// commands are supported via the motion service (see MoveToPosition).
func (s *viamFairinoFairino) Properties(ctx context.Context, extra map[string]interface{}) (arm.Properties, error) {
	return arm.Properties{SupportManualMode: true, SupportCartesianCommands: true}, nil
}

// ManualMode reports whether the controller is in drag-teach mode, read from
// the status feed (robot_mode == drag). Drag can also be toggled from the
// pendant or end button, so this reflects hardware truth, not just our calls.
func (s *viamFairinoFairino) ManualMode(ctx context.Context, extra map[string]interface{}) (bool, error) {
	fr, err := s.latestFrame(ctx)
	if err != nil {
		return false, err
	}
	return fr.RobotMode == robotModeDrag, nil
}

// SetManualMode enters (manualMode) or exits drag-teach mode via dragTeachSwitch
// (command port, CMD_ID 333). When entering with enabledFor > 0, drag-teach is
// automatically exited after that duration; enabledFor == 0 leaves it on until
// an explicit exit.
func (s *viamFairinoFairino) SetManualMode(
	ctx context.Context, manualMode bool, enabledFor time.Duration, extra map[string]interface{},
) error {
	s.manualExit.cancel() // supersede any pending auto-exit
	if err := s.cmd.dragTeachSwitch(ctx, manualMode); err != nil {
		return err
	}
	if manualMode && enabledFor > 0 {
		s.manualExit.schedule(enabledFor, func() {
			ctx, cancel := context.WithTimeout(context.Background(), cmdReadTimeout)
			defer cancel()
			if err := s.cmd.dragTeachSwitch(ctx, false); err != nil {
				s.logger.Warnw("manual mode auto-exit failed", "err", err)
			}
		})
	}
	return nil
}

// exitTimer is a mutex-guarded time.AfterFunc used to auto-exit manual mode.
// Its zero value is ready to use.
type exitTimer struct {
	mu    sync.Mutex
	timer *time.Timer
}

func (t *exitTimer) schedule(d time.Duration, f func()) {
	t.mu.Lock()
	defer t.mu.Unlock()
	if t.timer != nil {
		t.timer.Stop()
	}
	t.timer = time.AfterFunc(d, f)
}

func (t *exitTimer) cancel() {
	t.mu.Lock()
	defer t.mu.Unlock()
	if t.timer != nil {
		t.timer.Stop()
		t.timer = nil
	}
}
