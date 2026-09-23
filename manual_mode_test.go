package viamfairino

import (
	"context"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
)

func TestProperties(t *testing.T) {
	props, err := (&viamFairinoFairino{}).Properties(context.Background(), nil)
	if err != nil {
		t.Fatalf("Properties: %v", err)
	}
	if !props.SupportManualMode || !props.SupportCartesianCommands {
		t.Errorf("Properties = %+v, want both true", props)
	}
}

func TestManualModeFromStatus(t *testing.T) {
	for _, tc := range []struct {
		mode uint8
		want bool
	}{
		{robotModeAuto, false},
		{robotModeManual, false}, // pendant "manual" (jog) is not drag-teach
		{robotModeDrag, true},
	} {
		s := &viamFairinoFairino{
			logger: logging.NewTestLogger(t),
			status: &statusClient{frame: statusFrame{RobotMode: tc.mode}, have: true},
		}
		got, err := s.ManualMode(context.Background(), nil)
		if err != nil {
			t.Fatalf("robot_mode %d: ManualMode: %v", tc.mode, err)
		}
		if got != tc.want {
			t.Errorf("robot_mode %d: ManualMode = %v, want %v", tc.mode, got, tc.want)
		}
	}
}

func TestExitTimerFires(t *testing.T) {
	var et exitTimer
	done := make(chan struct{})
	et.schedule(10*time.Millisecond, func() { close(done) })
	select {
	case <-done:
	case <-time.After(time.Second):
		t.Fatal("scheduled timer never fired")
	}
}

func TestExitTimerCancel(t *testing.T) {
	var et exitTimer
	fired := make(chan struct{}, 1)
	et.schedule(20*time.Millisecond, func() { fired <- struct{}{} })
	et.cancel()
	select {
	case <-fired:
		t.Fatal("cancelled timer fired")
	case <-time.After(60 * time.Millisecond):
	}
}
