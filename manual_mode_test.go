package viamfairino

import (
	"context"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
)

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

// Switching to free-drive must not happen under a move still in flight: the
// move is cancelled (and the arm stopped) before DragTeachSwitch goes out.
func TestEnterManualModeCancelsMove(t *testing.T) {
	fc := newFakeController(t)
	fc.setReply(fakeFKReply)
	s, _ := newControllerTestArm(t, fc, movingFrame())
	moveErr := make(chan error, 1)
	go func() { moveErr <- s.MoveToJointPositions(context.Background(), testTarget(), nil) }()
	waitFor(t, "MoveJ", func() bool { return fc.count("MoveJ") == 1 })
	if err := s.SetManualMode(context.Background(), true, 0, nil); err != nil {
		t.Fatal(err)
	}
	if err := <-moveErr; err == nil {
		t.Error("move still succeeded after entering manual mode")
	}
	sent := fc.sent()
	if len(sent) < 2 || sent[len(sent)-2] != "STOP" || sent[len(sent)-1] != "DragTeachSwitch" {
		t.Errorf("want STOP then DragTeachSwitch last, got %v", sent)
	}
}

// DoCommand's drag-mode commands are manual mode too, so they supersede a
// pending auto-exit instead of being cut short by it.
func TestDragModeDoCommandCancelsAutoExit(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	if err := s.SetManualMode(context.Background(), true, 30*time.Millisecond, nil); err != nil {
		t.Fatal(err)
	}
	if _, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "enter_drag_mode"}); err != nil {
		t.Fatal(err)
	}
	time.Sleep(100 * time.Millisecond)
	if n := fc.count("DragTeachSwitch"); n != 2 {
		t.Errorf("sent %d DragTeachSwitch, want 2 (no auto-exit): %v", n, fc.sent())
	}
}
