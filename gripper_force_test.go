package viamfairino

import (
	"context"
	"errors"
	"testing"
	"time"
)

func TestDecodeGripperHold(t *testing.T) {
	for _, tc := range []struct {
		vendor                       string
		motionDone                   uint8
		holding, dropped, detectable bool
	}{
		{"", gripperMotionClamped, true, false, true},
		{gripperVendorDH, gripperMotionDropped, false, true, true},
		{gripperVendorDH, gripperMotionStopped, false, false, true},
		{gripperVendorRobotiq, robotiqStoppedTouchClosing, true, false, true},
		{gripperVendorRobotiq, robotiqStoppedTouchOpening, true, false, true},
		{gripperVendorRobotiq, robotiqStoppedNoObjectFound, false, false, true},
		{gripperVendorHuiling, 1, false, false, false},
	} {
		h, d, det := decodeGripperHold(tc.vendor, tc.motionDone)
		if h != tc.holding || d != tc.dropped || det != tc.detectable {
			t.Errorf("%q/%d: got holding=%v dropped=%v detectable=%v", tc.vendor, tc.motionDone, h, d, det)
		}
	}
}

// liveFeed republishes fr every status period until the test ends, as a live
// controller would, so moves waiting on fresh frames can finish.
func liveFeed(t *testing.T, sc *statusClient, fr statusFrame) {
	t.Helper()
	stop := make(chan struct{})
	done := make(chan struct{})
	t.Cleanup(func() { close(stop); <-done })
	go func() {
		defer close(done)
		for {
			setTestFrame(sc, fr)
			select {
			case <-stop:
				return
			case <-time.After(20 * time.Millisecond):
			}
		}
	}()
}

func TestGripperSetSpeedForce(t *testing.T) {
	g := newCommandGripper(t, newFakeController(t), frameWith(t, gripperMotionStopped, 50))
	resp, err := g.DoCommand(context.Background(), map[string]any{"command": "set_speed_force", "force_percent": 80.0})
	if err != nil {
		t.Fatal(err)
	}
	if resp["speed_percent"] != 50 || resp["force_percent"] != 80 {
		t.Errorf("got %v", resp)
	}
	for _, bad := range []map[string]any{
		{"command": "set_speed_force"},
		{"command": "set_speed_force", "speed_percent": 101.0},
		{"command": "set_speed_force", "force_percent": "hard"},
	} {
		if _, err := g.DoCommand(context.Background(), bad); err == nil {
			t.Errorf("%v: expected an error", bad)
		}
	}
}

// grab_with_force uses its own speed/force for the one move, and doesn't
// change the defaults.
func TestGripperGrabWithForce(t *testing.T) {
	fc := newFakeController(t)
	g := newCommandGripper(t, fc, frameWith(t, gripperMotionClamped, 20))
	liveFeed(t, g.status, frameWith(t, gripperMotionClamped, 20))
	resp, err := g.DoCommand(context.Background(), map[string]any{
		"command": "grab_with_force", "position": 10.0, "speed_percent": 30.0, "force_percent": 90.0,
	})
	if err != nil {
		t.Fatal(err)
	}
	if resp["holding"] != true {
		t.Errorf("got %v", resp)
	}
	fc.mu.Lock()
	got := append([]string(nil), fc.commands...)
	fc.mu.Unlock()
	if want := "MoveGripper(1,10,30,90,30000,1,0,0,0,0)"; len(got) != 1 || got[0] != want {
		t.Errorf("got %v want [%s]", got, want)
	}
	if speed, force := g.speedForce(); speed != 50 || force != 50 {
		t.Errorf("defaults changed to %d/%d", speed, force)
	}
}

// Status must decode gripper_motion_done the same way IsHoldingSomething
// does, per vendor.
func TestGripperStatusDecodesPerVendor(t *testing.T) {
	g := newTestGripper(t, frameWith(t, robotiqStoppedNoObjectFound, 100))
	g.vendor = gripperVendorRobotiq
	st, err := g.Status(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if st["holding"] != false || st["dropped"] != false || st["hold_detection"] != true {
		t.Errorf("robotiq 'no object found': got %v", st)
	}
	g = newTestGripper(t, frameWith(t, gripperMotionClamped, 20))
	g.vendor = gripperVendorHuiling
	if st, _ := g.Status(context.Background()); st["holding"] != false || st["hold_detection"] != false {
		t.Errorf("huiling can't detect a hold: got %v", st)
	}
}

// DoCommand integers are validated, not coerced: "50" isn't position 0
// (fully closed), and a 0% or 150% speed is refused rather than kept as the
// default for every later move.
func TestGripperDoCommandValidation(t *testing.T) {
	fc := newFakeController(t)
	g := newCommandGripper(t, fc, frameWith(t, gripperMotionStopped, 50))
	for _, bad := range []map[string]any{
		{"command": "move"},
		{"command": "move", "position": "50"},
		{"command": "move", "position": 50.0, "speed": 150.0},
		{"command": "move", "position": 50.0, "force": 0.0},
		{"command": "set_speed_force", "speed_percent": 0.0},
		{"command": "grab_with_force", "force_percent": 0.0},
	} {
		if _, err := g.DoCommand(context.Background(), bad); err == nil {
			t.Errorf("%v: expected an error", bad)
		}
	}
	if sent := fc.sent(); len(sent) != 0 {
		t.Errorf("sent %v for invalid commands", sent)
	}
	if speed, force := g.speedForce(); speed != 50 || force != 50 {
		t.Errorf("defaults changed to %d/%d", speed, force)
	}
	if _, err := g.DoCommand(context.Background(), map[string]any{"command": "nope"}); !errors.Is(err, errUnknownCommand) {
		t.Errorf("unknown command: got %v", err)
	}
}
