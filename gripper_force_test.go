package viamfairino

import (
	"context"
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
