package viamfairino

import (
	"context"
	"errors"
	"testing"
)

func TestDoCommandGetError(t *testing.T) {
	fc := newFakeController(t)
	fc.setReply(func(cmdID int, _ string) string {
		if cmdID == cmdIDGetRobotErrorCode {
			return "3,7"
		}
		return "1"
	})
	s, _ := newControllerTestArm(t, fc, statusFrame{ErrorCode: 3})
	resp, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "get_error"})
	if err != nil {
		t.Fatal(err)
	}
	if resp["main_code"] != 3.0 || resp["sub_code"] != 7.0 || resp["fault"] != "collision failure" {
		t.Errorf("got %v", resp)
	}
}

func TestDoCommandStateAndLoad(t *testing.T) {
	s, _ := newControllerTestArm(t, newFakeController(t), statusFrame{
		MotionDone:   true,
		ProgramState: programStateRun,
		JointTorques: [6]float64{1, 2, 3, 4, 5, 6},
	})
	state, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "get_state"})
	if err != nil {
		t.Fatal(err)
	}
	if state["motion_done"] != true || state["move_running"] != false || state["program_state"] != programStateString(programStateRun) {
		t.Errorf("get_state: got %v", state)
	}
	load, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "load"})
	if err != nil {
		t.Fatal(err)
	}
	if got, ok := load["load"].([]interface{}); !ok || len(got) != 6 || got[5] != 6.0 {
		t.Errorf("load: got %v", load)
	}
}

func TestDoCommandSetDigitalOutputs(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	for _, cmd := range []map[string]interface{}{
		{"command": "set_do", "index": 15.0, "on": true},
		{"command": "set_tool_do", "index": 1.0, "on": false},
	} {
		if _, err := s.DoCommand(context.Background(), cmd); err != nil {
			t.Fatalf("%v: %v", cmd, err)
		}
	}
	fc.mu.Lock()
	got := append([]string(nil), fc.commands...)
	fc.mu.Unlock()
	want := []string{"SetDO(15,1,0,0)", "SetToolDO(1,0,0,0)"}
	if len(got) != 2 || got[0] != want[0] || got[1] != want[1] {
		t.Errorf("got %v want %v", got, want)
	}

	for _, bad := range []map[string]interface{}{
		{"command": "set_do", "index": 16.0, "on": true},
		{"command": "set_do", "index": 1.5, "on": true},
		{"command": "set_tool_do", "index": 2.0, "on": true},
		{"command": "set_do", "index": 0.0},
		{"command": "set_do", "on": true},
	} {
		if _, err := s.DoCommand(context.Background(), bad); err == nil {
			t.Errorf("%v: expected an error", bad)
		}
	}
	if n := len(fc.sent()); n != 2 {
		t.Errorf("invalid commands reached the controller: %v", fc.sent())
	}
}

func TestDoCommandUnknown(t *testing.T) {
	s, _ := newControllerTestArm(t, newFakeController(t), restingFrame([6]float64{}))
	if _, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "nope"}); !errors.Is(err, errUnknownCommand) {
		t.Errorf("got %v, want errUnknownCommand", err)
	}
}
