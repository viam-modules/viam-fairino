package viamfairino

import (
	"context"
	"errors"
	"fmt"
	"math"
)

// Digital outputs SetDO/SetToolDO can drive: the control box has 16 (status
// port cl_dgt_output_h/l), the tool flange 2 (tl_dgt_output_l, bits 0-1).
const (
	numBoxDOs  = 16
	numToolDOs = 2
)

// getError reports the controller's current fault: the status frame's fault
// code, plus GetRobotErrorCode's main/sub codes (manual §6.3.16).
func (s *viamFairinoFairino) getError(ctx context.Context) (map[string]interface{}, error) {
	fr, err := s.latestFrame(ctx)
	if err != nil {
		return nil, err
	}
	mainCode, subCode, err := s.cmd.getRobotErrorCode(ctx)
	if err != nil {
		return nil, err
	}
	return map[string]interface{}{
		"error_code":     float64(fr.ErrorCode),
		"fault":          faultCodeString(fr.ErrorCode),
		"main_code":      float64(mainCode),
		"sub_code":       float64(subCode),
		"emergency_stop": fr.EmergencyStop,
	}, nil
}

// getState is the short form of "status": what the arm is doing right now.
func (s *viamFairinoFairino) getState(ctx context.Context) (map[string]interface{}, error) {
	fr, err := s.latestFrame(ctx)
	if err != nil {
		return nil, err
	}
	return map[string]interface{}{
		"program_state":  programStateString(fr.ProgramState),
		"robot_mode":     robotModeString(fr.RobotMode),
		"motion_done":    fr.MotionDone,
		"emergency_stop": fr.EmergencyStop,
		"fault":          faultCodeString(fr.ErrorCode),
		"move_running":   s.opMgr.OpRunning() || s.planning(),
	}, nil
}

// getLoad returns the joint torques, N·m — viam-ufactory-xarm's "load".
func (s *viamFairinoFairino) getLoad(ctx context.Context) (map[string]interface{}, error) {
	fr, err := s.latestFrame(ctx)
	if err != nil {
		return nil, err
	}
	return map[string]interface{}{"load": floatSlice(fr.JointTorques[:])}, nil
}

// setDigitalOutput handles set_do / set_tool_do:
// {"command": "set_do", "index": 0-15, "on": true}.
func (s *viamFairinoFairino) setDigitalOutput(ctx context.Context, cmd map[string]interface{}, tool bool) (map[string]interface{}, error) {
	n := numBoxDOs
	if tool {
		n = numToolDOs
	}
	index, ok, err := intArg(cmd, "index", 0, n-1)
	if err != nil || !ok {
		return nil, fmt.Errorf(`%s needs an integer "index" from 0 to %d`, cmd["command"], n-1)
	}
	on, ok := cmd["on"].(bool)
	if !ok {
		return nil, fmt.Errorf(`%s needs a boolean "on"`, cmd["command"])
	}
	if err := s.cmd.setDO(ctx, tool, index, on); err != nil {
		return nil, err
	}
	return map[string]interface{}{"index": index, "on": on}, nil
}

var errUnknownCommand = errors.New("unknown command")

// intArg reads an optional integer in [lo, hi] from a DoCommand; ok is false
// when the key is absent, and anything else — a string, a fraction, NaN, out
// of range — is an error rather than a silent zero.
func intArg(cmd map[string]interface{}, key string, lo, hi int) (v int, ok bool, err error) {
	if _, present := cmd[key]; !present {
		return 0, false, nil
	}
	f, isNum := floatArg(cmd, key)
	// Written so NaN, which compares false against everything, fails too.
	if !isNum || !(f >= float64(lo) && f <= float64(hi)) || f != math.Trunc(f) {
		return 0, false, fmt.Errorf("%s must be an integer from %d to %d, got %v", key, lo, hi, cmd[key])
	}
	return int(f), true, nil
}
