package viamfairino

import (
	"context"
	"fmt"
)

// Gripper vendors whose gripper_motion_done encoding is known (8083 manual
// Table 2-2 row 41). Selected by GripperConfig.Vendor.
const (
	gripperVendorDH      = "dh" // also "Dahuan"
	gripperVendorRobotiq = "robotiq"
	gripperVendorHuiling = "huiling" // also Tianji
)

// Robotiq's gripper_motion_done values.
const (
	robotiqMotionMoving         = 0 // movement not completed
	robotiqStoppedTouchOpening  = 1 // stopped: object touched while opening
	robotiqStoppedTouchClosing  = 2 // stopped: object touched while closing
	robotiqStoppedNoObjectFound = 3 // reached the position without touching anything
)

// decodeGripperHold interprets gripper_motion_done for the configured vendor.
// detectable is false for a vendor whose encoding can't say whether an object
// is held (Huiling/Tianji only report moving/done); holding is then false.
func decodeGripperHold(vendor string, motionDone uint8) (holding, dropped, detectable bool) {
	switch vendor {
	case gripperVendorRobotiq:
		return motionDone == robotiqStoppedTouchOpening || motionDone == robotiqStoppedTouchClosing, false, true
	case gripperVendorHuiling:
		return false, false, false
	default: // DH, the vendor this module was brought up against
		return motionDone == gripperMotionClamped, motionDone == gripperMotionDropped, true
	}
}

// percentArg reads an optional 0-100 percentage from a DoCommand; ok is false
// when the key is absent.
func percentArg(cmd map[string]any, key string) (v int, ok bool, err error) {
	f, ok := floatArg(cmd, key)
	if !ok {
		if _, present := cmd[key]; present {
			return 0, false, fmt.Errorf("%s must be a number", key)
		}
		return 0, false, nil
	}
	if f < 0 || f > 100 || f != float64(int(f)) {
		return 0, false, fmt.Errorf("%s must be an integer from 0 to 100, got %v", key, f)
	}
	return int(f), true, nil
}

// setSpeedForce handles {"command": "set_speed_force", "speed_percent": N,
// "force_percent": N}: either or both become the default for later moves.
func (g *fairinoGripper) setSpeedForce(cmd map[string]any) (map[string]any, error) {
	speed, hasSpeed, err := percentArg(cmd, "speed_percent")
	if err != nil {
		return nil, err
	}
	force, hasForce, err := percentArg(cmd, "force_percent")
	if err != nil {
		return nil, err
	}
	if !hasSpeed && !hasForce {
		return nil, fmt.Errorf(`set_speed_force needs "speed_percent" and/or "force_percent"`)
	}
	g.paramMu.Lock()
	if hasSpeed {
		g.speed = speed
	}
	if hasForce {
		g.force = force
	}
	speed, force = g.speed, g.force
	g.paramMu.Unlock()
	return map[string]any{"speed_percent": speed, "force_percent": force}, nil
}

// grabWithForce handles {"command": "grab_with_force", "position": N,
// "speed_percent": N, "force_percent": N}, each optional (defaulting to the
// grab position and current speed/force): a Grab at a one-off speed and force
// that aren't kept for later moves — viam-ufactory-xarm's grab_with_torque.
func (g *fairinoGripper) grabWithForce(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	speed, force := g.speedForce()
	pos := g.grabPos
	for key, dst := range map[string]*int{"position": &pos, "speed_percent": &speed, "force_percent": &force} {
		v, ok, err := percentArg(cmd, key)
		if err != nil {
			return nil, err
		}
		if ok {
			*dst = v
		}
	}

	ctx, done := g.opMgr.New(ctx)
	defer done()
	// Returns on a post-move frame, so the hold check reads this grab's result.
	if err := g.moveGripperWith(ctx, pos, speed, force); err != nil {
		return nil, err
	}
	st, err := g.IsHoldingSomething(ctx, nil)
	if err != nil {
		return nil, err
	}
	return map[string]any{"holding": st.IsHoldingSomething, "position": pos, "speed_percent": speed, "force_percent": force}, nil
}
