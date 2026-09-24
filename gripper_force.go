package viamfairino

import (
	"context"
	"errors"
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

// Speed and force are 1-100%: 0 would never move or never grip, so it's
// refused rather than stored as a default (config's 0 means "unset").
const minSpeedForcePercent = 1

// move handles {"command": "move", "position": N, "speed": N, "force": N}:
// position is required; speed/force, if given, become the defaults for later
// moves too.
func (g *fairinoGripper) move(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	pos, ok, err := intArg(cmd, "position", 0, 100)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, errors.New(`move needs a "position" from 0 to 100`)
	}
	speed, hasSpeed, err := intArg(cmd, "speed", minSpeedForcePercent, 100)
	if err != nil {
		return nil, err
	}
	force, hasForce, err := intArg(cmd, "force", minSpeedForcePercent, 100)
	if err != nil {
		return nil, err
	}
	g.setDefaults(speed, hasSpeed, force, hasForce)
	ctx, done := g.opMgr.New(ctx)
	defer done()
	if err := g.moveGripper(ctx, pos); err != nil {
		return nil, err
	}
	return map[string]any{"ok": true, "position": pos}, nil
}

// setDefaults updates the default speed and/or force, returning both.
func (g *fairinoGripper) setDefaults(speed int, hasSpeed bool, force int, hasForce bool) (int, int) {
	g.paramMu.Lock()
	defer g.paramMu.Unlock()
	if hasSpeed {
		g.speed = speed
	}
	if hasForce {
		g.force = force
	}
	return g.speed, g.force
}

// setSpeedForce handles {"command": "set_speed_force", "speed_percent": N,
// "force_percent": N}: either or both become the default for later moves.
func (g *fairinoGripper) setSpeedForce(cmd map[string]any) (map[string]any, error) {
	speed, hasSpeed, err := intArg(cmd, "speed_percent", minSpeedForcePercent, 100)
	if err != nil {
		return nil, err
	}
	force, hasForce, err := intArg(cmd, "force_percent", minSpeedForcePercent, 100)
	if err != nil {
		return nil, err
	}
	if !hasSpeed && !hasForce {
		return nil, errors.New(`set_speed_force needs "speed_percent" and/or "force_percent"`)
	}
	speed, force = g.setDefaults(speed, hasSpeed, force, hasForce)
	return map[string]any{"speed_percent": speed, "force_percent": force}, nil
}

// grabWithForce handles {"command": "grab_with_force", "position": N,
// "speed_percent": N, "force_percent": N}, each optional (defaulting to the
// grab position and current speed/force): a Grab at a one-off speed and force
// that aren't kept for later moves — viam-ufactory-xarm's grab_with_torque.
func (g *fairinoGripper) grabWithForce(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	speed, force := g.speedForce()
	pos := g.grabPos
	for _, arg := range []struct {
		key string
		lo  int
		dst *int
	}{{"position", 0, &pos}, {"speed_percent", minSpeedForcePercent, &speed}, {"force_percent", minSpeedForcePercent, &force}} {
		v, ok, err := intArg(cmd, arg.key, arg.lo, 100)
		if err != nil {
			return nil, err
		}
		if ok {
			*arg.dst = v
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
