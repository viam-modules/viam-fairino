// Fairino gripper component. The controller drives grippers over XML-RPC on
// TCP 20003 (xmlrpc.go) — a generic, percentage-based interface that works for
// any Fairino-configured gripper, so this one model covers the AG-105-145 and
// others. Model-specific bits (which physical gripper, its stroke) are config.
//
// NOTE ON TRANSPORT: the arm deliberately speaks only the documented 8080
// command protocol (see command.go). Grippers are not available there — the
// vendor SDK itself uses XML-RPC on 20003 for them — so this component uses
// XML-RPC. That's a deliberate exception to the "8080 only" rule, scoped to the
// gripper.
package viamfairino

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// Gripper is the model triple for the Fairino gripper component. It shares the
// arm's namespace/family (see Fairino in module.go).
var Gripper = resource.NewModel("3a94d2f9082b44ad8569a9d8a1330050", "viam-fairino", "gripper")

func init() {
	resource.RegisterComponent(gripper.API, Gripper,
		resource.Registration[gripper.Gripper, *GripperConfig]{
			Constructor: newFairinoGripper,
		},
	)
}

// GripperConfig configures a Fairino gripper attached to a controller.
type GripperConfig struct {
	// Host is the controller IP; defaults to 192.168.58.2 (factory default).
	Host string `json:"host,omitempty"`

	// Company/Device/SoftwareVersion/Bus identify the physical gripper for
	// SetGripperConfig. They only take effect when Configure is true. The
	// numeric codes come from the controller's WebApp gripper dropdown; read
	// the currently-configured values any time with DoCommand{"gripper_config"}.
	Company         int `json:"company,omitempty"`
	Device          int `json:"device,omitempty"`
	SoftwareVersion int `json:"software_version,omitempty"`
	Bus             int `json:"bus,omitempty"`

	// Configure, when true, pushes SetGripperConfig(Company,Device,...) at
	// startup so the module configures the gripper itself. When false (the
	// default), the gripper is assumed already configured on the controller
	// (teach pendant / WebApp) and is only activated.
	Configure bool `json:"configure,omitempty"`

	// Activate controls whether ActGripper is called at startup to activate
	// the gripper. Defaults to true; set false to skip (e.g. for bring-up).
	Activate *bool `json:"activate,omitempty"`

	// SpeedPercent / ForcePercent are the vel/force passed to MoveGripper
	// (0-100). Default 50 each.
	SpeedPercent int `json:"speed_percent,omitempty"`
	ForcePercent int `json:"force_percent,omitempty"`

	// MaxTimeMs is MoveGripper's max_time (ms) before it gives up. Default 30000.
	MaxTimeMs int `json:"max_time_ms,omitempty"`

	// OpenPosition / GrabPosition are the MoveGripper positions (0-100) used by
	// Open() and Grab(). Defaults: open 100, grab 0. Pointers so an explicit 0
	// is distinguishable from unset.
	OpenPosition *int `json:"open_position,omitempty"`
	GrabPosition *int `json:"grab_position,omitempty"`
}

// Validate checks ranges. No dependencies.
func (cfg *GripperConfig) Validate(path string) ([]string, []string, error) {
	for name, v := range map[string]int{"speed_percent": cfg.SpeedPercent, "force_percent": cfg.ForcePercent} {
		if v < 0 || v > 100 {
			return nil, nil, fmt.Errorf("%s: %s must be between 0 and 100, got %d", path, name, v)
		}
	}
	for name, p := range map[string]*int{"open_position": cfg.OpenPosition, "grab_position": cfg.GrabPosition} {
		if p != nil && (*p < 0 || *p > 100) {
			return nil, nil, fmt.Errorf("%s: %s must be between 0 and 100, got %d", path, name, *p)
		}
	}
	return nil, nil, nil
}

// gripperIndex is the controller's gripper selector, passed to every gripper
// call. The controller can address multiple grippers, but a machine has one, so
// this is fixed at 1 (matching every vendor SDK example) rather than exposed as
// config.
const gripperIndex = 1

type fairinoGripper struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	client *xmlrpcClient
	opMgr  *operation.SingleOperationManager

	speed   int
	force   int
	maxTime int
	openPos int
	grabPos int
}

func newFairinoGripper(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	conf, err := resource.NativeConfig[*GripperConfig](rawConf)
	if err != nil {
		return nil, err
	}

	host := conf.Host
	if host == "" {
		host = defaultHost
	}
	g := &fairinoGripper{
		name:    rawConf.ResourceName(),
		logger:  logger,
		client:  newXMLRPCClient(host, xmlrpcPort),
		opMgr:   operation.NewSingleOperationManager(),
		speed:   valueOr(conf.SpeedPercent, 50),
		force:   valueOr(conf.ForcePercent, 50),
		maxTime: valueOr(conf.MaxTimeMs, 30000),
		openPos: ptrOr(conf.OpenPosition, 100),
		grabPos: ptrOr(conf.GrabPosition, 0),
	}

	if conf.Configure {
		if _, err := g.client.callChecked(ctx, "SetGripperConfig", conf.Company, conf.Device, conf.SoftwareVersion, conf.Bus); err != nil {
			return nil, fmt.Errorf("SetGripperConfig(company=%d device=%d): %w", conf.Company, conf.Device, err)
		}
		logger.CInfow(ctx, "configured gripper", "company", conf.Company, "device", conf.Device,
			"software_version", conf.SoftwareVersion, "bus", conf.Bus)
	}

	if conf.Activate == nil || *conf.Activate {
		if _, err := g.client.callChecked(ctx, "ActGripper", gripperIndex, 1); err != nil {
			// Don't fail construction — surface it so the resource comes up and
			// the user can inspect/config via DoCommand. Activation typically
			// fails only when no gripper is configured on the controller.
			logger.CWarnw(ctx, "gripper activation (ActGripper) failed; is a gripper configured on the controller?",
				"index", gripperIndex, "err", err)
		}
	}

	return g, nil
}

// moveGripper issues a blocking MoveGripper to pos (0-100). type=0 / rot* = 0
// are the parallel two-finger (non-rotary) values; rotNum is a double.
func (g *fairinoGripper) moveGripper(ctx context.Context, pos int) error {
	_, err := g.client.callChecked(ctx, "MoveGripper",
		gripperIndex, pos, g.speed, g.force, g.maxTime, 1 /*block*/, 0 /*type*/, 0.0 /*rotNum*/, 0 /*rotVel*/, 0 /*rotTorque*/)
	return err
}

func (g *fairinoGripper) Open(ctx context.Context, extra map[string]any) error {
	ctx, done := g.opMgr.New(ctx)
	defer done()
	return g.moveGripper(ctx, g.openPos)
}

func (g *fairinoGripper) Grab(ctx context.Context, extra map[string]any) (bool, error) {
	ctx, done := g.opMgr.New(ctx)
	defer done()
	if err := g.moveGripper(ctx, g.grabPos); err != nil {
		return false, err
	}
	status, err := g.IsHoldingSomething(ctx, extra)
	if err != nil {
		return false, err
	}
	return status.IsHoldingSomething, nil
}

// IsHoldingSomething reports whether the gripper is holding an object, from
// GetGripperMotionDone's status field.
//
// The exact status encoding isn't documented in what we have; based on the
// common Fairino/OEM convention it's 0=moving, 1=reached (no object),
// 2=object gripped. We treat >=2 as holding. This needs confirmation against a
// physically-configured gripper (none was configured during bring-up); the raw
// (fault,status) is returned in Meta so it can be verified in the field.
func (g *fairinoGripper) IsHoldingSomething(ctx context.Context, extra map[string]any) (gripper.HoldingStatus, error) {
	fault, status, err := g.motionDone(ctx)
	if err != nil {
		return gripper.HoldingStatus{}, err
	}
	return gripper.HoldingStatus{
		IsHoldingSomething: status >= 2,
		Meta:               map[string]any{"fault": fault, "status": status},
	}, nil
}

// motionDone returns GetGripperMotionDone's (fault, status).
func (g *fairinoGripper) motionDone(ctx context.Context) (fault, status int, err error) {
	vals, err := g.client.callChecked(ctx, "GetGripperMotionDone")
	if err != nil {
		return 0, 0, err
	}
	return intAt(vals, 0), intAt(vals, 1), nil
}

// IsMoving reflects whether this component has a gripper operation in flight.
// Open/Grab issue blocking MoveGripper calls, so this is true only while one is
// running (hardware has no reliable "is moving" query — see IsHoldingSomething).
func (g *fairinoGripper) IsMoving(ctx context.Context) (bool, error) {
	return g.opMgr.OpRunning(), nil
}

// Stop cancels any in-flight Open/Grab. The controller's gripper API has no
// explicit stop, so this cancels our blocking call's context; it does not
// forcibly halt the gripper mid-motion.
func (g *fairinoGripper) Stop(ctx context.Context, extra map[string]any) error {
	g.opMgr.CancelRunning(ctx)
	return nil
}

// Geometries returns no geometry (the gripper is represented as a frameless
// end-effector for now).
func (g *fairinoGripper) Geometries(ctx context.Context, extra map[string]any) ([]spatialmath.Geometry, error) {
	return nil, nil
}

// Kinematics returns a trivial (0-DoF) model — the gripper has no articulated
// joints exposed to the frame system.
func (g *fairinoGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return referenceframe.NewSimpleModel(g.name.ShortName()), nil
}

func (g *fairinoGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return []referenceframe.Input{}, nil
}

func (g *fairinoGripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return nil
}

func (g *fairinoGripper) Name() resource.Name { return g.name }

// Status returns a lightweight status map (the gripper's current position and
// raw motion fault/status), best-effort — errors are reported in the map
// rather than failing, so status polling stays cheap.
func (g *fairinoGripper) Status(ctx context.Context) (map[string]any, error) {
	pos, err := g.client.callChecked(ctx, "GetGripperCurPosition")
	if err != nil {
		return map[string]any{"error": err.Error()}, nil
	}
	fault, status, err := g.motionDone(ctx)
	if err != nil {
		return map[string]any{"error": err.Error()}, nil
	}
	return map[string]any{
		"position":      intAt(pos, 1),
		"motion_fault":  fault,
		"motion_status": status,
	}, nil
}

func (g *fairinoGripper) Close(ctx context.Context) error { return nil }

// DoCommand exposes raw controller queries/actions, useful for discovering the
// gripper's config codes and validating status semantics against real hardware.
// Supported commands (key "command"):
//   - "gripper_config": GetGripperConfig -> [company, device, software_version, bus]
//   - "software_version": GetSoftwareVersion
//   - "status": current position + (fault,status) from GetGripperMotionDone
//   - "activate" / "deactivate": ActGripper(index, 1|0)
//   - "set_config": SetGripperConfig using company/device/software_version/bus keys
//   - "move": MoveGripper to "position" (0-100), optional "speed"/"force"
func (g *fairinoGripper) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	command, _ := cmd["command"].(string)
	switch command {
	case "gripper_config":
		vals, err := g.client.callChecked(ctx, "GetGripperConfig")
		if err != nil {
			return nil, err
		}
		return map[string]any{
			"company": intAt(vals, 0), "device": intAt(vals, 1),
			"software_version": intAt(vals, 2), "bus": intAt(vals, 3),
		}, nil
	case "software_version":
		vals, err := g.client.callChecked(ctx, "GetSoftwareVersion")
		if err != nil {
			return nil, err
		}
		return map[string]any{"values": vals}, nil
	case "status":
		pos, err := g.client.callChecked(ctx, "GetGripperCurPosition")
		if err != nil {
			return nil, err
		}
		fault, status, err := g.motionDone(ctx)
		if err != nil {
			return nil, err
		}
		return map[string]any{
			"position": intAt(pos, 1), "position_fault": intAt(pos, 0),
			"motion_fault": fault, "motion_status": status,
		}, nil
	case "activate", "deactivate":
		act := 1
		if command == "deactivate" {
			act = 0
		}
		if _, err := g.client.callChecked(ctx, "ActGripper", gripperIndex, act); err != nil {
			return nil, err
		}
		return map[string]any{"ok": true}, nil
	case "set_config":
		if _, err := g.client.callChecked(ctx, "SetGripperConfig",
			intFromCmd(cmd, "company"), intFromCmd(cmd, "device"),
			intFromCmd(cmd, "software_version"), intFromCmd(cmd, "bus")); err != nil {
			return nil, err
		}
		return map[string]any{"ok": true}, nil
	case "move":
		pos := intFromCmd(cmd, "position")
		if s := intFromCmd(cmd, "speed"); s > 0 {
			g.speed = s
		}
		if f := intFromCmd(cmd, "force"); f > 0 {
			g.force = f
		}
		if err := g.moveGripper(ctx, pos); err != nil {
			return nil, err
		}
		return map[string]any{"ok": true, "position": pos}, nil
	default:
		return nil, fmt.Errorf("unknown command %q (supported: gripper_config, software_version, status, activate, deactivate, set_config, move)", command)
	}
}

// small helpers

func valueOr(v, def int) int {
	if v == 0 {
		return def
	}
	return v
}

func ptrOr(p *int, def int) int {
	if p == nil {
		return def
	}
	return *p
}

func intAt(vals []any, i int) int {
	if i < len(vals) {
		if n, ok := vals[i].(int); ok {
			return n
		}
	}
	return 0
}

func intFromCmd(cmd map[string]any, key string) int {
	switch v := cmd[key].(type) {
	case float64: // JSON numbers arrive as float64
		return int(v)
	case int:
		return v
	}
	return 0
}
