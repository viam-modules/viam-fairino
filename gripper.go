// Fairino gripper component. Gripper control is percentage-based (0-100% open)
// and model-agnostic, so this one model covers the AG-105-145 and any other
// controller-configured gripper; model-specific bits (which physical gripper,
// its stroke) are config.
//
// TRANSPORT: same two documented ports as the arm — commands on 8080
// (command.go, manual chapter 7.1) and state on 8083 (status.go, Table 2-2
// rows 41 and 56-63). An earlier revision of this component spoke XML-RPC on
// TCP 20003 because that is what the vendor SDK does, on the mistaken premise
// that grippers were absent from the documented command protocol. They are not:
// chapter 7.1 documents SetGripperConfig/GetGripperConfig/ActGripper/
// MoveGripper with the same names and argument order the SDK uses.
//
// The split between the two ports is not arbitrary. The command port has no
// gripper *readback* at all — no GetGripperMotionDone exists there — because
// the controller publishes gripper position, activation, fault, and
// motion-done on the status feed instead. Reading those over XML-RPC is what
// produced the "fault=1 even when healthy" confusion during bring-up.
package viamfairino

import (
	"context"
	"fmt"
	"sync"
	"time"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// Gripper is the model triple for the Fairino gripper component. It shares the
// arm's namespace/family (see Fairino in module.go).
var Gripper = resource.NewModel("viam-soleng", "viam-fairino", "gripper")

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

	// ActivateWaitMs is how long to wait after ActGripper for the gripper to
	// finish its activation homing before it will accept a move. Activating
	// (and resetting) drives a physical homing motion that takes seconds;
	// moving before it completes fails with controller errcode 73 ("gripper
	// movement error"). Default 3000.
	ActivateWaitMs int `json:"activate_wait_ms,omitempty"`

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

	// StrokeMM is the gripper's total finger travel (fully closed to fully
	// open) in mm, used to build the kinematic model so the fingers render
	// open/closed in the 3D scene. Defaults to 145 (the AG-105-145); set it for
	// other grippers.
	StrokeMM float64 `json:"stroke_mm,omitempty"`
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

// errcodeGripperMove is the controller errcode for "gripper movement error"
// ("check whether the communication status of the gripper is normal"). It
// comes back from MoveGripper when the gripper isn't activated/ready, so we
// treat it as a signal to re-activate and retry.
const errcodeGripperMove = 73

// moveAckMargin is the slack added to MaxTimeMs when bounding a blocking
// MoveGripper, covering the controller's own overhead past the move's timeout.
const moveAckMargin = 5 * time.Second

// gripperSettleTimeout bounds how long Grab waits for the status feed (8083) to
// publish a post-move frame before reading hold state. The blocking MoveGripper
// ack (8080) can beat the status feed by a frame or two, so a hold check run the
// instant the move returns can read a stale, pre-clamp gripper_motion_done.
const gripperSettleTimeout = 2 * time.Second

type fairinoGripper struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	host   string
	cmd    *commandClient
	status *statusClient
	opMgr  *operation.SingleOperationManager

	model    referenceframe.Model
	strokeMM float64

	speed        int
	force        int
	maxTime      int
	openPos      int
	grabPos      int
	activateWait time.Duration

	actMu sync.Mutex // serializes (re)activation
}

func newFairinoGripper(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	conf, err := resource.NativeConfig[*GripperConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewFairinoGripper(ctx, rawConf.ResourceName(), conf, logger)
}

// NewFairinoGripper builds a Fairino gripper from an already-parsed config. It
// is the shared core of the registered constructor, exported so tools (e.g.
// cmd/fairino-test) can drive a real gripper without going through resource
// registration.
func NewFairinoGripper(ctx context.Context, name resource.Name, conf *GripperConfig, logger logging.Logger) (gripper.Gripper, error) {
	host := conf.Host
	if host == "" {
		host = defaultHost
	}
	strokeMM := gripperStrokeMM
	if conf.StrokeMM > 0 {
		strokeMM = conf.StrokeMM
	}
	model, err := makeGripperModel(name.ShortName(), strokeMM)
	if err != nil {
		return nil, err
	}
	g := &fairinoGripper{
		name:   name,
		logger: logger,
		host:   host,
		// SHARE both connections with the arm: the command port (8080) accepts
		// only a tiny number of connections and the status port (8083) admits
		// one client, so each resource holding its own would starve the other
		// ("arm can't connect" once the gripper is added). Both are
		// reference-counted per host:port (command.go / status.go).
		cmd:          acquireCommandClient(host, commandPort, logger),
		status:       acquireStatusClient(ctx, host, statusPort, logger),
		opMgr:        operation.NewSingleOperationManager(),
		model:        model,
		strokeMM:     strokeMM,
		speed:        valueOr(conf.SpeedPercent, 50),
		force:        valueOr(conf.ForcePercent, 50),
		maxTime:      valueOr(conf.MaxTimeMs, 30000),
		openPos:      ptrOr(conf.OpenPosition, 100),
		grabPos:      ptrOr(conf.GrabPosition, 0),
		activateWait: time.Duration(valueOr(conf.ActivateWaitMs, 3000)) * time.Millisecond,
	}

	if conf.Configure {
		if err := g.cmd.setGripperConfig(ctx, conf.Company, conf.Device, conf.SoftwareVersion, conf.Bus); err != nil {
			releaseStatusClient(host, statusPort)
			releaseCommandClient(host, commandPort)
			return nil, fmt.Errorf("SetGripperConfig(company=%d device=%d): %w", conf.Company, conf.Device, err)
		}
		logger.CInfow(ctx, "configured gripper", "company", conf.Company, "device", conf.Device,
			"software_version", conf.SoftwareVersion, "bus", conf.Bus)
	}

	if conf.Activate == nil || *conf.Activate {
		if err := g.activate(ctx, false); err != nil {
			// Don't fail construction — surface it so the resource comes up and
			// the user can inspect/config via DoCommand. Activation typically
			// fails only when no gripper is configured on the controller.
			logger.CWarnw(ctx, "gripper activation (ActGripper) failed; is a gripper configured on the controller?",
				"index", gripperIndex, "err", err)
		}
	}

	return g, nil
}

// activate activates the gripper and waits activateWait for its activation
// homing to finish before it will accept moves. When reset is true it first
// issues an ActGripper reset (action 0) and waits — used to recover a gripper
// that has dropped out or faulted (this is what the vendor examples do before
// re-activating). Serialized so a move-triggered re-activation can't race
// startup activation.
func (g *fairinoGripper) activate(ctx context.Context, reset bool) error {
	g.actMu.Lock()
	defer g.actMu.Unlock()

	if reset {
		if err := g.cmd.actGripper(ctx, gripperIndex, 0); err != nil {
			return fmt.Errorf("ActGripper reset: %w", err)
		}
		if !sleepCtx(ctx, g.activateWait) {
			return ctx.Err()
		}
	}
	if err := g.cmd.actGripper(ctx, gripperIndex, 1); err != nil {
		return fmt.Errorf("ActGripper activate: %w", err)
	}
	if !sleepCtx(ctx, g.activateWait) {
		return ctx.Err()
	}
	return nil
}

// moveGripper issues a blocking MoveGripper to pos (0-100), so the call returns
// only once the fingers have reached position (or maxTime elapses) — Open/Grab
// depend on that, and Grab's hold check must run after the motion completes.
//
// If the controller reports errcode 73 ("gripper movement error"), the gripper
// is not activated/ready; we reset+re-activate it once and retry, since this
// happens when the gripper wasn't activated in time or dropped out.
func (g *fairinoGripper) moveGripper(ctx context.Context, pos int) error {
	err := g.moveGripperOnce(ctx, pos)
	if !isControllerErrcode(err, errcodeGripperMove) {
		return err
	}
	g.logger.CWarnw(ctx, "MoveGripper failed with gripper-movement error (73); re-activating gripper and retrying", "pos", pos)
	if aerr := g.activate(ctx, true); aerr != nil {
		g.logger.CWarnw(ctx, "gripper re-activation failed", "err", aerr)
	}
	err = g.moveGripperOnce(ctx, pos)
	if isControllerErrcode(err, errcodeGripperMove) {
		return fmt.Errorf("gripper move failed (controller errcode 73, gripper movement error): confirm a "+
			"gripper is configured and its peripheral protocol is enabled on the controller (WebApp/teach "+
			"pendant), and that it is powered and communicating: %w", err)
	}
	return err
}

// moveGripperOnce issues a single blocking MoveGripper to pos (0-100). block=0
// is blocking (per the vendor SDK: 0-blocking, 1-non-blocking); type=0 / rot*
// are the parallel two-finger (non-rotary) values; rotNum is a double.
func (g *fairinoGripper) moveGripperOnce(ctx context.Context, pos int) error {
	// A blocking move isn't acknowledged until the fingers stop, which can take
	// up to MaxTimeMs — longer than the command client's default 10s ack
	// timeout. Give the call a deadline that covers the motion when the caller
	// hasn't set one, or the read times out mid-move and drops the connection.
	if _, ok := ctx.Deadline(); !ok {
		var cancel context.CancelFunc
		ctx, cancel = context.WithTimeout(ctx, time.Duration(g.maxTime)*time.Millisecond+moveAckMargin)
		defer cancel()
	}
	return g.cmd.moveGripper(ctx, moveGripperParams{
		Index:   gripperIndex,
		Pos:     pos,
		Vel:     g.speed,
		Force:   g.force,
		MaxTime: g.maxTime,
		Block:   0, // blocking: the ack means the fingers have stopped
		Type:    0, // parallel two-finger, not rotary
	})
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
	// The 8080 move ack can arrive a frame or two before the 8083 status feed
	// reflects the clamp, so wait for a fresh, settled frame before reading hold
	// state — otherwise a real grab reads stale (pre-clamp) and reports false.
	g.waitForGripperSettled(ctx, time.Now())
	status, err := g.IsHoldingSomething(ctx, extra)
	if err != nil {
		return false, err
	}
	return status.IsHoldingSomething, nil
}

// waitForGripperSettled polls the status feed until it publishes a frame
// captured after `after` whose gripper motion has stopped (not "moving"), so a
// hold check run next reflects the move just issued rather than a stale
// pre-move frame. Returns once settled or after gripperSettleTimeout; callers
// read the resulting state via the status feed as usual.
func (g *fairinoGripper) waitForGripperSettled(ctx context.Context, after time.Time) {
	deadline := time.Now().Add(gripperSettleTimeout)
	for time.Now().Before(deadline) && ctx.Err() == nil {
		fr, ok, _ := g.status.latest()
		if ok && fr.GripperValid && fr.ReceivedAt.After(after) && fr.GripperMotionDone != gripperMotionMoving {
			return
		}
		if !sleepCtx(ctx, 20*time.Millisecond) {
			return
		}
	}
}

// IsHoldingSomething reports whether the gripper is holding an object, from the
// status feed's gripper_motion_done field (Table 2-2 row 41).
//
// For DH grippers that field is 0=moving, 1=stopped with nothing clamped,
// 2=stopped with an object clamped, 3=object clamped and then dropped. Only 2
// means "holding": 3 is the workpiece-dropped signal, so treating it as holding
// (as an earlier `status >= 2` did) would report a dropped part as held.
//
// The raw value is returned in Meta so a field failure can be diagnosed without
// a code change, and so a non-DH gripper's different encoding is visible.
func (g *fairinoGripper) IsHoldingSomething(ctx context.Context, extra map[string]any) (gripper.HoldingStatus, error) {
	fr, err := g.gripperFrame(ctx)
	if err != nil {
		return gripper.HoldingStatus{}, err
	}
	return gripper.HoldingStatus{
		IsHoldingSomething: fr.GripperMotionDone == gripperMotionClamped,
		Meta: map[string]any{
			"motion_done": fr.GripperMotionDone,
			"dropped":     fr.GripperMotionDone == gripperMotionDropped,
			"fault":       fr.GripperFault,
			"active":      fr.GripperActive,
			"position":    fr.GripperPosition,
		},
	}, nil
}

// gripperFrame returns the latest status frame, blocking briefly if none has
// arrived yet, and fails if the frame is too short to carry the gripper fields
// — better an explicit error than silently reporting a zeroed gripper.
func (g *fairinoGripper) gripperFrame(ctx context.Context) (statusFrame, error) {
	fr, ok, _ := g.status.latest()
	if !ok {
		var err error
		fr, err = g.status.waitForFrame(ctx)
		if err != nil {
			return statusFrame{}, err
		}
	}
	if !fr.GripperValid {
		return statusFrame{}, fmt.Errorf("status frame is %d bytes, too short to contain the gripper fields (need %d) — "+
			"the controller firmware may predate them", len(fr.RawPayload), minGripperPayloadLen)
	}
	return fr, nil
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

// Geometries returns the gripper's body + finger geometry posed at the live
// gripper position, so the 3D scene shows the fingers open/closed. On a failure
// to read the position it falls back to the fully-open pose rather than
// erroring, so visualization degrades gracefully.
func (g *fairinoGripper) Geometries(ctx context.Context, extra map[string]any) ([]spatialmath.Geometry, error) {
	inputs, err := g.CurrentInputs(ctx)
	if err != nil {
		inputs = positionToInputs(100, g.strokeMM)
	}
	gif, err := g.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

// Kinematics returns the 2-DoF finger model (gripper_kinematics.go) so the
// frame system can render the gripper opening and closing.
func (g *fairinoGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.model, nil
}

// CurrentInputs reads the live gripper position (status feed gripper_position,
// 0-100%) and maps it to the model's two prismatic finger inputs, so
// Kinematics/Geometries reflect how open the gripper actually is.
func (g *fairinoGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	fr, err := g.gripperFrame(ctx)
	if err != nil {
		return nil, err
	}
	return positionToInputs(float64(fr.GripperPosition), g.strokeMM), nil
}

// GoToInputs drives the fingers to the requested opening by mapping the joint
// input back to a percentage and issuing MoveGripper.
func (g *fairinoGripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	for _, in := range inputs {
		if err := g.moveGripper(ctx, int(inputsToPercent(in, g.strokeMM))); err != nil {
			return err
		}
	}
	return nil
}

func (g *fairinoGripper) Name() resource.Name { return g.name }

// Status returns a lightweight status map, best-effort — errors are reported in
// the map rather than failing, so status polling stays cheap. Everything here
// comes from one status frame, so it costs no controller round-trip at all.
func (g *fairinoGripper) Status(ctx context.Context) (map[string]any, error) {
	fr, err := g.gripperFrame(ctx)
	if err != nil {
		return map[string]any{"error": err.Error()}, nil
	}
	return map[string]any{
		"position":     fr.GripperPosition,
		"motion_done":  fr.GripperMotionDone,
		"holding":      fr.GripperMotionDone == gripperMotionClamped,
		"dropped":      fr.GripperMotionDone == gripperMotionDropped,
		"active":       fr.GripperActive,
		"fault":        fr.GripperFault,
		"fault_id":     fr.GripperFaultID,
		"speed":        fr.GripperSpeed,
		"current":      fr.GripperCurrent,
		"temperature":  fr.GripperTemp,
		"voltage":      fr.GripperVoltage,
		"frame_age_ms": time.Since(fr.ReceivedAt).Milliseconds(),
	}, nil
}

// Close releases this resource's share of the status reader (stopping it if the
// arm isn't using it too) and drops the command connection. Unlike the previous
// XML-RPC transport — stateless HTTP with nothing to tear down — both of these
// own real resources.
func (g *fairinoGripper) Close(ctx context.Context) error {
	releaseStatusClient(g.host, statusPort)
	releaseCommandClient(g.host, commandPort)
	return nil
}

// DoCommand exposes raw controller queries/actions, useful for discovering the
// gripper's config codes and validating status semantics against real hardware.
// Supported commands (key "command"):
//   - "gripper_config": GetGripperConfig -> company, device, software_version
//   - "status": the full gripper slice of the status feed (same as Status)
//   - "activate": reset + activate (ActGripper 0 then 1) with waits
//   - "deactivate": ActGripper(index, 0)
//   - "set_config": SetGripperConfig using company/device/software_version/bus keys
//   - "move": MoveGripper to "position" (0-100), optional "speed"/"force"
//   - "diag": config + robot error codes + the raw gripper status fields
//
// "software_version" and "reset_error" are gone with the XML-RPC transport:
// GetSoftwareVersion is an SDK call with no chapter-7 equivalent, and the
// command protocol documents no general error-reset (only per-peripheral ones
// like CNCErrorReset). Controller faults surface via "diag"'s error codes.
func (g *fairinoGripper) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	command, _ := cmd["command"].(string)
	switch command {
	case "gripper_config":
		conf, err := g.cmd.getGripperConfig(ctx)
		if err != nil {
			return nil, err
		}
		return map[string]any{
			"company": conf.Company, "device": conf.Device,
			"software_version": conf.SoftwareVersion,
		}, nil
	case "status":
		return g.Status(ctx)
	case "activate":
		// Reset first, then activate, waiting for the homing to complete —
		// matches the recovery path so manual activation from the UI is enough
		// to clear an errcode-73 gripper.
		if err := g.activate(ctx, true); err != nil {
			return nil, err
		}
		return map[string]any{"ok": true}, nil
	case "deactivate":
		if err := g.cmd.actGripper(ctx, gripperIndex, 0); err != nil {
			return nil, err
		}
		return map[string]any{"ok": true}, nil
	case "diag":
		out := map[string]any{}
		if conf, err := g.cmd.getGripperConfig(ctx); err != nil {
			out["GetGripperConfig"] = "ERR: " + err.Error()
		} else {
			out["GetGripperConfig"] = fmt.Sprintf("company=%d device=%d software_version=%d",
				conf.Company, conf.Device, conf.SoftwareVersion)
		}
		if mainCode, subCode, err := g.cmd.getRobotErrorCode(ctx); err != nil {
			out["GetRobotErrorCode"] = "ERR: " + err.Error()
		} else {
			out["GetRobotErrorCode"] = fmt.Sprintf("main=%d (%s) sub=%d", mainCode, cmdErrorString(mainCode), subCode)
		}
		if st, err := g.Status(ctx); err == nil {
			out["status"] = st
		}
		return out, nil
	case "set_config":
		if err := g.cmd.setGripperConfig(ctx,
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
