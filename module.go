// Package viamfairino implements Fairino FR-series arms (FR5 and FR5-WML,
// selected via Config.Model) as a Viam arm component.
//
// A statusClient (status.go) reads the controller's 8083 status broadcast
// and backs the read-only arm.Arm methods (JointPositions, CurrentInputs,
// IsMoving, Status, DoCommand status dump). A commandClient (command.go)
// sends motion commands on the controller's 8080 command port, via the
// shared moveToJointPositionsOnce primitive: MoveToJointPositions calls it
// directly for a single target, MoveThroughJointPositions loops it for a
// multi-waypoint sequence, and GoToInputs delegates to
// MoveThroughJointPositions. MoveToPosition (Cartesian moves) delegates to a
// motion.Service (this robot's default "builtin" service unless Config.Motion
// names another one) rather than planning IK itself — mirrors
// viam-ufactory-xarm's MoveToPosition, not viam-dobot's self-contained
// armplanning.MoveArm, specifically to keep the nlopt/cgo dependency that
// motion planning needs out of this module's own binary (see
// MoveToPosition's doc comment for what this requires in the robot's JSON
// config). Kinematics returns a real link/geometry model (kinematics.go),
// parsed from the selected model's URDF shipped in the module tarball (see
// arm/fr5_SOURCES.md / arm/fr5wml_SOURCES.md for provenance); Geometries and
// Get3DModels both use that same model and its bundled meshes. EndPosition
// is implemented via forward kinematics (see its own doc comment for the
// still-open design question). Stop is still stubbed.
//
// The wire protocols use degrees and millimeters (constants.go/status.go/
// command.go); all conversion to radians/SI happens in this file, at the
// arm.Arm boundary.
package viamfairino

import (
	"context"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"time"

	commonpb "go.viam.com/api/common/v1"
	arm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
)

var Fairino = resource.NewModel("viam-soleng", "viam-fairino", "fairino")

func init() {
	resource.RegisterComponent(arm.API, Fairino,
		resource.Registration[arm.Arm, *Config]{
			Constructor: newViamFairinoFairino,
		},
	)
}

type Config struct {
	// Host is the controller IP address; defaults to 192.168.58.2 (the
	// Fairino factory default). The controller's ports are fixed (status
	// 8083, commands 8080) and deliberately not configurable.
	Host string `json:"host,omitempty"`

	// Model selects which robot's kinematics/meshes to load: "fr5" (standard
	// FR5) or "fr5-wml" (long-reach FR5-WML). Optional: if empty, the model is
	// auto-detected at construction by querying the controller's
	// GetSoftwareVersion (see NewFairino/detectModel). Set it explicitly to
	// override detection or to let the arm construct while the robot is
	// unreachable. Validated against modelRegistry when non-empty.
	Model string `json:"model,omitempty"`

	// ToolNum and WorkpieceNum select the active tool/workpiece coordinate
	// frame (manual: 0-14 each) used for MoveJ's toolNum/workPieceNum
	// fields. Default 0 (no tool, base workpiece) for both — since that's
	// also the zero value, there's no separate "unset" sentinel needed.
	ToolNum      int `json:"tool_num,omitempty"`
	WorkpieceNum int `json:"workpiece_num,omitempty"`

	// JointSpeedPercent is the MoveJ "speed" field (manual: 1-100). Zero
	// (unset) defaults to 20, matching fairino-python-sdk's own MoveJ
	// default (vel=20.0) — not otherwise documented as a controller-side
	// default, so this module picks the same value the vendor SDK does.
	JointSpeedPercent int `json:"joint_speed_percent,omitempty"`

	// UseURDF selects the selected model's on-disk URDF as the kinematics
	// source instead of the default embedded JSON (kinematics.go). The URDF
	// path requires VIAM_MODULE_ROOT to be set (true for a normally-deployed
	// module; not automatically true for e.g. bare `go test`). See PLAN.md
	// Step 10 for why JSON is the default: among other things, it's the only
	// one the Viam app's joint-limit sliders currently parse correctly.
	UseURDF bool `json:"use_urdf,omitempty"`

	// MeshDecimationRatios is only used when UseURDF is true; forwarded
	// as-is to referenceframe.ParseModelXMLFile. nil means no decimation.
	// See that function's doc comment for the per-collision-mesh ordering
	// it expects (document order: base_link, then the 6 links in URDF
	// joint order — not one entry per joint).
	MeshDecimationRatios []float64 `json:"mesh_decimation_ratios,omitempty"`

	// Motion names the motion.Service MoveToPosition delegates to (see that
	// method's doc comment). Empty (the default) means "use this robot's
	// default 'builtin' motion service if one is available" — an optional
	// dependency, so the arm still constructs successfully even if it isn't
	// (MoveToPosition then just errors when actually called). Naming a
	// service explicitly makes it a hard dependency instead: construction
	// fails if that named service isn't available.
	Motion string `json:"motion,omitempty"`

	// BlendTimeMs is MoveJ's non-blocking "blendT" (manual Table 2-1), used
	// for every waypoint but the last in a multi-waypoint move
	// (MoveThroughJointPositions, and GoToInputs/MoveToPosition which
	// delegate to it). Zero (default) disables blending: every waypoint
	// stops fully, matching this module's pre-blending behavior. Range
	// [0,500]; see README for the IK "cut corner" caveat.
	BlendTimeMs int `json:"blend_time_ms,omitempty"`
}

// Validate ensures all parts of the config are valid. Host is optional (it
// has a default) and any non-empty value is accepted as-is — a bad address
// surfaces as a dial error from the status client. Also declares Motion (see
// its doc comment) as a dependency — required if named explicitly, optional
// (defaulting to "builtin") otherwise — mirroring viam-ufactory-xarm's
// Validate exactly; this is what actually gets the motion service into the
// resource.Dependencies map handed to NewFairino, not just it existing on
// the robot.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	// Model is optional — empty means auto-detect at construction. Only
	// validate it when set.
	if cfg.Model != "" {
		if _, ok := modelRegistry[cfg.Model]; !ok {
			return nil, nil, fmt.Errorf("%s: unsupported model %q; supported models: %s", path, cfg.Model, supportedModels())
		}
	}
	for name, v := range map[string]int{
		"tool_num":      cfg.ToolNum,
		"workpiece_num": cfg.WorkpieceNum,
	} {
		if v < 0 || v > 14 {
			return nil, nil, fmt.Errorf("%s: %s must be between 0 and 14, got %d", path, name, v)
		}
	}
	if v := cfg.JointSpeedPercent; v < 0 || v > 100 {
		return nil, nil, fmt.Errorf("%s: joint_speed_percent must be between 1 and 100, got %d", path, v)
	}
	if v := cfg.BlendTimeMs; v < 0 || v > 500 {
		return nil, nil, fmt.Errorf("%s: blend_time_ms must be between 0 and 500, got %d", path, v)
	}

	var deps, optionalDeps []string
	if cfg.Motion != "" {
		deps = append(deps, motion.Named(cfg.Motion).String())
	} else {
		optionalDeps = append(optionalDeps, motion.Named("builtin").String())
	}
	return deps, optionalDeps, nil
}

// defaultJointSpeedPercent is used when Config.JointSpeedPercent is unset
// (zero value) — see that field's doc comment.
const defaultJointSpeedPercent = 20

type viamFairinoFairino struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *Config

	// host is the resolved controller address (cfg.Host or the default). Kept
	// separately because it is the key the shared status client is registered
	// under — releasing with cfg.Host would miss the default-host case.
	host string

	// model is the selected robot's kinematic model, built once at
	// construction time by kinematics.go's makeModelFrame (per conf.Model).
	model referenceframe.Model

	// meshVisualDir is the selected model's visual-mesh directory (relative to
	// VIAM_MODULE_ROOT) that Get3DModels serves from — set from the model's
	// modelRegistry entry at construction.
	meshVisualDir string

	// motion is the motion.Service MoveToPosition delegates to; nil if none
	// was available (see Config.Motion's doc comment and MoveToPosition's).
	motion motion.Service

	// meshMu guards meshCache — see Get3DModels.
	meshMu    sync.Mutex
	meshCache map[string]*commonpb.Mesh

	status *statusClient
	cmd    *commandClient
	opMgr  *operation.SingleOperationManager

	cancelCtx  context.Context
	cancelFunc func()
}

func newViamFairinoFairino(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (arm.Arm, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewFairino(ctx, deps, rawConf.ResourceName(), conf, logger)
}

// detectModelTimeout bounds the GetSoftwareVersion round-trip used for model
// auto-detection at construction, so a dead/wrong host fails fast instead of
// hanging module startup.
const detectModelTimeout = 5 * time.Second

// detectModel queries the controller's GetSoftwareVersion (CMD_ID 905) and maps
// its reported robotModel to a modelRegistry key. Returns an error if the query
// fails (robot unreachable) or the robotModel is unrecognized — in both cases
// the caller should tell the user to set model explicitly. The raw version
// fields are logged so an unrecognized robotModel is visible for adding a
// mapping.
func detectModel(ctx context.Context, cmd *commandClient, logger logging.Logger) (string, error) {
	ctx, cancel := context.WithTimeout(ctx, detectModelTimeout)
	defer cancel()

	robotModel, webVersion, controllerVersion, err := cmd.getSoftwareVersion(ctx)
	if err != nil {
		return "", fmt.Errorf("querying GetSoftwareVersion for model auto-detection: %w", err)
	}
	logger.CInfow(ctx, "queried robot software version for model auto-detection",
		"robot_model", robotModel, "web_version", webVersion, "controller_version", controllerVersion)

	model, ok := modelFromRobotModel(robotModel)
	if !ok {
		return "", fmt.Errorf("unrecognized robotModel %q (supported models: %s)", robotModel, supportedModels())
	}
	logger.CInfow(ctx, "auto-detected robot model", "model", model, "robot_model", robotModel)
	return model, nil
}

func NewFairino(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (arm.Arm, error) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	host := conf.Host
	if host == "" {
		host = defaultHost
	}

	cmd := acquireCommandClient(host, commandPort, logger)
	// Release the shared command client if construction fails before the
	// resource takes ownership (its Close is what releases it thereafter).
	cmdOwned := false
	defer func() {
		if !cmdOwned {
			releaseCommandClient(host, commandPort)
		}
	}()

	// Resolve the robot model. An explicit conf.Model wins (and lets the arm
	// construct while the robot is unreachable); otherwise auto-detect it by
	// querying the controller (detectModel), which needs a live connection.
	modelName := conf.Model
	if modelName == "" {
		detected, err := detectModel(ctx, cmd, logger)
		if err != nil {
			cancelFunc()
			return nil, fmt.Errorf("model not set in config and auto-detection failed "+
				"(set model=%s explicitly): %w", supportedModels(), err)
		}
		modelName = detected
	}

	model, err := makeModelFrame(name.Name, modelName, conf.UseURDF, conf.MeshDecimationRatios)
	if err != nil {
		cancelFunc()
		return nil, err
	}
	// modelName is either validated (Validate) or came from detectModel, so
	// this lookup can't miss.
	assets := modelRegistry[modelName]

	// Resolve the motion service MoveToPosition delegates to (see its doc
	// comment). An explicitly-named service is a hard dependency — fail
	// construction if it's missing. The implicit "builtin" default is
	// optional: log and continue with motionSvc == nil rather than fail,
	// since a fairino arm with no Cartesian-motion needs shouldn't refuse to
	// construct over it — mirrors viam-ufactory-xarm's NewXArm exactly.
	var motionSvc motion.Service
	if conf.Motion != "" {
		motionSvc, err = motion.FromProvider(deps, conf.Motion)
		if err != nil {
			cancelFunc()
			return nil, fmt.Errorf("motion service %q: %w", conf.Motion, err)
		}
	} else if motionSvc, err = motion.FromProvider(deps, "builtin"); err != nil {
		logger.CDebugw(ctx, "no default motion service available; MoveToPosition will be unavailable", "err", err)
		motionSvc = nil
	}

	s := &viamFairinoFairino{
		name:          name,
		logger:        logger,
		cfg:           conf,
		host:          host,
		model:         model,
		meshVisualDir: assets.meshVisualDir,
		motion:        motionSvc,
		status:        acquireStatusClient(ctx, host, statusPort, logger),
		cmd:           cmd,
		opMgr:         operation.NewSingleOperationManager(),
		cancelCtx:     cancelCtx,
		cancelFunc:    cancelFunc,
	}
	cmdOwned = true // s now owns cmd; its Close releases it
	// Wait for the first status frame so early API calls don't race the
	// connection; warn rather than fail so the resource can come up while the
	// robot is off and recover via the reconnect loop.
	waitCtx, cancel := context.WithTimeout(ctx, 3*time.Second)
	defer cancel()
	if _, err := s.status.waitForFrame(waitCtx); err != nil {
		logger.CWarnw(ctx, "no initial status frame received", "host", host, "err", err)
	}

	return s, nil
}

func (s *viamFairinoFairino) Name() resource.Name {
	return s.name
}

// latestFrame fetches the latest status frame, blocking briefly if none has
// arrived yet (typical right after startup).
func (s *viamFairinoFairino) latestFrame(ctx context.Context) (statusFrame, error) {
	fr, ok, _ := s.status.latest()
	if ok {
		return fr, nil
	}
	return s.status.waitForFrame(ctx)
}

// JointPositions returns the current joint positions of the arm.
func (s *viamFairinoFairino) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	fr, err := s.latestFrame(ctx)
	if err != nil {
		return nil, err
	}
	out := make([]referenceframe.Input, 6)
	for i := 0; i < 6; i++ {
		out[i] = rdkutils.DegToRad(fr.JointDegs[i])
	}
	return out, nil
}

func (s *viamFairinoFairino) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return s.JointPositions(ctx, nil)
}

// IsMoving reports whether the controller considers the robot "in place".
//
// TODO(phase 2+): robot_motion_done may only reflect commanded-motion
// completion, not pendant/drag-mode motion. If hardware testing shows drag
// moves reported as done, fold in program_state == running and/or joint-delta
// detection. See PLAN.md, Risks #3.
func (s *viamFairinoFairino) IsMoving(ctx context.Context) (bool, error) {
	fr, ok, err := s.status.latest()
	if !ok {
		if err == nil {
			err = errors.New("no status frame received yet")
		}
		return false, err
	}
	return !fr.MotionDone, nil
}

// Status is the resource health surface: connection state and a decoded
// summary while the status stream is live, or the connection error when it
// is down (which surfaces the resource as unhealthy in the machine status
// API rather than only failing lazily on use).
func (s *viamFairinoFairino) Status(ctx context.Context) (map[string]interface{}, error) {
	fr, ok, err := s.status.latest()
	if !ok {
		if err == nil {
			err = errors.New("no status frame received yet")
		}
		return nil, fmt.Errorf("status stream down: %w", err)
	}
	return map[string]interface{}{
		"connected":         true,
		"last_frame_age_ms": float64(time.Since(fr.ReceivedAt).Milliseconds()),
		"program_state":     programStateString(fr.ProgramState),
		"robot_mode":        robotModeString(fr.RobotMode),
		"fault":             faultCodeString(fr.ErrorCode),
		"emergency_stop":    fr.EmergencyStop,
		"motion_done":       fr.MotionDone,
	}, nil
}

// DoCommand exposes the full decoded status for bring-up and debugging, plus
// drag-teach (free-drive) mode control:
//
//	{"command": "status"} → every Phase 1 field plus payload diagnostics.
//	{"command": "enter_drag_mode"} → DragTeachSwitch(1)
//	{"command": "exit_drag_mode"} → DragTeachSwitch(0).
func (s *viamFairinoFairino) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	command, _ := cmd["command"].(string)
	switch command {
	case "status":
		fr, err := s.latestFrame(ctx)
		if err != nil {
			return nil, err
		}
		return statusDump(fr), nil
	case "enter_drag_mode":
		if err := s.cmd.dragTeachSwitch(ctx, true); err != nil {
			return nil, err
		}
		return map[string]interface{}{"ok": true}, nil
	case "exit_drag_mode":
		if err := s.cmd.dragTeachSwitch(ctx, false); err != nil {
			return nil, err
		}
		return map[string]interface{}{"ok": true}, nil
	default:
		return nil, fmt.Errorf("unknown command %q (supported: \"status\", \"enter_drag_mode\", \"exit_drag_mode\")", command)
	}
}

// statusDump flattens a statusFrame into a JSON-friendly map (float64, bool,
// string, and []interface{} values only, per structpb).
func statusDump(fr statusFrame) map[string]interface{} {
	p := fr.RawPayload
	return map[string]interface{}{
		"program_state":     programStateString(fr.ProgramState),
		"error_code":        float64(fr.ErrorCode),
		"fault":             faultCodeString(fr.ErrorCode),
		"robot_mode":        robotModeString(fr.RobotMode),
		"jt_cur_pos_deg":    floatSlice(fr.JointDegs[:]),
		"tl_cur_pos":        floatSlice(fr.TCPPose[:]),
		"tool_num":          float64(fr.ToolNum),
		"jt_cur_tor_nm":     floatSlice(fr.JointTorques[:]),
		"program_name":      fr.ProgramName,
		"prog_total_line":   float64(p[offProgTotalLine]),
		"prog_cur_line":     float64(p[offProgCurLine]),
		"cl_dgt_output_h":   float64(p[offClDgtOutputH]),
		"cl_dgt_output_l":   float64(p[offClDgtOutputL]),
		"tl_dgt_output_l":   float64(p[offTlDgtOutputL]),
		"cl_dgt_input_h":    float64(p[offClDgtInputH]),
		"cl_dgt_input_l":    float64(p[offClDgtInputL]),
		"tl_dgt_input_l":    float64(p[offTlDgtInputL]),
		"ft_data":           ftData(p),
		"ft_act_status":     float64(p[offFTActStatus]),
		"emergency_stop":    fr.EmergencyStop,
		"robot_motion_done": fr.MotionDone,
		"frame_count":       float64(fr.FrameCount),
		"payload_len":       float64(len(p)),
		"received_at":       fr.ReceivedAt.Format(time.RFC3339Nano),
	}
}

func floatSlice(vals []float64) []interface{} {
	out := make([]interface{}, len(vals))
	for i, v := range vals {
		out[i] = v
	}
	return out
}

func ftData(payload []byte) []interface{} {
	out := make([]interface{}, 6)
	for i := 0; i < 6; i++ {
		out[i] = readF64(payload, offFTData+i*8)
	}
	return out
}

// ---------- arm.Arm: motion ----------

// Motion-completion polling constants, mirroring viam-dobot's
// waitForMotionCompleteLocked (arm/cr10a.go there) adapted to Fairino's
// statusFrame. motionPollInterval matches the poll cadence status.go's own
// waitForFrame already uses elsewhere in this package.
const (
	motionPollInterval = 20 * time.Millisecond

	// motionStartGrace is how long we wait after a MoveJ ack for MotionDone
	// to flip false (motion actually started) before falling through to the
	// completion poll anyway — covers zero-distance moves and commands that
	// finish faster than our poll cadence.
	motionStartGrace = 200 * time.Millisecond

	// jointToleranceDeg is the per-joint slop accepted when confirming a
	// completed move actually reached its target, degrees.
	jointToleranceDeg = 0.5
)

// MoveToJointPositions moves the arm's joints to the given positions via a
// single blocking MoveJ() (manual §2.1). See moveToJointPositionsOnce for
// the actual mechanics — this just wraps it with this module's own
// single-operation bookkeeping, since it's a standalone public entry point.
func (s *viamFairinoFairino) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	ctx, done := s.opMgr.New(ctx)
	defer done()
	return s.moveToJointPositionsOnce(ctx, positions, blendTBlocking)
}

// blendTBlocking is moveJParams.BlendT's "-1": decelerate to a full stop at
// this waypoint before any next queued motion.
const blendTBlocking = -1

// moveToJointPositionsOnce sends a single blocking MoveJ() (manual §2.1) and
// waits for completion. It returns once the controller reports the move
// complete (statusFrame.MotionDone) and the joints are within tolerance of
// the target, or once ctx is cancelled — in which case it sends STOP before
// returning ctx.Err(), mirroring viam-dobot's cancellation handling.
//
// Shared by MoveToJointPositions (always blendTBlocking) and
// MoveThroughJointPositions (chooses blendT per waypoint). Deliberately does
// NOT call s.opMgr.New itself: operation.SingleOperationManager cancels the
// previous operation the moment a new one starts, so if this were called
// per-waypoint from inside a loop that's already under one opMgr operation,
// each waypoint would immediately cancel the one before it. The public entry
// points (MoveToJointPositions, MoveToPosition) own the single opMgr.New call
// for the whole move; this is the shared mechanics underneath.
//
// blendTBlocking waits for physical completion (statusFrame.MotionDone, joints
// within tolerance), sending STOP and returning ctx.Err() on cancellation. Any
// positive blendT (a blend — see Config.BlendTimeMs) returns as soon as the
// command is acked; the caller must send the next waypoint immediately so the
// controller's own motion queue blends between them. blendT == 0 is rejected: a
// 0ms blend is indistinguishable from a full stop, so it's never a meaningful
// value to send.
func (s *viamFairinoFairino) moveToJointPositionsOnce(ctx context.Context, positions []referenceframe.Input, blendT float64) error {
	if blendT == 0 {
		return errors.New("moveToJointPositionsOnce: blendT must not be 0 (0ms blend is no different from a full stop)")
	}
	moveStart := time.Now()
	s.logger.Debugw("moveToJointPositionsOnce: called", "positions_rad", positions)

	var targetDeg [6]float64
	for i := 0; i < 6; i++ {
		targetDeg[i] = rdkutils.RadToDeg(positions[i])
	}
	s.logger.Debugw("moveToJointPositionsOnce: target", "target_deg", targetDeg)

	// desc_pos: see moveJParams.Pose's doc comment in command.go — the
	// controller validates this against the joint target, so it must be the
	// real forward-kinematics pose for targetDeg, not a placeholder. Fetched
	// from the controller itself (GetForwardKin) rather than computed from
	// s.model — this is proven working on hardware, and MoveToPosition
	// (which now has a real kinematic model available) still routes through
	// this same path via MoveThroughJointPositions, so there was no reason
	// to change it.
	targetPose, err := s.cmd.getForwardKin(ctx, targetDeg)
	if err != nil {
		s.logger.Debugw("moveToJointPositionsOnce: GetForwardKin failed", "err", err)
		return fmt.Errorf("GetForwardKin: %w", err)
	}
	s.logger.Debugw("moveToJointPositionsOnce: forward kinematics for target", "target_deg", targetDeg, "target_pose", targetPose)

	params := moveJParams{
		Joints:       targetDeg,
		Pose:         targetPose,
		ToolNum:      s.cfg.ToolNum,
		WorkPieceNum: s.cfg.WorkpieceNum,
		SpeedPercent: float64(orDefaultInt(s.cfg.JointSpeedPercent, defaultJointSpeedPercent)),
		AccelPercent: 0, // reserved/non-functional on the controller — see moveJParams.AccelPercent
		OverridePct:  100,
		BlendT:       blendT,
	}

	if err := s.cmd.moveJ(ctx, params); err != nil {
		s.logger.Debugw("moveToJointPositionsOnce: MoveJ ack failed", "elapsed", time.Since(moveStart), "err", err)
		return fmt.Errorf("MoveJ: %w", err)
	}

	if blendT > 0 {
		// Blend: queued, blending into whatever's sent next — nothing to wait
		// for. blendT > 0 rather than != blendTBlocking as a precaution: any
		// non-positive value falls through to waiting.
		s.logger.Debugw("moveToJointPositionsOnce: MoveJ acknowledged (blend), not waiting for completion", "elapsed", time.Since(moveStart))
		return nil
	}
	s.logger.Debugw("moveToJointPositionsOnce: MoveJ acknowledged, waiting for motion to complete", "elapsed", time.Since(moveStart))

	err = s.waitForMotionComplete(ctx, targetDeg)
	s.logger.Debugw("moveToJointPositionsOnce: done", "elapsed", time.Since(moveStart), "err", err)
	return err
}

// waitForMotionComplete polls the status stream until the controller
// reports the move done (statusFrame.MotionDone) and the joints are within
// jointToleranceDeg of targetDeg, or returns an error if the controller
// e-stops or latches a fault mid-move. On ctx cancellation it issues STOP
// on a fresh short-lived context before returning ctx.Err(), same shape as
// viam-dobot's waitForMotionCompleteLocked.
func (s *viamFairinoFairino) waitForMotionComplete(ctx context.Context, targetDeg [6]float64) error {
	// Per-poll-tick Debug logging of the MotionDone/JointDegs transitions.
	pollStart := time.Now()
	s.logger.Debugw("waitForMotionComplete: entering start-grace phase", "grace", motionStartGrace)

	startCtx, cancelGrace := context.WithTimeout(ctx, motionStartGrace)
	defer cancelGrace()
	for {
		fr, ok, _ := s.status.latest()
		s.logger.Debugw("waitForMotionComplete: start-grace poll", "have_frame", ok,
			"motion_done", fr.MotionDone, "joint_deg", fr.JointDegs, "e_stop", fr.EmergencyStop)
		if ok && !fr.MotionDone {
			s.logger.Debugw("waitForMotionComplete: motion started", "elapsed", time.Since(pollStart))
			break // motion has started
		}
		if ok && fr.EmergencyStop {
			return errors.New("Fairino controller is e-stopped — motion did not start")
		}
		select {
		case <-startCtx.Done():
			// No transition seen within the grace window — fall through to
			// the completion poll anyway; covers zero-distance moves.
			s.logger.Debugw("waitForMotionComplete: start-grace window elapsed with no MotionDone transition, falling through to completion poll", "elapsed", time.Since(pollStart))
			goto completed
		case <-time.After(motionPollInterval):
		}
	}
completed:

	for {
		select {
		case <-ctx.Done():
			s.logger.Debugw("waitForMotionComplete: ctx cancelled, sending STOP", "elapsed", time.Since(pollStart), "ctx_err", ctx.Err())
			stopCtx, cancelStop := context.WithTimeout(context.Background(), 2*time.Second)
			_ = s.cmd.stop(stopCtx)
			cancelStop()
			return ctx.Err()
		case <-time.After(motionPollInterval):
		}

		fr, ok, _ := s.status.latest()
		if !ok {
			s.logger.Debugw("waitForMotionComplete: completion poll — no status frame yet", "elapsed", time.Since(pollStart))
			continue
		}
		s.logger.Debugw("waitForMotionComplete: completion poll", "elapsed", time.Since(pollStart),
			"motion_done", fr.MotionDone, "joint_deg", fr.JointDegs, "target_deg", targetDeg,
			"error_code", fr.ErrorCode, "e_stop", fr.EmergencyStop)
		if fr.EmergencyStop {
			return errors.New("Fairino controller e-stopped during motion")
		}
		if fr.ErrorCode != 0 && !isWarningCode(fr.ErrorCode) {
			return fmt.Errorf("Fairino controller reported a fault during motion: %s", faultCodeString(fr.ErrorCode))
		}
		if fr.MotionDone {
			if jointsCloseDeg(fr.JointDegs, targetDeg, jointToleranceDeg) {
				if fr.ErrorCode != 0 {
					// A warning (e.g. joint configuration warning) doesn't stop
					// the controller; the joints reached the target, so the move
					// succeeded. Surface it without failing the call.
					s.logger.Warnw("Fairino controller raised a warning during motion; move completed",
						"warning", faultCodeString(fr.ErrorCode), "joint_deg", fr.JointDegs)
				}
				s.logger.Debugw("waitForMotionComplete: motion complete, joints within tolerance", "elapsed", time.Since(pollStart), "joint_deg", fr.JointDegs)
				return nil
			}
			s.logger.Debugw("waitForMotionComplete: motion done but joints out of tolerance", "elapsed", time.Since(pollStart), "joint_deg", fr.JointDegs, "target_deg", targetDeg)
			return fmt.Errorf("motion completed but joints did not reach target (got %v want %v degrees)", fr.JointDegs, targetDeg)
		}
	}
}

func jointsCloseDeg(a, b [6]float64, tolDeg float64) bool {
	for i := 0; i < 6; i++ {
		d := a[i] - b[i]
		if d < 0 {
			d = -d
		}
		if d > tolDeg {
			return false
		}
	}
	return true
}

// orDefaultInt returns def when v is the zero value, else v — for Config
// fields whose valid range excludes 0 (see Config.JointSpeedPercent).
func orDefaultInt(v, def int) int {
	if v == 0 {
		return def
	}
	return v
}

func (s *viamFairinoFairino) Close(context.Context) error {
	releaseStatusClient(s.host, statusPort)
	releaseCommandClient(s.host, commandPort)
	s.cancelFunc()
	return nil
}

// ---------- stubbed methods ----------

// errPhase2 marks Stop, the one remaining stubbed method. Everything else
// that needs the command port (MoveToJointPositions, MoveThroughJointPositions,
// MoveToPosition, GoToInputs) is implemented.
var errPhase2 = errors.New("not implemented yet; motion via the command port is planned but not yet built for this method")

// EndPosition returns the current position of the arm, computed via the
// kinematic model's forward kinematics (kinematics.go) from the current
// joint positions — matches dobot's own EndPosition exactly (arm/cr10a.go:
// model.Transform(inputs)).
//
// TEMPORARY: this unblocks the app's 3D scene viewer, which turned out to
// require EndPosition to succeed before it renders anything at all — it's
// not the final design. The real decision (serve the device-reported TCP
// pose, tl_cur_pos, vs. compute via FK — and tl_cur_pos's Euler convention,
// which was never confirmed against spatialmath's) is still open. See
// PLAN.md Step 9.
func (s *viamFairinoFairino) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	return s.model.Transform(inputs)
}

// MoveToPosition moves the arm to the given absolute position by delegating
// to a motion.Service (s.motion — this robot's default "builtin" service
// unless Config.Motion names another one; see that field's doc comment).
// Mirrors viam-ufactory-xarm's MoveToPosition exactly (arm/comm.go), NOT
// viam-dobot's self-contained armplanning.MoveArm: the service does the
// same IK/self-collision-aware planning (using this arm's own
// Kinematics()/Geometries() via the robot's frame system) that
// armplanning.MoveArm did, but the nlopt/cgo dependency that requires now
// lives in viam-server's own builtin motion service binary instead of this
// module's — Viam ships viam-server pre-built with that already resolved,
// avoiding the runtime "libnlopt.so.0: cannot open shared object file"
// deployment failure the previous approach hit.
//
// Requires this arm's robot-config JSON entry to include a "frame" block
// (at minimum a "parent", e.g. "world") — without it, the arm is never
// added to the frame system, "<name>_origin" never exists, and this fails
// with "component named <name> not found in robot frame system". That's a
// robot-config change, not something this module can supply on its own.
//
// Execution goes through GoToInputs below, not MoveThroughJointPositions
// directly — that's what the builtin motion service's plan executor
// actually calls (framesystem.InputEnabled), so GoToInputs can't be left
// stubbed once this depends on a motion service.
func (s *viamFairinoFairino) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	if s.motion == nil {
		return errors.New("fairino cannot do MoveToPosition without a motion service available (see MoveToPosition's doc comment)")
	}
	ctx, done := s.opMgr.New(ctx)
	defer done()
	_, err := s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.Name().Name,
		Destination:   referenceframe.NewPoseInFrame(fmt.Sprintf("%s_origin", s.Name().Name), pose),
	})
	return err
}

// MoveThroughJointPositions moves the arm's joints through the given
// positions in order, via moveToJointPositionsOnce per waypoint. Doesn't
// call s.opMgr.New itself — see that method's doc comment for why — so a
// direct caller other than MoveToPosition or GoToInputs (which wrap one
// operation around the whole sequence) runs without single-operation
// protection.
//
// options (MaxVelRads/MaxAccRads) isn't consumed: this module's command
// protocol only has a single fixed per-config speed (Config.JointSpeedPercent).
//
// Blending: with Config.BlendTimeMs nonzero and more than one waypoint, every
// waypoint but the last blends (non-blocking, no completion wait); the last
// always blocks, so this still only returns once the arm is at rest at the
// final target. Applies equally to GoToInputs/MoveToPosition's IK-planned
// trajectories, since they delegate here too — see the README for the
// resulting "cut corner" caveat.
func (s *viamFairinoFairino) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
	for i, goal := range positions {
		// The last waypoint always blocks; intermediate waypoints blend when
		// Config.BlendTimeMs is set, otherwise they block too.
		blendT := float64(blendTBlocking)
		if s.cfg.BlendTimeMs > 0 && i < len(positions)-1 {
			blendT = float64(s.cfg.BlendTimeMs)
		}
		if err := s.moveToJointPositionsOnce(ctx, goal, blendT); err != nil {
			return fmt.Errorf("waypoint %d/%d: %w", i+1, len(positions), err)
		}
	}
	return nil
}

// GoToInputs moves the arm's joints through the given series of joint
// positions. Delegates to MoveThroughJointPositions — this is the method
// RDK's builtin motion service's plan executor actually calls
// (framesystem.InputEnabled) to drive a MoveToPosition plan, so this
// couldn't stay stubbed once MoveToPosition depends on a motion service.
// Mirrors viam-ufactory-xarm's GoToInputs exactly (arm/xarm.go).
func (s *viamFairinoFairino) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return s.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (s *viamFairinoFairino) Stop(ctx context.Context, extra map[string]interface{}) error {
	return errPhase2
}

// Kinematics returns the selected model's kinematic model, built once at
// construction time (kinematics.go's makeModelFrame) from the model's embedded
// JSON (or on-disk URDF if Config.UseURDF is set).
func (s *viamFairinoFairino) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return s.model, nil
}

// Geometries returns the arm's current collision geometry: the kinematic
// model's link geometries (parsed from the URDF's <collision> meshes,
// kinematics.go) posed at the arm's current joint positions.
//
// Meshes go through rebuildMeshForPLYBug (see its doc comment) to work
// around an RDK bug where a mesh loaded from STL serializes as garbage
// binary bytes mislabeled ContentType "ply" — this broke the app's 3D scene
// viewer's getGeometries call outright (goply panic: "Invalid ply file"),
// confirmed via TestGeometryProtoRoundTrip. Non-mesh geometries (box,
// sphere, capsule) aren't affected and pass through unchanged.
func (s *viamFairinoFairino) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	inputs, err := s.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gif, err := s.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	geoms := gif.Geometries()
	for i, g := range geoms {
		geoms[i] = rebuildMeshForPLYBug(g)
	}
	return geoms, nil
}

// rebuildMeshForPLYBug works around an RDK bug in spatialmath.Mesh: a mesh
// loaded from a non-PLY source (our case: STL, via
// referenceframe.ParseModelXMLFile's <collision> mesh loading) has its
// rawBytes field populated with the *original* file's bytes (raw binary
// STL, in our case). Mesh.ensurePLYBytes only regenerates real PLY when
// rawBytes is nil, so it returns those raw STL bytes unchanged — and
// Mesh.ToProtobuf unconditionally labels whatever ensurePLYBytes returns as
// ContentType "ply" regardless. The result: GetGeometries sends raw binary
// STL data mislabeled as PLY, which goply (RDK's PLY reader) can't parse,
// panicking with "Invalid ply file" the moment it reads the STL header
// instead of the literal string "ply" — reproduced directly in
// TestGeometryProtoRoundTrip. This affects any module serving Geometries()
// for STL-sourced meshes, not just ours; worth reporting upstream.
//
// The fix: spatialmath.NewMesh(pose, triangles, label) — unlike the
// STL/PLY-file constructors — never sets rawBytes, so a mesh built this way
// correctly regenerates PLY on demand. Rebuilding from the same pose/
// triangles/label is otherwise a no-op geometrically.
func rebuildMeshForPLYBug(g spatialmath.Geometry) spatialmath.Geometry {
	mesh, ok := g.(*spatialmath.Mesh)
	if !ok {
		return g
	}
	return spatialmath.NewMesh(mesh.Pose(), mesh.Triangles(), mesh.Label())
}

// Get3DModels returns the bundled per-link visualization meshes as binary
// glTF (GLB, model/gltf-binary — the app's 3D scene viewer renders glTF,
// not PLY), keyed by link/frame name (the .glb filename minus extension,
// e.g. "link6.glb" -> "link6", which matches the URDF's own link names
// exactly since that's how they were generated — see arm/fr5wml_SOURCES.md /
// arm/fr5_SOURCES.md). Meshes are read from the selected model's
// meshVisualDir (set at construction from modelRegistry).
//
// If VIAM_MODULE_ROOT is unset, logs a warning and returns an empty map
// with a nil error rather than failing — 3D visualization is simply
// unavailable in that environment (unit tests, developer workstations
// without a real module install). A per-file read failure returns a
// wrapped error.
//
// Results are cached on the struct after the first successful build; the
// returned map is a shallow copy so callers can't mutate the cache.
func (s *viamFairinoFairino) Get3DModels(ctx context.Context, extra map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	root := os.Getenv("VIAM_MODULE_ROOT")
	if root == "" {
		s.logger.Warn("Get3DModels: VIAM_MODULE_ROOT is not set; returning empty mesh map")
		return map[string]*commonpb.Mesh{}, nil
	}

	s.meshMu.Lock()
	defer s.meshMu.Unlock()

	if s.meshCache != nil {
		return cloneMeshMap(s.meshCache), nil
	}

	built, err := buildMeshMap(root, s.meshVisualDir)
	if err != nil {
		return nil, err
	}
	s.meshCache = built
	return cloneMeshMap(built), nil
}

// buildMeshMap reads every .glb under moduleRoot/meshVisualDir and keys the
// raw bytes by filename minus extension.
func buildMeshMap(moduleRoot, meshVisualDir string) (map[string]*commonpb.Mesh, error) {
	dir := filepath.Join(moduleRoot, meshVisualDir)
	entries, err := os.ReadDir(dir)
	if err != nil {
		return nil, fmt.Errorf("reading mesh dir %q: %w", dir, err)
	}

	out := make(map[string]*commonpb.Mesh, len(entries))
	for _, entry := range entries {
		if entry.IsDir() || filepath.Ext(entry.Name()) != ".glb" {
			continue
		}
		path := filepath.Join(dir, entry.Name())
		//nolint:gosec // path is built from the module root and a directory listing, no user input.
		glb, err := os.ReadFile(path)
		if err != nil {
			return nil, fmt.Errorf("loading mesh %q: %w", path, err)
		}
		name := strings.TrimSuffix(entry.Name(), ".glb")
		out[name] = &commonpb.Mesh{
			ContentType: "model/gltf-binary",
			Mesh:        glb,
		}
	}
	return out, nil
}

// cloneMeshMap returns a shallow copy of m (a fresh map sharing the same
// *commonpb.Mesh values), so a caller mutating the returned map can't
// corrupt the cached one.
func cloneMeshMap(m map[string]*commonpb.Mesh) map[string]*commonpb.Mesh {
	out := make(map[string]*commonpb.Mesh, len(m))
	for k, v := range m {
		out[k] = v
	}
	return out
}
