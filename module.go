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
// still-open design question).
//
// The wire protocols use degrees and millimeters (constants.go/status.go/
// command.go); all conversion to radians/SI happens in this file, at the
// arm.Arm boundary.
package viamfairino

import (
	"context"
	"errors"
	"fmt"
	"math"
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

	// SpeedDegsPerSec is the default joint speed in degrees per second, an
	// alternative to JointSpeedPercent (set at most one). Converted to MoveJ's
	// percentage against maxJointSpeedDegsPerSec. Range [1.8, 180] (1.8 is 1%).
	SpeedDegsPerSec float64 `json:"speed_degs_per_sec,omitempty"`

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

	// BadJoints lists joints (by index, 0-5) that can't move. Each is locked
	// to about the position it reports at startup by narrowing its limits in
	// the kinematics, so the motion service plans around it. Same as
	// viam-ufactory-xarm's "bad-joints".
	BadJoints []int `json:"bad_joints,omitempty"`

	// CollisionLevel sets every joint's collision detection level at startup
	// (SetAnticollision), 1 (most sensitive) to 10 — the controller's own
	// scale, which runs the opposite way to viam-ufactory-xarm's 0-5.
	// CollisionLevels sets them per joint instead (6 values); set at most one.
	// Unset leaves the controller's configured levels alone.
	CollisionLevel  int   `json:"collision_level,omitempty"`
	CollisionLevels []int `json:"collision_levels,omitempty"`

	// WebAppProxy starts a local reverse proxy to the controller's WebApp on
	// WebAppProxyPort (default 18080), for when the browser can't reach the
	// controller directly. No authentication: anyone who can reach the port
	// can drive the arm. WebAppURL overrides the target (default
	// https://<host>).
	WebAppProxy     bool   `json:"webapp_proxy,omitempty"`
	WebAppProxyPort int    `json:"webapp_proxy_port,omitempty"`
	WebAppURL       string `json:"webapp_url,omitempty"`
}

// collisionLevels resolves CollisionLevel/CollisionLevels to per-joint levels;
// ok is false when neither is set.
func (cfg *Config) collisionLevels() (levels [6]int, ok bool) {
	switch {
	case cfg.CollisionLevel != 0:
		for i := range levels {
			levels[i] = cfg.CollisionLevel
		}
		return levels, true
	case len(cfg.CollisionLevels) == 6:
		copy(levels[:], cfg.CollisionLevels)
		return levels, true
	}
	return levels, false
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
	if v := cfg.SpeedDegsPerSec; v < 0 || (v > 0 && v < minSpeedDegsPerSec-speedEpsilon) || v > maxJointSpeedDegsPerSec {
		return nil, nil, fmt.Errorf("%s: speed_degs_per_sec must be between %v and %v, got %v",
			path, minSpeedDegsPerSec, maxJointSpeedDegsPerSec, v)
	}
	if cfg.SpeedDegsPerSec > 0 && cfg.JointSpeedPercent > 0 {
		return nil, nil, fmt.Errorf("%s: set at most one of speed_degs_per_sec and joint_speed_percent", path)
	}
	if v := cfg.BlendTimeMs; v < 0 || v > 500 {
		return nil, nil, fmt.Errorf("%s: blend_time_ms must be between 0 and 500, got %d", path, v)
	}

	for _, j := range cfg.BadJoints {
		if j < 0 || j > 5 {
			return nil, nil, fmt.Errorf("%s: bad_joints entries must be joint indices 0-5, got %d", path, j)
		}
	}

	if cfg.CollisionLevel != 0 && len(cfg.CollisionLevels) > 0 {
		return nil, nil, fmt.Errorf("%s: set at most one of collision_level and collision_levels", path)
	}
	if n := len(cfg.CollisionLevels); n != 0 && n != 6 {
		return nil, nil, fmt.Errorf("%s: collision_levels needs 6 values, one per joint, got %d", path, n)
	}
	if levels, ok := cfg.collisionLevels(); ok {
		for _, v := range levels {
			if v < 1 || v > 10 {
				return nil, nil, fmt.Errorf("%s: collision levels must be between 1 and 10, got %d", path, v)
			}
		}
	}

	if v := cfg.WebAppProxyPort; v < 0 || v > 65535 {
		return nil, nil, fmt.Errorf("%s: webapp_proxy_port must be a TCP port, got %d", path, v)
	}
	if _, err := webAppProxyTarget(cfg.WebAppURL, "controller"); err != nil {
		return nil, nil, fmt.Errorf("%s: %w", path, err)
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

	// speedMu guards speedPct, the current default speed as a MoveJ
	// percentage; zero means "use the configured speed" (see speed.go).
	speedMu  sync.Mutex
	speedPct float64

	webApp *webAppProxy // nil unless Config.WebAppProxy

	// plans holds the cancel funcs of in-flight MoveToPosition calls, which
	// hold no operation of their own (see MoveToPosition), so that Stop and
	// Close can still cancel them while the motion service is planning.
	plansMu sync.Mutex
	plans   map[*context.CancelFunc]struct{}
	closed  bool // set by Close; later MoveToPosition calls are refused

	closeOnce sync.Once
}

func newViamFairinoFairino(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (arm.Arm, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewFairino(ctx, deps, rawConf.ResourceName(), conf, logger)
}

// badJointsFrameWait is how long construction waits for the arm's position
// when bad_joints needs it and the status feed is still connecting.
const badJointsFrameWait = 15 * time.Second

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
			return nil, fmt.Errorf("model not set in config and auto-detection failed "+
				"(set model=%s explicitly): %w", supportedModels(), err)
		}
		modelName = detected
	}

	model, err := makeModelFrame(name.Name, modelName, conf.UseURDF, conf.MeshDecimationRatios)
	if err != nil {
		return nil, err
	}

	status := acquireStatusClient(ctx, host, statusPort, logger)
	statusOwned := false
	defer func() {
		if !statusOwned {
			releaseStatusClient(host, statusPort)
		}
	}()
	// Wait for the first status frame so early API calls don't race the
	// connection; warn rather than fail so the resource can come up while the
	// robot is off and recover via the reconnect loop.
	waitCtx, cancelWait := context.WithTimeout(ctx, 3*time.Second)
	defer cancelWait()
	firstFrame, frameErr := status.waitForFrame(waitCtx)
	if frameErr != nil {
		logger.CWarnw(ctx, "no initial status frame received", "host", host, "err", frameErr)
	}

	// Publish the configured default speed, not set_speed's runtime value:
	// kinematics describe how the arm is configured, same as viam-ufactory-xarm.
	overrides := jointLimitOverrides{maxVelocityDegsPerSec: speedDegsFromPercent(conf.speedPercent())}
	if len(conf.BadJoints) > 0 {
		// A lock is only meaningful where the joint actually is, so this
		// can't come up without a live position. Give a reader that is
		// still (re)connecting time to deliver one.
		if frameErr != nil {
			waitCtx, cancelWait := context.WithTimeout(ctx, badJointsFrameWait)
			firstFrame, frameErr = status.waitForFrameThroughErrors(waitCtx)
			cancelWait()
		}
		if frameErr != nil {
			return nil, fmt.Errorf("bad_joints needs the arm's current joint positions at startup: %w", frameErr)
		}
		overrides.lockedDeg = map[int]float64{}
		for _, j := range conf.BadJoints {
			overrides.lockedDeg[j] = firstFrame.JointDegs[j]
			logger.CInfow(ctx, "locking joint", "joint", j, "position_deg", firstFrame.JointDegs[j])
		}
	}
	if conf.UseURDF {
		logger.CWarnw(ctx, "not publishing joint speed limits or bad_joints locks: use_urdf is set, and limits can "+
			"only be written into JSON kinematics. Locks still apply to this module's own joint-limit checks.")
	}
	model, err = applyJointLimitOverrides(model, name.Name, overrides)
	if err != nil {
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
		status:        status,
		cmd:           cmd,
		opMgr:         operation.NewSingleOperationManager(),
	}
	// Applied on every connection rather than saved on the controller, so a
	// failure here is an error: the arm must not run at a sensitivity other
	// than the one configured.
	if levels, ok := conf.collisionLevels(); ok {
		if err := cmd.setAnticollision(ctx, name.String(), levels); err != nil {
			return nil, fmt.Errorf("setting collision levels %v: %w", levels, err)
		}
	}

	if conf.WebAppProxy {
		target, err := webAppProxyTarget(conf.WebAppURL, host)
		if err != nil {
			return nil, err
		}
		port := conf.WebAppProxyPort
		if port == 0 {
			port = defaultWebAppProxyPort
		}
		if s.webApp, err = startWebAppProxy(ctx, target, port, logger); err != nil {
			return nil, err
		}
	}

	// s now owns cmd and status; its Close releases them.
	cmdOwned, statusOwned = true, true
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
//	{"command": "get_state"} → program state, robot mode, motion done, e-stop, fault
//	{"command": "get_error"} → fault code plus GetRobotErrorCode's main/sub codes
//	{"command": "load"} → joint torques, N·m
//	{"command": "set_do", "index": 0-15, "on": bool} → control-box digital output
//	{"command": "set_tool_do", "index": 0-1, "on": bool} → tool digital output
//	{"command": "set_speed", "speed_degs_per_sec": N} → default joint speed for later moves
//	{"command": "get_speed"} → the current default joint speed
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
	case "set_speed":
		v, ok := floatArg(cmd, "speed_degs_per_sec")
		if !ok {
			return nil, errors.New(`set_speed needs a numeric "speed_degs_per_sec"`)
		}
		if err := s.setSpeedDegsPerSec(v); err != nil {
			return nil, err
		}
		return map[string]interface{}{"speed_degs_per_sec": v}, nil
	case "get_speed":
		return map[string]interface{}{"speed_degs_per_sec": speedDegsFromPercent(s.currentSpeedPercent())}, nil
	case "get_error":
		return s.getError(ctx)
	case "get_state":
		return s.getState(ctx)
	case "load":
		return s.getLoad(ctx)
	case "set_do":
		return s.setDigitalOutput(ctx, cmd, false)
	case "set_tool_do":
		return s.setDigitalOutput(ctx, cmd, true)
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
		return nil, fmt.Errorf("%w %q (supported: status, get_state, get_error, load, set_speed, get_speed, "+
			"set_do, set_tool_do, enter_drag_mode, exit_drag_mode)", errUnknownCommand, command)
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

	// statusFeedLostTimeout is how long a move keeps going without a status
	// frame before it is abandoned (and the arm stopped): with no feed there
	// is no way to see a fault, an e-stop or the move finishing.
	statusFeedLostTimeout = time.Second

	// motionTimeoutMargin is the fixed part of motionTimeout, covering
	// acceleration, settling and the controller's own queueing.
	motionTimeoutMargin = 10 * time.Second

	// maxJointSpeedDegsPerSec is the joint speed that MoveJ's speed=100%
	// corresponds to. The FR5 datasheet gives 180°/s on every joint. The
	// FR5-WML's isn't published, so it is assumed to be the same; if its real
	// maximum is lower, a requested speed comes out slower than asked, never
	// faster.
	maxJointSpeedDegsPerSec = 180.0
)

// motionTimeout bounds how long a move that covers travelDeg of joint travel
// (the largest single joint's) at speedPct may take before it is treated as
// stuck: twice the time at the commanded speed, plus motionTimeoutMargin.
func motionTimeout(travelDeg, speedPct float64) time.Duration {
	secs := travelDeg / (speedPct / 100 * maxJointSpeedDegsPerSec)
	return time.Duration(2*secs*float64(time.Second)) + motionTimeoutMargin
}

// maxJointTravelDeg is the largest per-joint distance between a and b.
func maxJointTravelDeg(a, b [6]float64) float64 {
	travel := 0.0
	for i := range a {
		travel = math.Max(travel, math.Abs(a[i]-b[i]))
	}
	return travel
}

// inputsToDeg converts a 6-joint target from radians to the controller's
// degrees. Callers have checked the joint count (checkJointTargets).
func inputsToDeg(positions []referenceframe.Input) [6]float64 {
	var deg [6]float64
	for i := range deg {
		deg[i] = rdkutils.RadToDeg(positions[i])
	}
	return deg
}

// MoveToJointPositions moves the arm's joints to the given positions via a
// single blocking MoveJ() (manual §2.1). See moveToJointPositionsOnce for
// the actual mechanics — this just wraps it with this module's own
// single-operation bookkeeping, since it's a standalone public entry point.
func (s *viamFairinoFairino) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	ctx, done := s.opMgr.New(ctx)
	defer done()
	if err := s.checkReadyToMove(ctx); err != nil {
		return err
	}
	if err := s.checkJointTargets(ctx, [][]referenceframe.Input{positions}); err != nil {
		return err
	}
	speedPct, err := s.moveSpeedPercent(nil, extra)
	if err != nil {
		return err
	}
	fr, err := s.latestFrame(ctx)
	if err != nil {
		return err
	}
	sent, err := s.moveToJointPositionsOnce(ctx, positions, blendTBlocking, speedPct,
		maxJointTravelDeg(fr.JointDegs, inputsToDeg(positions)))
	if err != nil && sent {
		// Cancelled or failed after the MoveJ reached the controller, which
		// would otherwise carry on with it.
		s.stopDetached()
	}
	return err
}

// checkReadyToMove refuses to start a move while the controller is e-stopped
// or latched in a fault, rather than sending a MoveJ that it will reject or
// abandon. The documented command protocol (V3.9.9) has no command to clear a
// fault (the vendor SDK's ResetAllError isn't on port 8080), so unlike
// viam-ufactory-xarm this can't recover automatically: the error says what the
// fault is and where to clear it.
func (s *viamFairinoFairino) checkReadyToMove(ctx context.Context) error {
	fr, err := s.latestFrame(ctx)
	if err != nil {
		return err
	}
	// A feed that is connected but has stopped sending leaves an old frame
	// behind, which can't vouch for the controller's state now.
	if age := time.Since(fr.ReceivedAt); age > statusFeedLostTimeout {
		return fmt.Errorf("status stream stalled: no new frame for %v; not moving", age.Round(time.Millisecond))
	}
	if fr.EmergencyStop {
		return errors.New("Fairino controller is e-stopped: release the emergency stop before moving")
	}
	if fr.ErrorCode == 0 {
		return nil
	}
	detail := ""
	if mainCode, subCode, err := s.cmd.getRobotErrorCode(ctx); err == nil {
		detail = fmt.Sprintf(" (main code %d, sub code %d)", mainCode, subCode)
	}
	return fmt.Errorf("Fairino controller is in a fault state: %s%s; clear it on the teach pendant or WebApp before moving",
		faultCodeString(fr.ErrorCode), detail)
}

// checkJointTargets rejects, before anything is sent to the controller, any
// target with the wrong number of joints (which would otherwise index past the
// end of the slice), a NaN or infinite position, or one outside the kinematic
// model's joint limits. A joint already outside its limits may still move back
// toward them — that is arm.CheckDesiredJointPositions's rule, the same check
// viam-ufactory-xarm runs. NaN has to be caught here: it compares false against
// both limits, so CheckDesiredJointPositions lets it through, and it would
// reach the controller as the literal text "NaN".
func (s *viamFairinoFairino) checkJointTargets(ctx context.Context, targets [][]referenceframe.Input) error {
	dof := len(s.model.DoF())
	for i, target := range targets {
		if len(target) != dof {
			return fmt.Errorf("waypoint %d/%d has %d joint positions, the arm has %d joints", i+1, len(targets), len(target), dof)
		}
		for j, v := range target {
			if math.IsNaN(v) || math.IsInf(v, 0) {
				return fmt.Errorf("waypoint %d/%d: joint %d position is %v", i+1, len(targets), j, v)
			}
		}
		if err := arm.CheckDesiredJointPositions(ctx, s, target); err != nil {
			return fmt.Errorf("waypoint %d/%d: %w", i+1, len(targets), err)
		}
	}
	return nil
}

// blendTBlocking is moveJParams.BlendT's "-1": decelerate to a full stop at
// this waypoint before any next queued motion.
const blendTBlocking = -1

// moveToJointPositionsOnce sends a single MoveJ() (manual §2.1) and, for a
// blocking one, waits for completion: until the controller reports the move
// complete (statusFrame.MotionDone) with the joints within tolerance of the
// target, or ctx is cancelled, or the move outlasts motionTimeout for
// travelDeg — the largest joint travel still to cover, queued blends included.
//
// sent reports whether a MoveJ was handed to the controller (even if its ack
// then failed), in which case the arm may be moving. This never sends STOP
// itself: stopping is the job of the caller, which owns the operation
// (MoveToJointPositions, MoveThroughJointPositions) and knows whether earlier
// blended waypoints are still queued too.
//
// blendTBlocking waits for physical completion. Any positive blendT (a blend —
// see Config.BlendTimeMs) returns as soon as the command is acked; the caller
// must send the next waypoint immediately so the controller's own motion
// queue blends between them. blendT == 0 is rejected: a 0ms blend is
// indistinguishable from a full stop, so it's never a meaningful value to
// send.
func (s *viamFairinoFairino) moveToJointPositionsOnce(ctx context.Context, positions []referenceframe.Input, blendT, speedPct, travelDeg float64) (sent bool, err error) {
	if blendT == 0 {
		return false, errors.New("moveToJointPositionsOnce: blendT must not be 0 (0ms blend is no different from a full stop)")
	}
	if len(positions) != 6 {
		return false, fmt.Errorf("moveToJointPositionsOnce: expected 6 joint positions, got %d", len(positions))
	}
	moveStart := time.Now()
	s.logger.Debugw("moveToJointPositionsOnce: called", "positions_rad", positions)

	targetDeg := inputsToDeg(positions)
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
		return false, fmt.Errorf("GetForwardKin: %w", err)
	}
	s.logger.Debugw("moveToJointPositionsOnce: forward kinematics for target", "target_deg", targetDeg, "target_pose", targetPose)

	params := moveJParams{
		Joints:       targetDeg,
		Pose:         targetPose,
		ToolNum:      s.cfg.ToolNum,
		WorkPieceNum: s.cfg.WorkpieceNum,
		SpeedPercent: speedPct,
		AccelPercent: 0, // reserved/non-functional on the controller — see moveJParams.AccelPercent
		OverridePct:  100,
		BlendT:       blendT,
	}

	if err := s.cmd.moveJ(ctx, params); err != nil {
		s.logger.Debugw("moveToJointPositionsOnce: MoveJ ack failed", "elapsed", time.Since(moveStart), "err", err)
		// The frame may have reached the controller even though its ack didn't
		// come back, so this counts as sent.
		return true, fmt.Errorf("MoveJ: %w", err)
	}

	if blendT > 0 {
		// Blend: queued, blending into whatever's sent next — nothing to wait
		// for. blendT > 0 rather than != blendTBlocking as a precaution: any
		// non-positive value falls through to waiting.
		s.logger.Debugw("moveToJointPositionsOnce: MoveJ acknowledged (blend), not waiting for completion", "elapsed", time.Since(moveStart))
		return true, nil
	}
	s.logger.Debugw("moveToJointPositionsOnce: MoveJ acknowledged, waiting for motion to complete", "elapsed", time.Since(moveStart))

	err = s.waitForMotionComplete(ctx, targetDeg, motionTimeout(travelDeg, speedPct))
	s.logger.Debugw("moveToJointPositionsOnce: done", "elapsed", time.Since(moveStart), "err", err)
	return true, err
}

// waitForMotionComplete polls the status stream until the controller
// reports the move done (statusFrame.MotionDone) and the joints are within
// jointToleranceDeg of targetDeg. It returns an error if the controller
// e-stops or latches a fault mid-move, if the status feed goes quiet for
// statusFeedLostTimeout, or if the move takes longer than timeout, and
// ctx.Err() on cancellation — same shape as viam-dobot's
// waitForMotionCompleteLocked, minus the STOP, which the caller sends.
func (s *viamFairinoFairino) waitForMotionComplete(ctx context.Context, targetDeg [6]float64, timeout time.Duration) error {
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

	deadline := pollStart.Add(timeout)
	lastFrame := time.Now()
	for {
		select {
		case <-ctx.Done():
			s.logger.Debugw("waitForMotionComplete: ctx cancelled", "elapsed", time.Since(pollStart), "ctx_err", ctx.Err())
			return ctx.Err()
		case <-time.After(motionPollInterval):
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("move did not finish within %v", timeout)
		}

		fr, ok, feedErr := s.status.latest()
		if ok && time.Since(fr.ReceivedAt) > statusFeedLostTimeout {
			// Connected, but no new frame: as good as no feed.
			ok, feedErr = false, errors.New("no new status frames")
		}
		if !ok {
			s.logger.Debugw("waitForMotionComplete: completion poll — no status frame", "elapsed", time.Since(pollStart), "err", feedErr)
			if time.Since(lastFrame) > statusFeedLostTimeout {
				return fmt.Errorf("status feed lost during motion: %w", feedErr)
			}
			continue
		}
		lastFrame = time.Now()
		s.logger.Debugw("waitForMotionComplete: completion poll", "elapsed", time.Since(pollStart),
			"motion_done", fr.MotionDone, "joint_deg", fr.JointDegs, "target_deg", targetDeg,
			"error_code", fr.ErrorCode, "e_stop", fr.EmergencyStop)
		if fr.EmergencyStop {
			return errors.New("Fairino controller e-stopped during motion")
		}
		if fr.ErrorCode != 0 {
			return fmt.Errorf("Fairino controller reported a fault during motion: %s", faultCodeString(fr.ErrorCode))
		}
		if fr.MotionDone {
			if jointsCloseDeg(fr.JointDegs, targetDeg, jointToleranceDeg) {
				s.logger.Debugw("waitForMotionComplete: motion complete, joints within tolerance", "elapsed", time.Since(pollStart), "joint_deg", fr.JointDegs)
				return nil
			}
			s.logger.Debugw("waitForMotionComplete: motion done but joints out of tolerance", "elapsed", time.Since(pollStart), "joint_deg", fr.JointDegs, "target_deg", targetDeg)
			return fmt.Errorf("motion completed but joints did not reach target (got %v want %v degrees)", fr.JointDegs, targetDeg)
		}
	}
}

// stopDetached sends STOP for a move that is ending early — cancelled, or
// failed — after sending motion the controller may still be executing (or
// have queued, when blending). The move's own context may be done by now;
// commandClient.stop runs on a deadline of its own either way.
func (s *viamFairinoFairino) stopDetached() {
	if err := s.cmd.stop(context.Background()); err != nil {
		s.logger.Warnw("failed to send STOP after cancelling a move", "err", err)
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

// Close cancels any move still in flight before releasing the controller
// connections, so the arm doesn't keep executing it after the resource is
// gone (e.g. on reconfigure). The cancelled move sends STOP on its way out,
// and CancelRunning waits for that. With no move of ours running, nothing is
// sent: the command client is shared with the gripper, and a STOP there is
// only ours to send when we started the motion. Only the first call does
// anything: the connections are reference-counted and shared with the other
// resources on this controller, so a second release would take theirs.
func (s *viamFairinoFairino) Close(ctx context.Context) error {
	s.closeOnce.Do(func() {
		s.cancelPlans(true)
		s.opMgr.CancelRunning(ctx)
		s.cmd.dropSetup(s.name.String())
		s.webApp.stop()
		releaseStatusClient(s.host, statusPort)
		releaseCommandClient(s.host, commandPort)
	})
	return nil
}

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
//
// Deliberately not registered with s.opMgr (same as viam-ufactory-xarm): the
// motion service calls back into GoToInputs over gRPC with a fresh context,
// so GoToInputs's own opMgr.New can't see this call as its parent. If this
// held an operation, that New would cancel it and wait for it to return,
// while it is itself waiting on the motion service — the plan would fail
// every time. GoToInputs owns the operation instead, which is also what
// makes Stop halt a plan mid-execution. The call is tracked in s.plans
// instead, so Stop and Close cancel it even while it is still planning —
// otherwise a plan finished after a reconfigure would run on the rebuilt arm.
func (s *viamFairinoFairino) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	if s.motion == nil {
		return errors.New("fairino cannot do MoveToPosition without a motion service available (see MoveToPosition's doc comment)")
	}
	ctx, cancel := context.WithCancel(ctx)
	defer cancel()
	untrack, err := s.trackPlan(&cancel)
	if err != nil {
		return err
	}
	defer untrack()
	_, err = s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.Name().Name,
		Destination:   referenceframe.NewPoseInFrame(fmt.Sprintf("%s_origin", s.Name().Name), pose),
	})
	return err
}

// trackPlan registers an in-flight MoveToPosition's cancel func; the returned
// func unregisters it.
func (s *viamFairinoFairino) trackPlan(cancel *context.CancelFunc) (func(), error) {
	s.plansMu.Lock()
	defer s.plansMu.Unlock()
	if s.closed {
		return nil, errors.New("arm is closed")
	}
	if s.plans == nil {
		s.plans = map[*context.CancelFunc]struct{}{}
	}
	s.plans[cancel] = struct{}{}
	return func() {
		s.plansMu.Lock()
		delete(s.plans, cancel)
		s.plansMu.Unlock()
	}, nil
}

// cancelPlans cancels every in-flight MoveToPosition; with closing, it also
// refuses any started after this.
func (s *viamFairinoFairino) cancelPlans(closing bool) {
	s.plansMu.Lock()
	defer s.plansMu.Unlock()
	s.closed = s.closed || closing
	for cancel := range s.plans {
		(*cancel)()
	}
}

// planning reports whether a MoveToPosition is in flight.
func (s *viamFairinoFairino) planning() bool {
	s.plansMu.Lock()
	defer s.plansMu.Unlock()
	return len(s.plans) > 0
}

// MoveThroughJointPositions moves the arm's joints through the given
// positions in order, via moveToJointPositionsOnce per waypoint, as a single
// s.opMgr operation — so a new move or Stop cancels the whole sequence.
//
// Speed comes from options/extra when given, else the current default (see
// moveSpeedPercent). MaxAccRads isn't consumed: MoveJ's acceleration field is
// reserved on the controller (see moveJParams.AccelPercent).
//
// Blending: with Config.BlendTimeMs nonzero and more than one waypoint, every
// waypoint but the last blends (non-blocking, no completion wait); the last
// always blocks, so this still only returns once the arm is at rest at the
// final target. Applies equally to GoToInputs/MoveToPosition's IK-planned
// trajectories, since they delegate here too — see the README for the
// resulting "cut corner" caveat.
func (s *viamFairinoFairino) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
	ctx, done := s.opMgr.New(ctx)
	defer done()
	if err := s.checkReadyToMove(ctx); err != nil {
		return err
	}
	if err := s.checkJointTargets(ctx, positions); err != nil {
		return err
	}
	speedPct, err := s.moveSpeedPercent(options, extra)
	if err != nil {
		return err
	}
	fr, err := s.latestFrame(ctx)
	if err != nil {
		return err
	}
	// travel accumulates the joint travel queued since the last blocking
	// waypoint, which is what the next completion wait has to allow for.
	prevDeg, travel, sent := fr.JointDegs, 0.0, false
	for i, goal := range positions {
		// The last waypoint always blocks; intermediate waypoints blend when
		// Config.BlendTimeMs is set, otherwise they block too.
		blendT := float64(blendTBlocking)
		if s.cfg.BlendTimeMs > 0 && i < len(positions)-1 {
			blendT = float64(s.cfg.BlendTimeMs)
		}
		goalDeg := inputsToDeg(goal)
		travel += maxJointTravelDeg(prevDeg, goalDeg)
		prevDeg = goalDeg
		wpSent, err := s.moveToJointPositionsOnce(ctx, goal, blendT, speedPct, travel)
		sent = sent || wpSent
		if err != nil {
			if sent {
				// Cancelled or failed partway through: this waypoint, and
				// earlier ones still queued when blending, would keep running.
				s.stopDetached()
			}
			return fmt.Errorf("waypoint %d/%d: %w", i+1, len(positions), err)
		}
		if blendT == blendTBlocking {
			travel = 0
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

// Stop halts the arm. It first cancels any MoveToPosition still planning and
// any in-flight move this module is running (opMgr.New blocks until that move
// has returned — it sends STOP itself on the way out), then sends STOP (manual
// §2.35) regardless, so the arm also stops if the motion was queued some other
// way. The STOP goes out even if ctx is already done (see commandClient.stop).
func (s *viamFairinoFairino) Stop(ctx context.Context, extra map[string]interface{}) error {
	s.cancelPlans(false)
	ctx, done := s.opMgr.New(ctx)
	defer done()
	return s.cmd.stop(ctx)
}

// errNotImplemented marks arm.Arm methods added in newer RDK versions that
// this module doesn't support yet (see the CONSULT-2244 epic).
var errNotImplemented = errors.New("not implemented yet for the Fairino arm")

// Properties reports what this arm supports. Cartesian commands work via the
// motion service (MoveToPosition), so they are only reported when one is
// available. Manual mode is not reported yet: the
// SetManualMode/ManualMode API isn't wired up (see RSDK-14463), although
// drag-teach is reachable via DoCommand's enter_drag_mode/exit_drag_mode.
func (s *viamFairinoFairino) Properties(ctx context.Context, extra map[string]interface{}) (arm.Properties, error) {
	return arm.Properties{SupportCartesianCommands: s.motion != nil}, nil
}

// SetManualMode is not implemented yet.
func (s *viamFairinoFairino) SetManualMode(ctx context.Context, on bool, enabledFor time.Duration, extra map[string]interface{}) error {
	return errNotImplemented
}

// ManualMode is not implemented yet.
func (s *viamFairinoFairino) ManualMode(ctx context.Context, extra map[string]interface{}) (bool, error) {
	return false, errNotImplemented
}

// MoveThroughJointPositionsStreamed is not implemented yet.
func (s *viamFairinoFairino) MoveThroughJointPositionsStreamed(
	ctx context.Context,
	batches <-chan []arm.TrajectoryPoint,
	responses chan<- arm.Response,
	extra map[string]interface{},
) error {
	return errNotImplemented
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
