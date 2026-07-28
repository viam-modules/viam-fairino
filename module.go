// Package viamfairino implements a Fairino FR5-WML as a Viam arm component.
//
// Phase 1 (current): status feedback only. A statusClient (status.go) reads
// the controller's 8083 broadcast and backs the read-only arm.Arm methods
// (JointPositions, CurrentInputs, IsMoving, Status, DoCommand status dump).
// Motion, kinematics, and geometry are stubbed pending Phases 2–3 — see
// PLAN.md.
//
// The wire protocol uses degrees and millimeters (statusdef.go/status.go);
// all conversion to radians/SI happens in this file, at the arm.Arm boundary.
package viamfairino

import (
	"context"
	"errors"
	"fmt"
	"time"

	commonpb "go.viam.com/api/common/v1"
	arm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
)

var Fairino = resource.NewModel("3a94d2f9082b44ad8569a9d8a1330050", "viam-fairino", "fairino")

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
}

// Validate ensures all parts of the config are valid. Host is optional (it
// has a default) and any non-empty value is accepted as-is — a bad address
// surfaces as a dial error from the status client.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

type viamFairinoFairino struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *Config

	status *statusClient

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

func NewFairino(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (arm.Arm, error) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	host := conf.Host
	if host == "" {
		host = defaultHost
	}

	s := &viamFairinoFairino{
		name:       name,
		logger:     logger,
		cfg:        conf,
		status:     newStatusClient(host, statusPort, logger),
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}
	s.status.start(cancelCtx)

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

// DoCommand exposes the full decoded status for bring-up and debugging:
//
//	{"command": "status"} → every Phase 1 field plus payload diagnostics.
func (s *viamFairinoFairino) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	command, _ := cmd["command"].(string)
	switch command {
	case "status":
		fr, err := s.latestFrame(ctx)
		if err != nil {
			return nil, err
		}
		return statusDump(fr), nil
	default:
		return nil, fmt.Errorf("unknown command %q (supported: \"status\")", command)
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

func (s *viamFairinoFairino) Close(context.Context) error {
	s.status.stop()
	s.cancelFunc()
	return nil
}

// ---------- stubbed methods (Phases 2–3, see PLAN.md) ----------

// errPhase2 marks methods that need the command port (8080) client.
var errPhase2 = errors.New("not implemented in phase 1 (status-only); motion via the command port is planned for phase 2 — see PLAN.md")

// errPhase3 marks methods that need the FR5-WML kinematics/geometry models.
var errPhase3 = errors.New("not implemented in phase 1 (status-only); kinematics and geometry are planned for phase 3 — see PLAN.md")

// EndPosition returns the current position of the arm. Deliberately stubbed
// rather than serving the device-reported TCP (tl_cur_pos): its Euler
// convention is unverified, and phase 3 implements this via kinematic-model
// forward kinematics instead.
func (s *viamFairinoFairino) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	return nil, errPhase3
}

// MoveToPosition moves the arm to the given absolute position.
func (s *viamFairinoFairino) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	return errPhase2
}

// MoveToJointPositions moves the arm's joints to the given positions.
func (s *viamFairinoFairino) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	return errPhase2
}

// MoveThroughJointPositions moves the arm's joints through the given positions in the order they are specified.
func (s *viamFairinoFairino) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
	return errPhase2
}

func (s *viamFairinoFairino) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return errPhase2
}

func (s *viamFairinoFairino) Stop(ctx context.Context, extra map[string]interface{}) error {
	return errPhase2
}

func (s *viamFairinoFairino) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return nil, errPhase3
}

func (s *viamFairinoFairino) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, errPhase3
}

// Get3DModels returns the 3D models of the arm.
func (s *viamFairinoFairino) Get3DModels(ctx context.Context, extra map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	return nil, errPhase3
}
