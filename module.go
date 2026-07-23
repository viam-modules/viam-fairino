package viamfairino

import (
	"context"
	"errors"
	"fmt"

	commonpb "go.viam.com/api/common/v1"
	arm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var (
	Fairino          = resource.NewModel("3a94d2f9082b44ad8569a9d8a1330050", "viam-fairino", "fairino")
	errUnimplemented = errors.New("unimplemented")
)

func init() {
	resource.RegisterComponent(arm.API, Fairino,
		resource.Registration[arm.Arm, *Config]{
			Constructor: newViamFairinoFairino,
		},
	)
}

type Config struct {
	/*
		Put config attributes here. There should be public/exported fields
		with a `json` parameter at the end of each attribute.

		Example config struct:
			type Config struct {
				Pin   string `json:"pin"`
				Board string `json:"board"`
				MinDeg *float64 `json:"min_angle_deg,omitempty"`
			}

		If your model does not need a config, replace *Config in the init
		function with resource.NoNativeConfig
	*/
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns three values:
//  1. Required dependencies: other resources that must exist for this resource to work.
//  2. Optional dependencies: other resources that may exist but are not required.
//  3. An error if any Config fields are missing or invalid.
//
// The `path` parameter indicates
// where this resource appears in the machine's JSON configuration
// (for example, "components.0"). You can use it in error messages
// to indicate which resource has a problem.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	// Add config validation code here
	return nil, nil, nil
}

type viamFairinoFairino struct {
	resource.AlwaysRebuild
	resource.Named

	name resource.Name

	logger logging.Logger
	cfg    *Config

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

	s := &viamFairinoFairino{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}
	return s, nil
}

func (s *viamFairinoFairino) Name() resource.Name {
	return s.name
}

// EndPosition returns the current position of the arm.
func (s *viamFairinoFairino) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	var poseRetVal spatialmath.Pose

	return poseRetVal, fmt.Errorf("not implemented")
}

// MoveToPosition moves the arm to the given absolute position.
// This will block until done or a new operation cancels this one.
func (s *viamFairinoFairino) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	return fmt.Errorf("not implemented")
}

// MoveToJointPositions moves the arm's joints to the given positions.
// This will block until done or a new operation cancels this one.
func (s *viamFairinoFairino) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	return fmt.Errorf("not implemented")
}

// MoveThroughJointPositions moves the arm's joints through the given positions in the order they are specified.
// This will block until done or a new operation cancels this one.
func (s *viamFairinoFairino) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
	return fmt.Errorf("not implemented")
}

// MoveThroughJointPositionsStreamed receives a stream of trajectory point batches and executes
// them in order.
//
// The framework feeds one slice per wire TrajectoryBatch into `batches` and closes it when the
// client ends the stream. The implementation writes a Response to `responses` for each
// acknowledgment it wants to send back, and the framework forwards each onto the wire. Return
// nil once the trajectory is done, or an error on a fault, which the framework turns into the
// terminal gRPC status the client sees.
//
// The obvious nested loop is the intended shape:
// for batch := range batches { for _, p := range batch { ... } }
//
// The framework, not the implementation, owns both channels: it writes and closes `batches`, and
// it closes `responses` after the implementation returns. So an implementation only reads
// `batches` and only writes `responses`, and must not close either. That runs against Go's usual
// sender-closes habit, but it is what keeps a driver trivially correct: write responses, return
// when done.
func (s *viamFairinoFairino) MoveThroughJointPositionsStreamed(ctx context.Context, batches <-chan []TrajectoryPoint, responses chan<- Response, extra map[string]interface{}) error {
	return fmt.Errorf("not implemented")
}

// JointPositions returns the current joint positions of the arm.
func (s *viamFairinoFairino) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	return nil, fmt.Errorf("not implemented")
}

func (s *viamFairinoFairino) Stop(ctx context.Context, extra map[string]interface{}) error {
	return fmt.Errorf("not implemented")
}

func (s *viamFairinoFairino) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	var modelRetVal referenceframe.Model

	return modelRetVal, fmt.Errorf("not implemented")
}

func (s *viamFairinoFairino) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, fmt.Errorf("not implemented")
}

func (s *viamFairinoFairino) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return fmt.Errorf("not implemented")
}

func (s *viamFairinoFairino) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, fmt.Errorf("not implemented")
}

func (s *viamFairinoFairino) Status(ctx context.Context) (map[string]interface{}, error) {
	return nil, fmt.Errorf("not implemented")
}

func (s *viamFairinoFairino) IsMoving(ctx context.Context) (bool, error) {
	return false, fmt.Errorf("not implemented")
}

func (s *viamFairinoFairino) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, fmt.Errorf("not implemented")
}

// Get3DModels returns the 3D models of the arm.
func (s *viamFairinoFairino) Get3DModels(ctx context.Context, extra map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	return nil, fmt.Errorf("not implemented")
}

func (s *viamFairinoFairino) Close(context.Context) error {
	// Put close code here
	s.cancelFunc()
	return nil
}
