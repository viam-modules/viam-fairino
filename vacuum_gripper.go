// Vacuum gripper driven by the controller's digital outputs. Suction is one
// output (tool flange by default, or control box), optionally with a second
// output driven opposite to it for a blow-off/release valve — the same
// two-pin arrangement viam-ufactory-xarm's vacuum_gripper uses. Hold detection
// reads an optional vacuum-switch digital input; without one it reports the
// suction output's state. All state comes from the status feed (8083).
package viamfairino

import (
	"context"
	"errors"
	"sync"
	"time"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// VacuumGripper is the model triple for the vacuum gripper component.
var VacuumGripper = resource.NewModel("viam-soleng", "viam-fairino", "vacuum_gripper")

func init() {
	resource.RegisterComponent(gripper.API, VacuumGripper,
		resource.Registration[gripper.Gripper, *VacuumGripperConfig]{
			Constructor: newVacuumGripper,
		},
	)
}

type fairinoVacuumGripper struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	host   string
	cfg    *VacuumGripperConfig
	cmd    *commandClient
	status *statusClient
	model  referenceframe.Model

	// opMgr makes Grab and Open one operation at a time, as for the finger
	// gripper: each switches two outputs in turn, and two interleaved
	// sequences could leave suction and the release valve both on.
	opMgr *operation.SingleOperationManager

	closeOnce sync.Once
}

func newVacuumGripper(ctx context.Context, _ resource.Dependencies, rawConf resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	conf, err := resource.NativeConfig[*VacuumGripperConfig](rawConf)
	if err != nil {
		return nil, err
	}
	model, err := makeVacuumModel(rawConf.ResourceName().ShortName(), conf.LengthMM)
	if err != nil {
		return nil, err
	}
	host := conf.Host
	if host == "" {
		host = defaultHost
	}
	return &fairinoVacuumGripper{
		name:   rawConf.ResourceName(),
		logger: logger,
		host:   host,
		cfg:    conf,
		cmd:    acquireCommandClient(host, commandPort, logger),
		status: acquireStatusClient(ctx, host, statusPort, logger),
		model:  model,
		opMgr:  operation.NewSingleOperationManager(),
	}, nil
}

// setSuction turns suction on or off, driving the release output opposite.
// The release valve closes before suction starts and opens only after it
// stops, so the two never fight; callers hold the operation (opMgr), so no
// other sequence interleaves, and one cancelled partway sends nothing more.
func (g *fairinoVacuumGripper) setSuction(ctx context.Context, on bool) error {
	if on && g.cfg.ReleaseOutput != nil {
		if err := g.cmd.setDO(ctx, !g.cfg.box(), *g.cfg.ReleaseOutput, false); err != nil {
			return err
		}
	}
	if err := g.cmd.setDO(ctx, !g.cfg.box(), g.cfg.SuctionOutput, on); err != nil {
		return err
	}
	if !on && g.cfg.ReleaseOutput != nil {
		return g.cmd.setDO(ctx, !g.cfg.box(), *g.cfg.ReleaseOutput, true)
	}
	return nil
}

// Grab turns suction on. It reports whether a part is held when there is a
// sensor input to tell; without one it can only report that suction is on.
func (g *fairinoVacuumGripper) Grab(ctx context.Context, extra map[string]any) (bool, error) {
	ctx, done := g.opMgr.New(ctx)
	defer done()
	if err := g.setSuction(ctx, true); err != nil {
		return false, err
	}
	if g.cfg.SensorInput == nil {
		return true, nil
	}
	// Vacuum takes a moment to build, and the status feed trails the output
	// by up to a period, so only frames captured after suction came on count,
	// and a miss is reported only once the switch has had time to close.
	switchedAt := time.Now()
	deadline := switchedAt.Add(vacuumSettleTimeout)
	for {
		fr, err := g.status.await(ctx)
		if err != nil {
			return false, err
		}
		fresh := fr.ReceivedAt.After(switchedAt)
		if fresh && g.holding(fr) {
			return true, nil
		}
		if fresh && time.Now().After(deadline) {
			return false, nil
		}
		if time.Since(switchedAt) > vacuumSettleTimeout+statusFeedLostTimeout {
			return false, errors.New("no status frame since suction came on: the status feed is stalled")
		}
		if !sleepCtx(ctx, motionPollInterval) {
			return false, ctx.Err()
		}
	}
}

// vacuumSettleTimeout bounds how long Grab waits for the vacuum switch.
const vacuumSettleTimeout = time.Second

// Open turns suction off (and the release output on, if configured).
func (g *fairinoVacuumGripper) Open(ctx context.Context, extra map[string]any) error {
	ctx, done := g.opMgr.New(ctx)
	defer done()
	return g.setSuction(ctx, false)
}

// holding reads the sensor input if configured, else the suction output.
func (g *fairinoVacuumGripper) holding(fr statusFrame) bool {
	if g.cfg.SensorInput != nil {
		return digitalBit(fr, g.cfg.box(), true, *g.cfg.SensorInput)
	}
	return digitalBit(fr, g.cfg.box(), false, g.cfg.SuctionOutput)
}

// source names what holding reads, for Meta and Status.
func (g *fairinoVacuumGripper) source() string {
	if g.cfg.SensorInput != nil {
		return "sensor_input"
	}
	return "suction_output"
}

// IsHoldingSomething reads the sensor input if configured, else the suction
// output; Meta["source"] says which.
func (g *fairinoVacuumGripper) IsHoldingSomething(ctx context.Context, extra map[string]any) (gripper.HoldingStatus, error) {
	fr, err := g.status.await(ctx)
	if err != nil {
		return gripper.HoldingStatus{}, err
	}
	return gripper.HoldingStatus{IsHoldingSomething: g.holding(fr), Meta: map[string]any{"source": g.source()}}, nil
}

func (g *fairinoVacuumGripper) Name() resource.Name { return g.name }

// IsMoving is true while a Grab or Open is switching outputs or, for Grab,
// waiting for the vacuum switch.
func (g *fairinoVacuumGripper) IsMoving(context.Context) (bool, error) {
	return g.opMgr.OpRunning(), nil
}

// Stop cancels an in-flight Grab or Open, which then sends nothing more.
// The outputs stay as they were last switched.
func (g *fairinoVacuumGripper) Stop(ctx context.Context, _ map[string]any) error {
	g.opMgr.CancelRunning(ctx)
	return nil
}

func (g *fairinoVacuumGripper) Geometries(ctx context.Context, _ map[string]any) ([]spatialmath.Geometry, error) {
	gif, err := g.model.Geometries(nil)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

func (g *fairinoVacuumGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.model, nil
}

func (g *fairinoVacuumGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return []referenceframe.Input{}, nil
}

func (g *fairinoVacuumGripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return nil
}

// DoCommand has no commands: the vacuum gripper is all Grab and Open.
func (g *fairinoVacuumGripper) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	return nil, resource.ErrDoUnimplemented
}

func (g *fairinoVacuumGripper) Status(ctx context.Context) (map[string]any, error) {
	fr, err := g.status.current()
	if err != nil {
		return nil, err
	}
	return map[string]any{"holding": g.holding(fr), "source": g.source()}, nil
}

// Close leaves suction as it is: dropping a held part because the module
// restarted would be worse than keeping it. It releases the shared
// connections once: the references are shared with the other resources on
// this controller.
func (g *fairinoVacuumGripper) Close(ctx context.Context) error {
	g.closeOnce.Do(func() {
		g.opMgr.CancelRunning(ctx)
		releaseStatusClient(g.host, statusPort)
		releaseCommandClient(g.host, commandPort)
	})
	return nil
}
