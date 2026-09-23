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
	"fmt"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
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

	moving atomic.Bool
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
	}, nil
}

// setSuction turns suction on or off, driving the release output opposite.
// The release valve closes before suction starts and opens only after it stops,
// so the two never fight.
func (g *fairinoVacuumGripper) setSuction(ctx context.Context, on bool) error {
	g.moving.Store(true)
	defer g.moving.Store(false)
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
	if err := g.setSuction(ctx, true); err != nil {
		return false, err
	}
	if g.cfg.SensorInput == nil {
		return true, nil
	}
	// Vacuum takes a moment to build, and the status feed trails the output
	// by up to a period, so give the switch a moment before reporting a miss.
	deadline := time.Now().Add(vacuumSettleTimeout)
	for {
		st, err := g.IsHoldingSomething(ctx, extra)
		if err != nil || st.IsHoldingSomething || time.Now().After(deadline) {
			return st.IsHoldingSomething, err
		}
		select {
		case <-ctx.Done():
			return false, ctx.Err()
		case <-time.After(motionPollInterval):
		}
	}
}

// vacuumSettleTimeout bounds how long Grab waits for the vacuum switch.
const vacuumSettleTimeout = time.Second

// Open turns suction off (and the release output on, if configured).
func (g *fairinoVacuumGripper) Open(ctx context.Context, extra map[string]any) error {
	return g.setSuction(ctx, false)
}

// IsHoldingSomething reads the sensor input if configured, else the suction
// output; Meta["source"] says which.
func (g *fairinoVacuumGripper) IsHoldingSomething(ctx context.Context, extra map[string]any) (gripper.HoldingStatus, error) {
	fr, ok, err := g.status.latest()
	if !ok {
		if err == nil {
			err = errors.New("no status frame received yet")
		}
		return gripper.HoldingStatus{}, fmt.Errorf("status stream down: %w", err)
	}
	if g.cfg.SensorInput != nil {
		return gripper.HoldingStatus{
			IsHoldingSomething: digitalBit(fr, g.cfg.box(), true, *g.cfg.SensorInput),
			Meta:               map[string]any{"source": "sensor_input"},
		}, nil
	}
	return gripper.HoldingStatus{
		IsHoldingSomething: digitalBit(fr, g.cfg.box(), false, g.cfg.SuctionOutput),
		Meta:               map[string]any{"source": "suction_output"},
	}, nil
}

func (g *fairinoVacuumGripper) Name() resource.Name { return g.name }

func (g *fairinoVacuumGripper) IsMoving(context.Context) (bool, error) { return g.moving.Load(), nil }

// Stop has nothing to halt: suction switches instantly.
func (g *fairinoVacuumGripper) Stop(context.Context, map[string]any) error { return nil }

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

func (g *fairinoVacuumGripper) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	return nil, fmt.Errorf("%w %q: the vacuum gripper has no DoCommands", errUnknownCommand, cmd["command"])
}

func (g *fairinoVacuumGripper) Status(ctx context.Context) (map[string]any, error) {
	st, err := g.IsHoldingSomething(ctx, nil)
	if err != nil {
		return nil, err
	}
	return map[string]any{"holding": st.IsHoldingSomething, "source": st.Meta["source"]}, nil
}

// Close leaves suction as it is: dropping a held part because the module
// restarted would be worse than keeping it.
func (g *fairinoVacuumGripper) Close(ctx context.Context) error {
	releaseStatusClient(g.host, statusPort)
	releaseCommandClient(g.host, commandPort)
	return nil
}
