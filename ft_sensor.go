// Fairino wrist force/torque sensor as a Viam sensor. Readings come from the
// status feed (8083, Table 2-2 rows 32-38), which the controller publishes
// every period anyway, so a reading costs no command round-trip; tare and
// activation go over the command port (8080, manual §7.2).
package viamfairino

import (
	"context"
	"errors"
	"fmt"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

// FTSensor is the model triple for the force/torque sensor component.
var FTSensor = resource.NewModel("viam-soleng", "viam-fairino", "ft_sensor")

func init() {
	resource.RegisterComponent(sensor.API, FTSensor,
		resource.Registration[sensor.Sensor, *FTSensorConfig]{
			Constructor: newFTSensor,
		},
	)
}

// FTSensorConfig configures the sensor attached to a controller.
type FTSensorConfig struct {
	// Host is the controller IP; defaults to 192.168.58.2 (factory default).
	Host string `json:"host,omitempty"`
}

// Validate has nothing to check: the sensor has no dependencies.
func (cfg *FTSensorConfig) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

const (
	cmdIDFTActivate = 524 // §7.2.3, Table 7-2-3
	cmdIDFTSetZero  = 528 // §7.2.7, Table 7-2-7
)

// ftReadingKeys names FT_data[0..5] with their units, matching the keys
// viam-ufactory-xarm's ft_sensor uses.
var ftReadingKeys = [6]string{"Fx_N", "Fy_N", "Fz_N", "TRx_Nm", "TRy_Nm", "TRz_Nm"}

type fairinoFTSensor struct {
	resource.AlwaysRebuild

	name   resource.Name
	host   string
	cmd    *commandClient
	status *statusClient
}

func newFTSensor(ctx context.Context, _ resource.Dependencies, rawConf resource.Config, logger logging.Logger) (sensor.Sensor, error) {
	conf, err := resource.NativeConfig[*FTSensorConfig](rawConf)
	if err != nil {
		return nil, err
	}
	host := conf.Host
	if host == "" {
		host = defaultHost
	}
	// Shared with the arm and gripper, like every resource on this controller:
	// see acquireCommandClient/acquireStatusClient.
	return &fairinoFTSensor{
		name:   rawConf.ResourceName(),
		host:   host,
		cmd:    acquireCommandClient(host, commandPort, logger),
		status: acquireStatusClient(ctx, host, statusPort, logger),
	}, nil
}

func (s *fairinoFTSensor) Name() resource.Name { return s.name }

// Readings returns the six force/torque values. A sensor that isn't activated
// reports zeros, so that is an error rather than a confident "no load".
func (s *fairinoFTSensor) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	fr, ok, err := s.status.latest()
	if !ok {
		if err == nil {
			err = errors.New("no status frame received yet")
		}
		return nil, fmt.Errorf("status stream down: %w", err)
	}
	if fr.RawPayload[offFTActStatus] != 1 {
		return nil, errors.New("force/torque sensor is not activated: activate it on the controller " +
			`(WebApp, or DoCommand {"command": "activate"}) before reading`)
	}
	out := make(map[string]interface{}, len(ftReadingKeys))
	for i, key := range ftReadingKeys {
		out[key] = readF64(fr.RawPayload, offFTData+i*8)
	}
	return out, nil
}

// DoCommand supports:
//   - {"command": "tare"}: FT_SetZero(1) — zero the sensor at the current load.
//     Hold the arm still at the unloaded reference pose first.
//   - {"command": "clear_tare"}: FT_SetZero(0) — remove the zero offset.
//   - {"command": "activate"} / {"command": "deactivate"}: FT_Activate(1/0).
func (s *fairinoFTSensor) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	var cmdID int
	var data string
	switch command, _ := cmd["command"].(string); command {
	case "tare":
		cmdID, data = cmdIDFTSetZero, "FT_SetZero(1,0)"
	case "clear_tare":
		cmdID, data = cmdIDFTSetZero, "FT_SetZero(0,0)"
	case "activate":
		cmdID, data = cmdIDFTActivate, "FT_Activate(1)"
	case "deactivate":
		cmdID, data = cmdIDFTActivate, "FT_Activate(0)"
	default:
		return nil, fmt.Errorf("%w %q (supported: tare, clear_tare, activate, deactivate)", errUnknownCommand, command)
	}
	resp, err := s.cmd.send(ctx, cmdID, data)
	if err != nil {
		return nil, err
	}
	if err := parseErrcode(resp); err != nil {
		return nil, err
	}
	return map[string]interface{}{"ok": true}, nil
}

// Status reports whether the sensor is activated, from the status feed.
func (s *fairinoFTSensor) Status(ctx context.Context) (map[string]interface{}, error) {
	fr, ok, err := s.status.latest()
	if !ok {
		if err == nil {
			err = errors.New("no status frame received yet")
		}
		return nil, fmt.Errorf("status stream down: %w", err)
	}
	return map[string]interface{}{"activated": fr.RawPayload[offFTActStatus] == 1}, nil
}

func (s *fairinoFTSensor) Close(ctx context.Context) error {
	releaseStatusClient(s.host, statusPort)
	releaseCommandClient(s.host, commandPort)
	return nil
}
