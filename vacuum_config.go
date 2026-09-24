package viamfairino

import (
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// VacuumGripperConfig says which digital IO the vacuum is wired to.
type VacuumGripperConfig struct {
	// Host is the controller IP; defaults to 192.168.58.2 (factory default).
	Host string `json:"host,omitempty"`

	// IO selects "tool" (flange, outputs/inputs 0-1; the default) or "box"
	// (control box, 0-15) for every pin below.
	IO string `json:"io,omitempty"`

	// SuctionOutput is the output that turns suction on. Default 0.
	SuctionOutput int `json:"suction_output,omitempty"`

	// ReleaseOutput, if set, is driven opposite to SuctionOutput: off while
	// grabbing, on while open (a blow-off or release valve).
	ReleaseOutput *int `json:"release_output,omitempty"`

	// SensorInput, if set, is a vacuum-switch input that is on while a part is
	// held. Without it IsHoldingSomething reports whether suction is on.
	SensorInput *int `json:"sensor_input,omitempty"`

	// LengthMM, if set, adds a suction-cup collision box this long along the
	// approach (Z) axis.
	LengthMM float64 `json:"length_mm,omitempty"`
}

func (cfg *VacuumGripperConfig) box() bool { return cfg.IO == "box" }

// Validate checks that every pin exists on the selected IO.
func (cfg *VacuumGripperConfig) Validate(path string) ([]string, []string, error) {
	n, io := numToolDOs, "tool"
	switch cfg.IO {
	case "", "tool":
	case "box":
		n, io = numBoxDOs, "box"
	default:
		return nil, nil, fmt.Errorf(`%s: io must be "tool" or "box", got %q`, path, cfg.IO)
	}
	pins := map[string]*int{"suction_output": &cfg.SuctionOutput, "release_output": cfg.ReleaseOutput, "sensor_input": cfg.SensorInput}
	for name, p := range pins {
		if p != nil && (*p < 0 || *p >= n) {
			return nil, nil, fmt.Errorf("%s: %s must be between 0 and %d for %s IO, got %d", path, name, n-1, io, *p)
		}
	}
	if cfg.ReleaseOutput != nil && *cfg.ReleaseOutput == cfg.SuctionOutput {
		return nil, nil, fmt.Errorf("%s: release_output must differ from suction_output", path)
	}
	if cfg.LengthMM < 0 {
		return nil, nil, fmt.Errorf("%s: length_mm cannot be negative", path)
	}
	return nil, nil, nil
}

// makeVacuumModel is a 0-DoF model carrying the suction cup's geometry, if any:
// a 20 mm square box lengthMM long, from the flange out along Z.
func makeVacuumModel(name string, lengthMM float64) (referenceframe.Model, error) {
	var geometries []spatialmath.Geometry
	if lengthMM > 0 {
		cup, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{Z: lengthMM / 2}),
			r3.Vector{X: 20, Y: 20, Z: lengthMM}, "suction_cup")
		if err != nil {
			return nil, fmt.Errorf("building vacuum gripper model: %w", err)
		}
		geometries = append(geometries, cup)
	}
	return gripper.MakeModel(name, geometries)
}

// digitalBit reads one digital input or output from the status feed
// (Table 2-2: cl_dgt_*_h/l hold box pins 15-8/7-0, tl_dgt_*_l tool pins 0-1).
func digitalBit(fr statusFrame, box, input bool, index int) bool {
	p := fr.RawPayload
	if !box {
		off := offTlDgtOutputL
		if input {
			off = offTlDgtInputL
		}
		return p[off]&(1<<index) != 0
	}
	lo, hi := offClDgtOutputL, offClDgtOutputH
	if input {
		lo, hi = offClDgtInputL, offClDgtInputH
	}
	if index >= 8 {
		return p[hi]&(1<<(index-8)) != 0
	}
	return p[lo]&(1<<index) != 0
}
