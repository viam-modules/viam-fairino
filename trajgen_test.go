package viamfairino

import (
	"context"
	"testing"

	"go.viam.com/rdk/ml"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/testutils/inject"
	rdkutils "go.viam.com/rdk/utils"
	"gorgonia.org/tensor"
)

// fakeTrajGen returns the given samples (N x 6, radians), recording the
// velocity limit it was asked for.
func fakeTrajGen(samples [][]float64, gotVel *float64) *inject.MLModelService {
	svc := inject.NewMLModelService("trajex")
	svc.InferFunc = func(_ context.Context, in ml.Tensors) (ml.Tensors, error) {
		if gotVel != nil {
			*gotVel = in["velocity_limits_rads_per_sec"].Data().([]float64)[0]
		}
		if len(samples) == 0 {
			return ml.Tensors{}, nil
		}
		flat := []float64{}
		for _, s := range samples {
			flat = append(flat, s...)
		}
		return ml.Tensors{"configurations_rads": tensor.New(tensor.WithShape(len(samples), 6), tensor.WithBacking(flat))}, nil
	}
	return svc
}

func TestMoveThroughUsesTrajectoryGenerator(t *testing.T) {
	fc := newFakeController(t)
	final := []float64{0.02, 0.02, 0.02, 0.02, 0.02, 0.02}
	var deg [6]float64
	for i := range deg {
		deg[i] = rdkutils.RadToDeg(final[i])
	}
	s, _ := newControllerTestArm(t, fc, restingFrame(deg))
	var gotVel float64
	s.cfg = &Config{SpeedDegsPerSec: 90, TrajGen: &TrajGenConfig{Service: "trajex"}}
	s.trajGen = fakeTrajGen([][]float64{make([]float64, 6), {0.01, 0.01, 0.01, 0.01, 0.01, 0.01}, final}, &gotVel)

	if err := s.MoveThroughJointPositions(context.Background(), [][]referenceframe.Input{final}, nil, nil); err != nil {
		t.Fatal(err)
	}
	if n := fc.count("ServoJ"); n != 2 {
		t.Errorf("sent %d ServoJ, want 2 (every sample but the t=0 one): %v", n, fc.sent())
	}
	if fc.count("MoveJ") != 0 {
		t.Error("MoveJ used even though a trajectory generator is configured")
	}
	if want := rdkutils.DegToRad(90); gotVel < want-1e-9 || gotVel > want+1e-9 {
		t.Errorf("generator got velocity limit %v rad/s, want %v", gotVel, want)
	}
}

// When the generator says there's nothing to do, nothing is sent.
func TestTrajectoryGeneratorAlreadyAtGoal(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	s.cfg = &Config{TrajGen: &TrajGenConfig{Service: "trajex"}}
	s.trajGen = fakeTrajGen(nil, nil)
	if err := s.MoveThroughJointPositions(context.Background(), [][]referenceframe.Input{make([]float64, 6)}, nil, nil); err != nil {
		t.Fatal(err)
	}
	if sent := fc.sent(); len(sent) != 0 {
		t.Errorf("sent %v", sent)
	}
}

func TestValidateTrajGen(t *testing.T) {
	for _, tg := range []*TrajGenConfig{{}, {Service: "t", MaxAccelerationDegsPerSec2: -1}} {
		if _, _, err := (&Config{TrajGen: tg}).Validate("path"); err == nil {
			t.Errorf("%+v: expected an error", tg)
		}
	}
	deps, _, err := (&Config{TrajGen: &TrajGenConfig{Service: "trajex"}}).Validate("path")
	if err != nil || len(deps) != 1 {
		t.Errorf("got deps %v, %v; want the ML model service as a dependency", deps, err)
	}
}
