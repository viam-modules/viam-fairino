package viamfairino

import (
	"context"
	"fmt"
	"strings"
	"testing"
	"time"

	"go.viam.com/rdk/ml"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/testutils/inject"
	rdkutils "go.viam.com/rdk/utils"
	"gorgonia.org/tensor"
)

// fakeTrajGen returns the given samples (N x 6, radians), 8 ms apart,
// recording the velocity limit it was asked for. Like trajex, it rejects
// inputs of the wrong dtype.
func fakeTrajGen(samples [][]float64, gotVel *float64) *inject.MLModelService {
	times := make([]float64, len(samples))
	for i := range times {
		times[i] = 0.008 * float64(i)
	}
	return fakeTrajGenTimed(samples, times, gotVel)
}

// fakeTrajGenTimed is fakeTrajGen with the given sample times (seconds).
func fakeTrajGenTimed(samples [][]float64, sampleTimes []float64, gotVel *float64) *inject.MLModelService {
	svc := inject.NewMLModelService("trajex")
	svc.InferFunc = func(_ context.Context, in ml.Tensors) (ml.Tensors, error) {
		for name, tn := range in {
			if tn.Dtype() != tensor.Float64 {
				return nil, fmt.Errorf("input '%s' has wrong dtype", name)
			}
		}
		if gotVel != nil {
			*gotVel = in["velocity_limits_rads_per_sec"].Data().([]float64)[0]
		}
		flat, times := []float64{}, append([]float64(nil), sampleTimes...)
		for _, s := range samples {
			flat = append(flat, s...)
		}
		return ml.Tensors{
			"configurations_rads": tensor.New(tensor.WithShape(len(samples), 6), tensor.WithBacking(flat)),
			"sample_times_sec":    tensor.New(tensor.WithShape(len(samples)), tensor.WithBacking(times)),
		}, nil
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
	s, sc := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	// The arm arrives once the last setpoint is in.
	fc.setReply(func(cmdID int, data string) string {
		if cmdID == cmdIDServoJ && strings.HasPrefix(data, fmt.Sprintf("ServoJ(%.4f", deg[0])) {
			setTestFrame(sc, restingFrame(deg))
		}
		return "1"
	})
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

// A move to where the arm already is never reaches the generator, which
// (like trajex) would refuse a path without two distinct waypoints.
func TestTrajectoryGeneratorAlreadyAtGoal(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	s.cfg = &Config{TrajGen: &TrajGenConfig{Service: "trajex"}}
	gen := fakeTrajGen(nil, nil)
	called := false
	infer := gen.InferFunc
	gen.InferFunc = func(ctx context.Context, in ml.Tensors) (ml.Tensors, error) {
		called = true
		return infer(ctx, in)
	}
	s.trajGen = gen
	here := make([]referenceframe.Input, 6)
	if err := s.MoveThroughJointPositions(context.Background(), [][]referenceframe.Input{here, here}, nil, nil); err != nil {
		t.Fatal(err)
	}
	if called {
		t.Error("the generator was asked for a trajectory with nowhere to go")
	}
	if sent := fc.sent(); len(sent) != 0 {
		t.Errorf("sent %v", sent)
	}
}

// The whole generated trajectory is checked before any of it is sent: one
// that breaks a joint limit partway is refused, not cut off mid-motion.
func TestGeneratedTrajectoryCheckedBeforeSending(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	s.cfg = &Config{TrajGen: &TrajGenConfig{Service: "trajex"}}
	beyond := []float64{0, 0, 0, 0, 0, 100}
	s.trajGen = fakeTrajGen([][]float64{make([]float64, 6), {0.001, 0, 0, 0, 0, 0}, beyond}, nil)
	err := s.MoveThroughJointPositions(context.Background(), [][]referenceframe.Input{{0.01, 0, 0, 0, 0, 0}}, nil, nil)
	if err == nil {
		t.Fatal("expected an error")
	}
	if sent := fc.sent(); len(sent) != 0 {
		t.Errorf("sent %v for a trajectory that breaks a limit", sent)
	}
}

// A generator answer without timing can't be executed.
func TestTrajectoryGeneratorMissingOutputs(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	s.cfg = &Config{TrajGen: &TrajGenConfig{Service: "trajex"}}
	gen := inject.NewMLModelService("trajex")
	gen.InferFunc = func(context.Context, ml.Tensors) (ml.Tensors, error) { return ml.Tensors{}, nil }
	s.trajGen = gen
	if err := s.MoveThroughJointPositions(context.Background(), [][]referenceframe.Input{{0.01, 0, 0, 0, 0, 0}}, nil, nil); err == nil {
		t.Error("expected an error")
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

// trajex puts its last sample (the final waypoint) at the trajectory's exact
// duration, often a hair after the sample before; ServoJ can't be given an
// interval that short, so that sample takes the previous one's place.
func TestGeneratedTrajectoryFoldsACloseFinalSample(t *testing.T) {
	s, _ := newControllerTestArm(t, newFakeController(t), restingFrame([6]float64{}))
	s.cfg = &Config{TrajGen: &TrajGenConfig{Service: "trajex"}}
	step := func(v float64) []float64 { return []float64{v, v, v, v, v, v} }
	s.trajGen = fakeTrajGenTimed(
		[][]float64{step(0), step(0.001), step(0.002), step(0.0021)},
		[]float64{0, 0.008, 0.016, 0.0162}, nil)
	points, err := s.generateTrajectory(context.Background(), [][]referenceframe.Input{step(0.0021)}, defaultJointSpeedPercent)
	if err != nil {
		t.Fatal(err)
	}
	if len(points) != 3 {
		t.Fatalf("got %d points, want 3 (the last folded into the one before)", len(points))
	}
	last := points[2]
	if last.Time != 16200*time.Microsecond || last.Positions[0] != 0.0021 {
		t.Errorf("last point %v at %v, want the final waypoint at 16.2ms", last.Positions, last.Time)
	}
	st := &servoStreamer{s: s}
	for _, p := range points {
		if err := st.check(context.Background(), p); err != nil {
			t.Errorf("folded trajectory fails the stream checks: %v", err)
		}
	}
}
