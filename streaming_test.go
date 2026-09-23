package viamfairino

import (
	"context"
	"errors"
	"strings"
	"testing"
	"time"

	arm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	rdkutils "go.viam.com/rdk/utils"
)

func streamPoints(n int, spacing time.Duration) []arm.TrajectoryPoint {
	pts := make([]arm.TrajectoryPoint, n)
	for i := range pts {
		v := 0.001 * float64(i)
		pts[i] = arm.TrajectoryPoint{Time: time.Duration(i) * spacing, Positions: []referenceframe.Input{v, v, v, v, v, v}}
	}
	return pts
}

// stream runs MoveThroughJointPositionsStreamed over the given batches,
// closing the stream after them, and returns the responses seen and its error.
func stream(t *testing.T, s *viamFairinoFairino, batches ...[]arm.TrajectoryPoint) (int, error) {
	t.Helper()
	in := make(chan []arm.TrajectoryPoint, len(batches))
	for _, b := range batches {
		in <- b
	}
	close(in)
	out := make(chan arm.Response, len(batches))
	err := s.MoveThroughJointPositionsStreamed(context.Background(), in, out, nil)
	return len(out), err
}

func TestStreamedSendsServoJPerPoint(t *testing.T) {
	fc := newFakeController(t)
	pts := streamPoints(5, 8*time.Millisecond)
	last := pts[len(pts)-1].Positions
	var deg [6]float64
	for i := range deg {
		deg[i] = rdkutils.RadToDeg(last[i])
	}
	s, _ := newControllerTestArm(t, fc, restingFrame(deg))

	start := time.Now()
	responses, err := stream(t, s, pts[:2], pts[2:])
	if err != nil {
		t.Fatal(err)
	}
	if responses != 2 {
		t.Errorf("got %d responses, want one per batch (2)", responses)
	}
	if n := fc.count("ServoJ"); n != 4 {
		t.Errorf("sent %d ServoJ, want 4 (every point but the t=0 one): %v", n, fc.sent())
	}
	if elapsed := time.Since(start); elapsed < 30*time.Millisecond {
		t.Errorf("stream took %v; points should be paced by their Time (32ms)", elapsed)
	}
	fc.mu.Lock()
	first := fc.commands[0]
	fc.mu.Unlock()
	if !strings.HasSuffix(first, ",0.0080,0,0)") {
		t.Errorf("first ServoJ %q doesn't carry the 8ms interval", first)
	}
}

// Points too far apart for ServoJ must stop the stream, and the arm.
func TestStreamedRejectsSparsePoints(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	pts := streamPoints(3, 8*time.Millisecond)
	pts[2].Time = 100 * time.Millisecond
	_, err := stream(t, s, pts)
	if err == nil || !strings.Contains(err.Error(), "apart") {
		t.Fatalf("got %v, want a spacing error", err)
	}
	if fc.count("STOP") != 1 {
		t.Errorf("no STOP after aborting mid-stream: %v", fc.sent())
	}
}

func TestStreamedValidatesShape(t *testing.T) {
	for name, mutate := range map[string]func([]arm.TrajectoryPoint){
		"first point not at zero": func(p []arm.TrajectoryPoint) { p[0].Time = time.Millisecond },
		"time goes backwards":     func(p []arm.TrajectoryPoint) { p[2].Time = p[1].Time },
		"wrong joint count":       func(p []arm.TrajectoryPoint) { p[1].Positions = p[1].Positions[:5] },
		"first point moving": func(p []arm.TrajectoryPoint) {
			p[0].Constraints = &arm.KinematicConstraints{Velocities: []float64{0, 1, 0, 0, 0, 0}}
		},
		"beyond a joint limit": func(p []arm.TrajectoryPoint) { p[1].Positions = []referenceframe.Input{0, 0, 0, 0, 0, 100} },
	} {
		t.Run(name, func(t *testing.T) {
			fc := newFakeController(t)
			s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
			pts := streamPoints(3, 8*time.Millisecond)
			mutate(pts)
			if _, err := stream(t, s, pts); err == nil {
				t.Error("expected an error")
			}
		})
	}
}

func TestStopCancelsStream(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	in := make(chan []arm.TrajectoryPoint, 1)
	in <- streamPoints(2, 8*time.Millisecond)
	out := make(chan arm.Response, 1)
	errCh := make(chan error, 1)
	go func() { errCh <- s.MoveThroughJointPositionsStreamed(context.Background(), in, out, nil) }()
	waitFor(t, "first ServoJ", func() bool { return fc.count("ServoJ") > 0 })

	if err := s.Stop(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	select {
	case err := <-errCh:
		if !errors.Is(err, context.Canceled) {
			t.Errorf("got %v, want context.Canceled", err)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("stream didn't end after Stop")
	}
}
