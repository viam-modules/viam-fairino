package viamfairino

import (
	"context"
	"errors"
	"testing"
	"time"

	"go.viam.com/rdk/referenceframe"
	rdkutils "go.viam.com/rdk/utils"
)

// movingFrame is an arm that never reports the move done, so a blocking move
// against it runs until it is cancelled.
func movingFrame() statusFrame {
	return statusFrame{MotionDone: false}
}

func testTarget() []referenceframe.Input {
	return []referenceframe.Input{0.1, -0.2, 0.3, -0.1, 0.2, 0.1}
}

// waitFor polls cond until it holds or the test times out.
func waitFor(t *testing.T, what string, cond func() bool) {
	t.Helper()
	deadline := time.Now().Add(5 * time.Second)
	for !cond() {
		if time.Now().After(deadline) {
			t.Fatalf("timed out waiting for %s", what)
		}
		time.Sleep(5 * time.Millisecond)
	}
}

func TestStopWithNoMoveSendsStop(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	if err := s.Stop(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	if fc.count("STOP") != 1 {
		t.Errorf("expected one STOP, got commands %v", fc.sent())
	}
}

// Stop must cancel an in-flight move, whichever public entry point started
// it, and leave the controller with a STOP.
func TestStopCancelsInFlightMove(t *testing.T) {
	for name, move := range map[string]func(*viamFairinoFairino) error{
		"MoveToJointPositions": func(s *viamFairinoFairino) error {
			return s.MoveToJointPositions(context.Background(), testTarget(), nil)
		},
		"MoveThroughJointPositions": func(s *viamFairinoFairino) error {
			return s.MoveThroughJointPositions(context.Background(), [][]referenceframe.Input{testTarget(), testTarget()}, nil, nil)
		},
		"GoToInputs": func(s *viamFairinoFairino) error {
			return s.GoToInputs(context.Background(), testTarget())
		},
	} {
		t.Run(name, func(t *testing.T) {
			fc := newFakeController(t)
			fc.setReply(fakeFKReply)
			s, _ := newControllerTestArm(t, fc, movingFrame())

			errCh := make(chan error, 1)
			go func() { errCh <- move(s) }()
			waitFor(t, "MoveJ to be sent", func() bool { return fc.count("MoveJ") > 0 })
			if !s.opMgr.OpRunning() {
				t.Error("move is not registered as an operation")
			}

			if err := s.Stop(context.Background(), nil); err != nil {
				t.Fatal(err)
			}
			select {
			case err := <-errCh:
				if !errors.Is(err, context.Canceled) {
					t.Errorf("move returned %v, want context.Canceled", err)
				}
			case <-time.After(5 * time.Second):
				t.Fatal("move did not return after Stop")
			}
			if fc.count("STOP") == 0 {
				t.Errorf("no STOP sent; commands %v", fc.sent())
			}
		})
	}
}

// A second move supersedes the first, rather than both driving the arm.
func TestNewMoveCancelsPreviousMove(t *testing.T) {
	fc := newFakeController(t)
	fc.setReply(fakeFKReply)
	s, sc := newControllerTestArm(t, fc, movingFrame())

	first := make(chan error, 1)
	go func() {
		first <- s.MoveThroughJointPositions(context.Background(), [][]referenceframe.Input{testTarget()}, nil, nil)
	}()
	waitFor(t, "first MoveJ", func() bool { return fc.count("MoveJ") == 1 })

	// Let the second move complete: the arm reports it is at the target.
	target := testTarget()
	var deg [6]float64
	for i := range deg {
		deg[i] = rdkutils.RadToDeg(target[i])
	}
	second := make(chan error, 1)
	go func() { second <- s.MoveToJointPositions(context.Background(), target, nil) }()

	select {
	case err := <-first:
		if !errors.Is(err, context.Canceled) {
			t.Errorf("first move returned %v, want context.Canceled", err)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("first move was not cancelled by the second")
	}
	setTestFrame(sc, restingFrame(deg))
	select {
	case err := <-second:
		if err != nil {
			t.Errorf("second move: %v", err)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("second move did not complete")
	}
}
