package viamfairino

import (
	"context"
	"errors"
	"fmt"
	"math"
	"net"
	"strconv"
	"strings"
	"sync"
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

// Bad targets must be rejected before any command reaches the controller.
func TestJointTargetsValidatedBeforeSending(t *testing.T) {
	for name, target := range map[string][]referenceframe.Input{
		"too few joints":  {0, 0, 0},
		"too many joints": {0, 0, 0, 0, 0, 0, 0},
		"beyond a limit":  {0, 0, 0, 0, 0, 100},
	} {
		t.Run(name, func(t *testing.T) {
			fc := newFakeController(t)
			fc.setReply(fakeFKReply)
			s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))

			if err := s.MoveToJointPositions(context.Background(), target, nil); err == nil {
				t.Error("MoveToJointPositions: expected an error")
			}
			waypoints := [][]referenceframe.Input{testTarget(), target}
			if err := s.MoveThroughJointPositions(context.Background(), waypoints, nil, nil); err == nil {
				t.Error("MoveThroughJointPositions: expected an error")
			}
			if sent := fc.sent(); len(sent) != 0 {
				t.Errorf("commands sent for an invalid target: %v", sent)
			}
		})
	}
}

func TestCloseStopsInFlightMove(t *testing.T) {
	fc := newFakeController(t)
	fc.setReply(fakeFKReply)
	s, _ := newControllerTestArm(t, fc, movingFrame())
	s.host = "127.0.0.1"

	errCh := make(chan error, 1)
	go func() { errCh <- s.MoveToJointPositions(context.Background(), testTarget(), nil) }()
	waitFor(t, "MoveJ to be sent", func() bool { return fc.count("MoveJ") > 0 })

	if err := s.Close(context.Background()); err != nil {
		t.Fatal(err)
	}
	// Close waits for the move's operation to end, which the move goroutine
	// then reports a moment later, so this can't be a non-blocking check.
	select {
	case err := <-errCh:
		if !errors.Is(err, context.Canceled) {
			t.Errorf("move returned %v, want context.Canceled", err)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("move did not return after Close")
	}
	if fc.count("STOP") == 0 {
		t.Errorf("no STOP sent on Close; commands %v", fc.sent())
	}
}

// With nothing of ours moving, Close must not send STOP: the command client is
// shared with the gripper and the arm may be under someone else's control.
func TestCloseWhenIdleSendsNothing(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	s.host = "127.0.0.1"
	if err := s.Close(context.Background()); err != nil {
		t.Fatal(err)
	}
	if sent := fc.sent(); len(sent) != 0 {
		t.Errorf("idle Close sent %v", sent)
	}
}

func TestSendRefusesCancelledContext(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if _, err := s.cmd.getForwardKin(ctx, [6]float64{}); !errors.Is(err, context.Canceled) {
		t.Errorf("got %v, want context.Canceled", err)
	}
	if sent := fc.sent(); len(sent) != 0 {
		t.Errorf("sent %v on a cancelled context", sent)
	}
}

// STOP is the exception: it is most often sent for a context that is ending.
func TestStopSentOnCancelledContext(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if err := s.Stop(ctx, nil); err != nil {
		t.Fatal(err)
	}
	if fc.count("STOP") != 1 {
		t.Errorf("STOP not sent on a cancelled context: %v", fc.sent())
	}
}

// A connection that died unnoticed must not swallow a STOP: the failed write
// drops it, and STOP goes out again on a fresh one.
func TestStopRetriesOnADeadConnection(t *testing.T) {
	fc := newFakeController(t)
	fc.setReply(fakeFKReply)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	if _, err := s.cmd.getForwardKin(context.Background(), [6]float64{}); err != nil {
		t.Fatal(err)
	}
	s.cmd.connMu.Lock()
	_ = s.cmd.conn.Close() // dead, but still the client's connection
	s.cmd.connMu.Unlock()
	if err := s.cmd.stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	if fc.count("STOP") != 1 {
		t.Errorf("STOP not delivered: %v", fc.sent())
	}
}

// A STOP must not wait behind other commands queued for the shared connection,
// only behind the one already on the wire.
func TestStopJumpsTheCommandQueue(t *testing.T) {
	fc := newFakeController(t)
	fc.setReply(func(cmdID int, _ string) string {
		switch cmdID {
		case cmdIDGetForwardKin:
			time.Sleep(300 * time.Millisecond)
			return "100,200,300,0,0,0"
		case cmdIDGetRobotErrorCode:
			return "0,0"
		}
		return "1"
	})
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	ctx := context.Background()

	var wg sync.WaitGroup
	wg.Add(2)
	go func() { defer wg.Done(); _, _ = s.cmd.getForwardKin(ctx, [6]float64{}) }()
	waitFor(t, "the slow command to be on the wire", func() bool { return fc.count("GetForwardKin") == 1 })
	go func() { defer wg.Done(); _, _, _ = s.cmd.getRobotErrorCode(ctx) }()
	time.Sleep(50 * time.Millisecond) // let it queue
	if err := s.cmd.stop(ctx); err != nil {
		t.Fatal(err)
	}
	wg.Wait()
	want := []string{"GetForwardKin", "STOP", "GetRobotErrorCode"}
	if got := fc.sent(); fmt.Sprint(got) != fmt.Sprint(want) {
		t.Errorf("got order %v, want %v", got, want)
	}
}

// A blended sequence that fails (not just one that's cancelled) must stop the
// waypoints it already queued on the controller.
func TestFailedBlendedSequenceStops(t *testing.T) {
	fc := newFakeController(t)
	moveJs := 0
	fc.setReply(func(cmdID int, data string) string {
		switch cmdID {
		case cmdIDGetForwardKin:
			return "100,200,300,0,0,0"
		case cmdIDMoveJ:
			moveJs++
			if moveJs == 2 {
				return "154"
			}
		}
		return "1"
	})
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	s.cfg = &Config{BlendTimeMs: 100}
	err := s.MoveThroughJointPositions(context.Background(), [][]referenceframe.Input{testTarget(), testTarget(), testTarget()}, nil, nil)
	if err == nil || !strings.Contains(err.Error(), "waypoint 2/3") {
		t.Fatalf("got %v, want waypoint 2's MoveJ error", err)
	}
	if fc.count("STOP") != 1 {
		t.Errorf("queued blended waypoints not stopped: %v", fc.sent())
	}
}

// NaN compares false against both joint limits, so it needs its own check.
func TestNonFiniteJointTargetsRejected(t *testing.T) {
	for _, v := range []float64{math.NaN(), math.Inf(1)} {
		fc := newFakeController(t)
		s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
		target := testTarget()
		target[2] = v
		if err := s.MoveToJointPositions(context.Background(), target, nil); err == nil {
			t.Errorf("%v: expected an error", v)
		}
		if sent := fc.sent(); len(sent) != 0 {
			t.Errorf("%v: sent %v", v, sent)
		}
	}
}

// With no status feed there's no seeing a fault or the move finishing, so the
// move is abandoned and the arm stopped.
func TestMoveStopsWhenStatusFeedIsLost(t *testing.T) {
	fc := newFakeController(t)
	fc.setReply(fakeFKReply)
	s, sc := newControllerTestArm(t, fc, movingFrame())
	errCh := make(chan error, 1)
	go func() { errCh <- s.MoveToJointPositions(context.Background(), testTarget(), nil) }()
	waitFor(t, "MoveJ", func() bool { return fc.count("MoveJ") == 1 })
	sc.recordError(errors.New("status read: connection reset"))
	select {
	case err := <-errCh:
		if err == nil || !strings.Contains(err.Error(), "status feed lost") {
			t.Errorf("got %v, want a lost-feed error", err)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("move kept waiting with no status feed")
	}
	if fc.count("STOP") != 1 {
		t.Errorf("arm not stopped: %v", fc.sent())
	}
}

// Close must release the shared connections once, however often it's called:
// the references it drops are shared with the gripper.
func TestCloseIsIdempotent(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	// TEST-NET-3: the shared client connects lazily, and never does here.
	s.host = "203.0.113.3"
	acquireCommandClient(s.host, commandPort, s.logger) // the arm's
	acquireCommandClient(s.host, commandPort, s.logger) // a gripper's
	t.Cleanup(func() { releaseCommandClient(s.host, commandPort) })

	for range 2 {
		if err := s.Close(context.Background()); err != nil {
			t.Fatal(err)
		}
	}
	sharedCommandMu.Lock()
	e, ok := sharedCommandClients[net.JoinHostPort(s.host, strconv.Itoa(commandPort))]
	sharedCommandMu.Unlock()
	if !ok || e.refs != 1 {
		t.Errorf("after two Closes the gripper's reference is gone (entry present %v)", ok)
	}
}

// A faulted or e-stopped controller must be reported up front, with the fault
// decoded, and no MoveJ sent into it.
func TestMoveRefusedWhileFaultedOrEStopped(t *testing.T) {
	for name, tc := range map[string]struct {
		frame   statusFrame
		wantMsg string
	}{
		"e-stop": {statusFrame{EmergencyStop: true, MotionDone: true}, "e-stopped"},
		"fault":  {statusFrame{ErrorCode: 3, MotionDone: true}, "collision failure (main code 3, sub code 7)"},
	} {
		t.Run(name, func(t *testing.T) {
			fc := newFakeController(t)
			fc.setReply(func(cmdID int, _ string) string {
				if cmdID == cmdIDGetRobotErrorCode {
					return "3,7"
				}
				return "1"
			})
			s, _ := newControllerTestArm(t, fc, tc.frame)
			err := s.MoveToJointPositions(context.Background(), testTarget(), nil)
			if err == nil || !strings.Contains(err.Error(), tc.wantMsg) {
				t.Errorf("got %v, want an error containing %q", err, tc.wantMsg)
			}
			if fc.count("MoveJ") != 0 {
				t.Errorf("MoveJ sent to a controller that can't move: %v", fc.sent())
			}
		})
	}
}

// A feed that stays connected but stops delivering frames (say, every frame
// fails to parse) is no feed either.
func TestMoveStopsWhenStatusFeedFreezes(t *testing.T) {
	fc := newFakeController(t)
	fc.setReply(fakeFKReply)
	s, _ := newControllerTestArm(t, fc, movingFrame()) // never refreshed
	start := time.Now()
	err := s.MoveToJointPositions(context.Background(), testTarget(), nil)
	if err == nil || !strings.Contains(err.Error(), "status feed lost") {
		t.Errorf("got %v, want a lost-feed error", err)
	}
	if elapsed := time.Since(start); elapsed > 3*statusFeedLostTimeout {
		t.Errorf("took %v to notice", elapsed)
	}
	if fc.count("STOP") != 1 {
		t.Errorf("arm not stopped: %v", fc.sent())
	}
}
