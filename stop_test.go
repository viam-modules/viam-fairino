package viamfairino

import (
	"bufio"
	"context"
	"errors"
	"net"
	"slices"
	"sync"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

type fakeCommandPort struct {
	ln net.Listener

	mu             sync.Mutex
	received       []int
	at             []time.Time
	fwdKin         int
	failFwdKinFrom int
	slow           map[int]time.Duration
	dropOnce       int
}

func newFakeCommandPort(t *testing.T) *fakeCommandPort {
	t.Helper()
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	f := &fakeCommandPort{ln: ln}
	t.Cleanup(func() { _ = ln.Close() })
	go func() {
		for {
			conn, err := ln.Accept()
			if err != nil {
				return
			}
			go f.serve(conn)
		}
	}()
	return f
}

func (f *fakeCommandPort) serve(conn net.Conn) {
	defer func() { _ = conn.Close() }()
	r := bufio.NewReader(conn)
	for {
		raw, err := readCmdFrame(r)
		if err != nil {
			return
		}
		cnt, cmdID, _, err := decodeCmdFrame(raw)
		if err != nil {
			return
		}
		respID, respData := cmdID, "1"
		f.mu.Lock()
		f.received = append(f.received, cmdID)
		f.at = append(f.at, time.Now())
		if cmdID == cmdIDGetForwardKin {
			f.fwdKin++
			respData = "0,0,0,0,0,0"
			if f.failFwdKinFrom > 0 && f.fwdKin >= f.failFwdKinFrom {
				respID, respData = cmdIDScriptError, `[string "GetForwardKin()"]:1: robot errcode:154`
			}
		}
		delay := f.slow[cmdID]
		if cmdID == f.dropOnce {
			f.dropOnce = 0
			f.mu.Unlock()
			return
		}
		f.mu.Unlock()
		time.Sleep(delay)
		if _, err := conn.Write([]byte(encodeCmdFrame(cnt, respID, respData))); err != nil {
			return
		}
	}
}

func (f *fakeCommandPort) port() int {
	return f.ln.Addr().(*net.TCPAddr).Port
}

func (f *fakeCommandPort) commands() []int {
	f.mu.Lock()
	defer f.mu.Unlock()
	return slices.Clone(f.received)
}

func (f *fakeCommandPort) count(cmdID int) int {
	n := 0
	for _, c := range f.commands() {
		if c == cmdID {
			n++
		}
	}
	return n
}

func (f *fakeCommandPort) waitFor(t *testing.T, cmdID int) {
	t.Helper()
	deadline := time.Now().Add(5 * time.Second)
	for f.count(cmdID) == 0 {
		if time.Now().After(deadline) {
			t.Fatalf("controller never received CMD_ID %d; got %v", cmdID, f.commands())
		}
		time.Sleep(5 * time.Millisecond)
	}
}

type fakeMotion struct {
	motion.Service
	move func(ctx context.Context, req motion.MoveReq) (bool, error)
}

func (m *fakeMotion) Move(ctx context.Context, req motion.MoveReq) (bool, error) {
	return m.move(ctx, req)
}

func newMotionTestArm(t *testing.T, f *fakeCommandPort, fr statusFrame, cfg Config) *viamFairinoFairino {
	t.Helper()
	logger := logging.NewTestLogger(t)
	sc := newStatusClient("test", statusPort, logger)
	sc.frame, sc.have = fr, true
	tickStatus(t, sc, nil)
	toolNum := 0
	cfg.ToolNum = &toolNum
	cmd := newCommandClient("127.0.0.1", f.port(), logger)
	t.Cleanup(func() { _ = cmd.close() })
	return &viamFairinoFairino{
		name:         arm.Named("test"),
		logger:       logger,
		cfg:          &cfg,
		status:       sc,
		cmd:          cmd,
		opMgr:        operation.NewSingleOperationManager(),
		moveToPosOps: operation.NewSingleOperationManager(),
	}
}

// tickStatus publishes a new status frame every 10 ms, applying update to each first.
func tickStatus(t *testing.T, sc *statusClient, update func(*statusFrame)) {
	t.Helper()
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})
	go func() {
		defer close(done)
		ticker := time.NewTicker(10 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
			}
			sc.mu.Lock()
			sc.frame.FrameCount++
			if update != nil {
				update(&sc.frame)
			}
			sc.mu.Unlock()
		}
	}()
	t.Cleanup(func() {
		cancel()
		<-done
	})
}

var (
	movingFrame = statusFrame{MotionDone: false}
	atRestFrame = statusFrame{MotionDone: true}
)

func zeroJoints() []referenceframe.Input {
	return make([]referenceframe.Input, 6)
}

func waitErr(t *testing.T, errCh <-chan error) error {
	t.Helper()
	select {
	case err := <-errCh:
		return err
	case <-time.After(5 * time.Second):
		t.Fatal("call did not return")
		return nil
	}
}

func TestStopCancelsMoveToJointPositions(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, movingFrame, Config{})

	errCh := make(chan error, 1)
	go func() { errCh <- s.MoveToJointPositions(context.Background(), zeroJoints(), nil) }()
	f.waitFor(t, cmdIDMoveJ)

	if err := s.Stop(context.Background(), nil); err != nil {
		t.Fatalf("Stop: %v", err)
	}
	if err := waitErr(t, errCh); !errors.Is(err, context.Canceled) {
		t.Errorf("MoveToJointPositions returned %v, want context.Canceled", err)
	}
	cmds := f.commands()
	if cmds[len(cmds)-1] != cmdIDStop {
		t.Errorf("last command was %d, want STOP; got %v", cmds[len(cmds)-1], cmds)
	}
}

func TestStopCancelsGoToInputsBeforeNextWaypoint(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, movingFrame, Config{})

	errCh := make(chan error, 1)
	go func() { errCh <- s.GoToInputs(context.Background(), zeroJoints(), zeroJoints()) }()
	f.waitFor(t, cmdIDMoveJ)

	if err := s.Stop(context.Background(), nil); err != nil {
		t.Fatalf("Stop: %v", err)
	}
	if err := waitErr(t, errCh); !errors.Is(err, context.Canceled) {
		t.Errorf("GoToInputs returned %v, want context.Canceled", err)
	}
	if n := f.count(cmdIDMoveJ); n != 1 {
		t.Errorf("sent %d MoveJs, want 1 (the second waypoint must not go out); got %v", n, f.commands())
	}
	if f.count(cmdIDStop) == 0 {
		t.Errorf("no STOP sent; got %v", f.commands())
	}
}

func TestBlendedMoveFailureSendsStop(t *testing.T) {
	f := newFakeCommandPort(t)
	f.failFwdKinFrom = 2
	s := newMotionTestArm(t, f, movingFrame, Config{BlendTimeMs: 100})

	if err := s.MoveThroughJointPositions(context.Background(), [][]referenceframe.Input{zeroJoints(), zeroJoints()}, nil, nil); err == nil {
		t.Fatal("expected the second waypoint to fail")
	}
	cmds := f.commands()
	want := []int{cmdIDGetForwardKin, cmdIDMoveJ, cmdIDGetForwardKin, cmdIDStop}
	if !slices.Equal(cmds, want) {
		t.Errorf("got commands %v, want %v", cmds, want)
	}
}

func TestSendRefusesCancelledContext(t *testing.T) {
	f := newFakeCommandPort(t)
	c := newCommandClient("127.0.0.1", f.port(), logging.NewTestLogger(t))
	t.Cleanup(func() { _ = c.close() })

	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if _, err := c.send(ctx, cmdIDMoveJ, "MoveJ()"); !errors.Is(err, context.Canceled) {
		t.Errorf("send returned %v, want context.Canceled", err)
	}
	if err := c.stop(ctx); err != nil {
		t.Errorf("stop with a cancelled context: %v", err)
	}
	if got := f.commands(); !slices.Equal(got, []int{cmdIDStop}) {
		t.Errorf("controller received %v, want only STOP", got)
	}
}

func TestMoveToPositionPlanRunsThroughGoToInputs(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})
	s.motion = &fakeMotion{move: func(ctx context.Context, req motion.MoveReq) (bool, error) {
		return true, s.GoToInputs(context.Background(), zeroJoints())
	}}

	errCh := make(chan error, 1)
	go func() { errCh <- s.MoveToPosition(context.Background(), spatialmath.NewZeroPose(), nil) }()
	if err := waitErr(t, errCh); err != nil {
		t.Errorf("MoveToPosition: %v", err)
	}
}

func TestStopCancelsMoveToPosition(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})
	planning := make(chan struct{})
	s.motion = &fakeMotion{move: func(ctx context.Context, req motion.MoveReq) (bool, error) {
		close(planning)
		<-ctx.Done()
		return false, ctx.Err()
	}}

	errCh := make(chan error, 1)
	go func() { errCh <- s.MoveToPosition(context.Background(), spatialmath.NewZeroPose(), nil) }()
	<-planning

	if err := s.Stop(context.Background(), nil); err != nil {
		t.Fatalf("Stop: %v", err)
	}
	if err := waitErr(t, errCh); !errors.Is(err, context.Canceled) {
		t.Errorf("MoveToPosition returned %v, want context.Canceled", err)
	}
	if got := f.commands(); !slices.Equal(got, []int{cmdIDStop}) {
		t.Errorf("controller received %v, want only STOP", got)
	}
}

func TestStopWaitsForTheArmToComeToRest(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})
	start := time.Now()
	const travel = 200 * time.Millisecond
	tickStatus(t, s.status, func(fr *statusFrame) {
		fr.JointDegs[0] = min(2, 2*float64(time.Since(start))/float64(travel))
	})

	if err := s.Stop(context.Background(), nil); err != nil {
		t.Fatalf("Stop: %v", err)
	}
	if elapsed := time.Since(start); elapsed < travel {
		t.Errorf("Stop returned after %v, while the joints were still moving", elapsed)
	}
}

func TestNewMoveWaitsForStop(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})
	start := time.Now()
	const travel = 200 * time.Millisecond
	tickStatus(t, s.status, func(fr *statusFrame) {
		fr.JointDegs[0] = min(2, 2*float64(time.Since(start))/float64(travel))
	})

	stopErr := make(chan error, 1)
	go func() { stopErr <- s.Stop(context.Background(), nil) }()
	f.waitFor(t, cmdIDStop)
	moveErr := make(chan error, 1)
	target := degToInputs([]float64{2, 0, 0, 0, 0, 0})
	go func() { moveErr <- s.MoveToJointPositions(context.Background(), target, nil) }()

	if err := waitErr(t, stopErr); err != nil {
		t.Fatalf("Stop: %v", err)
	}
	if err := waitErr(t, moveErr); err != nil {
		t.Fatalf("the move sent during Stop: %v", err)
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	first := slices.Index(f.received, cmdIDGetForwardKin)
	if first < 0 || f.at[first].Sub(start) < travel {
		t.Errorf("the new move started before the arm came to rest; commands %v", f.received)
	}
	if slices.Contains(f.received[first:], cmdIDStop) {
		t.Errorf("STOP went out after the new move started: %v", f.received)
	}
}

func TestIsMovingWhileAMoveIsInProgress(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})

	_, done := s.opMgr.New(context.Background())
	if moving, err := s.IsMoving(context.Background()); err != nil || !moving {
		t.Errorf("IsMoving = %v, %v with a move in progress; want true", moving, err)
	}
	done()
	if moving, err := s.IsMoving(context.Background()); err != nil || moving {
		t.Errorf("IsMoving = %v, %v with no move and motion done; want false", moving, err)
	}
}

func TestStopBehindASlowCommandFailsInsteadOfWaiting(t *testing.T) {
	f := newFakeCommandPort(t)
	f.slow = map[int]time.Duration{cmdIDMoveL: 4 * time.Second}
	s := newMotionTestArm(t, f, atRestFrame, Config{})
	moveLDone := make(chan struct{})
	go func() {
		defer close(moveLDone)
		_, _ = s.cmd.send(context.Background(), cmdIDMoveL, "MoveL()")
	}()
	t.Cleanup(func() {
		_ = s.cmd.close()
		<-moveLDone
	})
	f.waitFor(t, cmdIDMoveL)

	start := time.Now()
	if err := s.Stop(context.Background(), nil); err == nil {
		t.Error("Stop reported success though STOP never reached the controller")
	}
	if elapsed := time.Since(start); elapsed > stopAckTimeout+time.Second {
		t.Errorf("Stop took %v; it should give up after about %v", elapsed, stopAckTimeout)
	}
	if n := f.count(cmdIDStop); n != 0 {
		t.Errorf("STOP reached the controller %d times; it should have been dropped", n)
	}
}

func TestStopWaitsThroughAStatusReadError(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})
	s.status.mu.Lock()
	saved := s.status.frame
	s.status.have, s.status.err = false, errors.New("status read timeout")
	s.status.mu.Unlock()
	time.AfterFunc(100*time.Millisecond, func() {
		s.status.mu.Lock()
		s.status.frame, s.status.have, s.status.err = saved, true, nil
		s.status.mu.Unlock()
	})

	if err := s.Stop(context.Background(), nil); err != nil {
		t.Errorf("Stop: %v; a brief status feed error should not end the wait", err)
	}
}

func TestStopSucceedsWhenRestIsNotConfirmed(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})
	start := time.Now()
	tickStatus(t, s.status, func(fr *statusFrame) {
		fr.JointDegs[0] = float64(time.Since(start)) / float64(100*time.Millisecond)
	})

	ctx, cancel := context.WithTimeout(context.Background(), 300*time.Millisecond)
	defer cancel()
	if err := s.waitForRest(ctx); !errors.Is(err, errRestNotConfirmed) || !errors.Is(err, context.DeadlineExceeded) {
		t.Errorf("waitForRest returned %v, want errRestNotConfirmed caused by the caller's deadline", err)
	}

	ctx, cancel = context.WithTimeout(context.Background(), 300*time.Millisecond)
	defer cancel()
	if err := s.Stop(ctx, nil); err != nil {
		t.Errorf("Stop returned %v; STOP was acknowledged, so only a warning is due", err)
	}
	if n := f.count(cmdIDStop); n != 1 {
		t.Errorf("STOP reached the controller %d times, want 1", n)
	}
}

func TestStopRetriesOnADroppedConnection(t *testing.T) {
	f := newFakeCommandPort(t)
	f.dropOnce = cmdIDStop
	s := newMotionTestArm(t, f, atRestFrame, Config{})

	if err := s.Stop(context.Background(), nil); err != nil {
		t.Fatalf("Stop: %v", err)
	}
	if n := f.count(cmdIDStop); n != 2 {
		t.Errorf("STOP reached the controller %d times, want 2 (the dropped one and the retry)", n)
	}
}
