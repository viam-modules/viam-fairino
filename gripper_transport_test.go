package viamfairino

import (
	"context"
	"encoding/binary"
	"fmt"
	"net"
	"strconv"
	"testing"
	"time"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/resource"
)

// buildGripperPayload extends buildTestPayload to a full-length frame with
// known values in every gripper field (Table 2-2 rows 41, 56-63).
func buildGripperPayload(motionDone uint8, position uint8) []byte {
	p := make([]byte, minGripperPayloadLen)
	copy(p, buildTestPayload())
	p[offGripperMotionDone] = motionDone
	p[offGripperFaultID] = 1
	binary.LittleEndian.PutUint16(p[offGripperFault:], 0)
	binary.LittleEndian.PutUint16(p[offGripperActive:], 1)
	p[offGripperPosition] = position
	p[offGripperSpeed] = byte(0xEC) // int8(-20)
	p[offGripperCurrent] = byte(int8(35))
	binary.LittleEndian.PutUint32(p[offGripperTemp:], uint32(41))
	binary.LittleEndian.PutUint32(p[offGripperVoltage:], uint32(24))
	return p
}

func TestParseStatusGripperFields(t *testing.T) {
	fr, ok := parseStatus(buildGripperPayload(gripperMotionClamped, 63), 1)
	if !ok {
		t.Fatal("parseStatus rejected a valid payload")
	}
	if !fr.GripperValid {
		t.Fatal("GripperValid false on a full-length payload")
	}
	if fr.GripperMotionDone != gripperMotionClamped {
		t.Errorf("motion_done: got %d want %d", fr.GripperMotionDone, gripperMotionClamped)
	}
	if fr.GripperPosition != 63 {
		t.Errorf("position: got %d want 63", fr.GripperPosition)
	}
	if fr.GripperActive != 1 || fr.GripperFault != 0 || fr.GripperFaultID != 1 {
		t.Errorf("active/fault fields wrong: %+v", fr)
	}
	if fr.GripperSpeed != -20 || fr.GripperCurrent != 35 {
		t.Errorf("signed fields wrong: speed=%d current=%d", fr.GripperSpeed, fr.GripperCurrent)
	}
	if fr.GripperTemp != 41 || fr.GripperVoltage != 24 {
		t.Errorf("temp/voltage wrong: %d %d", fr.GripperTemp, fr.GripperVoltage)
	}

	// The arm's own fields must still decode from the longer payload — the
	// gripper offsets sit past them, so a bad offset would corrupt neither, but
	// a bad *length* guard would drop them.
	if !fr.MotionDone || !fr.EmergencyStop {
		t.Error("arm fields lost when parsing a full-length payload")
	}
}

// A frame that stops at the arm's field set must parse, with the gripper fields
// explicitly marked invalid rather than silently reading as zero.
func TestParseStatusGripperFieldsAbsent(t *testing.T) {
	fr, ok := parseStatus(buildTestPayload(), 1)
	if !ok {
		t.Fatal("parseStatus rejected a valid arm-only payload")
	}
	if fr.GripperValid {
		t.Error("GripperValid true on a payload too short to contain the gripper fields")
	}
}

// newTestGripper builds a gripper whose status client is pre-seeded with fr and
// never started, so state-reading paths can be tested without a controller.
func newTestGripper(t *testing.T, fr statusFrame) *fairinoGripper {
	t.Helper()
	model, err := makeGripperModel("test", gripperStrokeMM)
	if err != nil {
		t.Fatal(err)
	}
	sc := newStatusClient("test", statusPort, logging.NewTestLogger(t))
	sc.frame = fr
	sc.have = true
	return &fairinoGripper{
		name:     gripper.Named("test"),
		logger:   logging.NewTestLogger(t),
		status:   sc,
		opMgr:    operation.NewSingleOperationManager(),
		model:    model,
		strokeMM: gripperStrokeMM,
	}
}

func frameWith(t *testing.T, motionDone, position uint8) statusFrame {
	t.Helper()
	fr, ok := parseStatus(buildGripperPayload(motionDone, position), 1)
	if !ok {
		t.Fatal("failed to build test frame")
	}
	return fr
}

// The DH encoding distinguishes "clamped" (2) from "clamped then dropped" (3).
// Only 2 is holding — 3 means the part is gone.
func TestIsHoldingSomethingDHEncoding(t *testing.T) {
	for _, tc := range []struct {
		motionDone  uint8
		wantHolding bool
		wantDropped bool
	}{
		{gripperMotionMoving, false, false},
		{gripperMotionStopped, false, false},
		{gripperMotionClamped, true, false},
		{gripperMotionDropped, false, true},
	} {
		g := newTestGripper(t, frameWith(t, tc.motionDone, 50))
		st, err := g.IsHoldingSomething(context.Background(), nil)
		if err != nil {
			t.Fatalf("motion_done=%d: %v", tc.motionDone, err)
		}
		if st.IsHoldingSomething != tc.wantHolding {
			t.Errorf("motion_done=%d: holding got %v want %v", tc.motionDone, st.IsHoldingSomething, tc.wantHolding)
		}
		if dropped, _ := st.Meta["dropped"].(bool); dropped != tc.wantDropped {
			t.Errorf("motion_done=%d: dropped got %v want %v", tc.motionDone, dropped, tc.wantDropped)
		}
	}
}

// CurrentInputs must track the live position from the status feed, so the 3D
// scene shows the fingers where they actually are.
func TestCurrentInputsFromStatusFrame(t *testing.T) {
	closed := newTestGripper(t, frameWith(t, gripperMotionStopped, 0))
	open := newTestGripper(t, frameWith(t, gripperMotionStopped, 100))

	ci, err := closed.CurrentInputs(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	oi, err := open.CurrentInputs(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if ci[0] != 0 {
		t.Errorf("closed: got %v want 0", ci[0])
	}
	if oi[0] <= ci[0] {
		t.Errorf("open (%v) should exceed closed (%v)", oi[0], ci[0])
	}
}

// A frame too short to carry the gripper fields must be an explicit error, not
// a confident report of position 0 / not-holding.
func TestGripperStateRejectsShortFrame(t *testing.T) {
	fr, ok := parseStatus(buildTestPayload(), 1)
	if !ok {
		t.Fatal("failed to build test frame")
	}
	g := newTestGripper(t, fr)
	if _, err := g.IsHoldingSomething(context.Background(), nil); err == nil {
		t.Error("expected an error when the frame lacks the gripper fields")
	}
	if _, err := g.CurrentInputs(context.Background()); err == nil {
		t.Error("expected an error when the frame lacks the gripper fields")
	}
}

// formatMoveGripper must reproduce the manual's own example (Table 7-1-4)
// exactly — this is the wire format, so a stray space or float rendering breaks
// the controller's parser.
func TestFormatMoveGripperMatchesManual(t *testing.T) {
	got := formatMoveGripper(moveGripperParams{
		Index: 1, Pos: 51, Vel: 68, Force: 64, MaxTime: 30000,
		Block: 0, Type: 0, RotNum: 0, RotSpeed: 0, RotTorq: 0,
	})
	want := "MoveGripper(1,51,68,64,30000,0,0,0,0,0)"
	if got != want {
		t.Errorf("got %q want %q", got, want)
	}
}

func TestParseIntList(t *testing.T) {
	// GetRobotErrorCode's example separator is a comma; GetGripperConfig's is
	// whitespace. Both have to work.
	for _, tc := range []struct {
		in   string
		want []int
	}{
		{"0,0", []int{0, 0}},
		{"30 00 00 00", []int{30, 0, 0, 0}},
		{" 4, 0 ,0 ", []int{4, 0, 0}},
	} {
		got, err := parseIntList(tc.in)
		if err != nil {
			t.Fatalf("%q: %v", tc.in, err)
		}
		if fmt.Sprint(got) != fmt.Sprint(tc.want) {
			t.Errorf("%q: got %v want %v", tc.in, got, tc.want)
		}
	}
	if _, err := parseIntList("MoveGripper failed"); err == nil {
		t.Error("expected an error on non-numeric data")
	}
}

// errcode 73 must stay detectable through a wrap, from both response shapes:
// a plain errcode and an embedded Lua script error.
func TestControllerErrcodeDetection(t *testing.T) {
	plain := fmt.Errorf("gripper move: %w", parseErrcode("73"))
	if !isControllerErrcode(plain, errcodeGripperMove) {
		t.Error("plain errcode 73 not detected")
	}
	script := fmt.Errorf("gripper move: %w", scriptErrorFromData(`[string "MoveGripper(1,0,50,50,30000,0,0,0,0,0)"]:1: robot errcode:73`))
	if !isControllerErrcode(script, errcodeGripperMove) {
		t.Error("script-error errcode 73 not detected")
	}
	if isControllerErrcode(plain, 14) {
		t.Error("matched the wrong code")
	}
	if parseErrcode("0") != nil || parseErrcode("1") != nil {
		t.Error("0 and 1 are success codes")
	}
}

var _ resource.Resource = (*fairinoGripper)(nil)

// The controller admits one status-port client at a time, so resources on the
// same host must share a single reader: two acquires yield the same client, and
// only the last release stops it.
func TestSharedStatusClientRefcount(t *testing.T) {
	logger := logging.NewTestLogger(t)
	host := "203.0.113.1" // TEST-NET-3: never routable, so the reader just retries

	a := acquireStatusClient(context.Background(), host, statusPort, logger)
	b := acquireStatusClient(context.Background(), host, statusPort, logger)
	if a != b {
		t.Fatal("two acquires for the same host returned different clients")
	}

	releaseStatusClient(host, statusPort)
	sharedStatusMu.Lock()
	_, stillThere := sharedStatusClients[net.JoinHostPort(host, strconv.Itoa(statusPort))]
	sharedStatusMu.Unlock()
	if !stillThere {
		t.Error("client dropped while a reference was still held")
	}

	releaseStatusClient(host, statusPort)
	sharedStatusMu.Lock()
	_, stillThere = sharedStatusClients[net.JoinHostPort(host, strconv.Itoa(statusPort))]
	sharedStatusMu.Unlock()
	if stillThere {
		t.Error("client not released after the last reference went away")
	}

	// A release with no outstanding acquire must be a no-op, not a panic.
	releaseStatusClient(host, statusPort)
}

func TestSharedCommandClientRefcount(t *testing.T) {
	logger := logging.NewTestLogger(t)
	host := "203.0.113.2" // TEST-NET-3: never routable; the client connects lazily anyway

	a := acquireCommandClient(host, commandPort, logger)
	b := acquireCommandClient(host, commandPort, logger)
	if a != b {
		t.Fatal("two acquires for the same host returned different command clients")
	}

	key := net.JoinHostPort(host, strconv.Itoa(commandPort))

	releaseCommandClient(host, commandPort)
	sharedCommandMu.Lock()
	_, stillThere := sharedCommandClients[key]
	sharedCommandMu.Unlock()
	if !stillThere {
		t.Error("command client dropped while a reference was still held")
	}

	releaseCommandClient(host, commandPort)
	sharedCommandMu.Lock()
	_, stillThere = sharedCommandClients[key]
	sharedCommandMu.Unlock()
	if stillThere {
		t.Error("command client not released after the last reference went away")
	}

	// A release with no outstanding acquire must be a no-op, not a panic.
	releaseCommandClient(host, commandPort)
}

// newCommandGripper is newTestGripper wired to a fake command port.
func newCommandGripper(t *testing.T, fc *fakeController, fr statusFrame) *fairinoGripper {
	t.Helper()
	g := newTestGripper(t, fr)
	g.cmd = newCommandClient("127.0.0.1", fc.port(), logging.NewTestLogger(t))
	t.Cleanup(func() { _ = g.cmd.close() })
	g.speed, g.force, g.maxTime, g.grabPos = 50, 50, 30000, 0
	return g
}

// MoveGripper goes out non-blocking, so the shared command connection is free
// for the arm's STOP while the fingers travel, and Grab follows the motion on
// the status feed: it returns only once a post-move frame shows them stopped.
func TestGrabIsNonBlockingAndWaitsForTheStatusFeed(t *testing.T) {
	fc := newFakeController(t)
	g := newCommandGripper(t, fc, frameWith(t, gripperMotionStopped, 100))

	type result struct {
		holding bool
		err     error
	}
	done := make(chan result, 1)
	go func() {
		holding, err := g.Grab(context.Background(), nil)
		done <- result{holding, err}
	}()
	waitFor(t, "MoveGripper", func() bool { return fc.count("MoveGripper") == 1 })
	fc.mu.Lock()
	sent := fc.commands[0]
	fc.mu.Unlock()
	if sent != "MoveGripper(1,0,50,50,30000,1,0,0,0,0)" {
		t.Errorf("sent %q, want a non-blocking (block=1) MoveGripper", sent)
	}

	// The connection is free while the fingers move.
	if err := g.cmd.stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	setTestFrame(g.status, frameWith(t, gripperMotionMoving, 50))
	time.Sleep(100 * time.Millisecond)
	select {
	case r := <-done:
		t.Fatalf("Grab returned %+v while the fingers were still moving", r)
	default:
	}
	setTestFrame(g.status, frameWith(t, gripperMotionClamped, 20))
	select {
	case r := <-done:
		if r.err != nil || !r.holding {
			t.Errorf("got %+v, want holding", r)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("Grab didn't return once the fingers stopped")
	}
}

// A gripper already at the target never reports moving; the move counts as
// done once gripperStartGrace passes with fresh frames saying so.
func TestGripperMoveWithNoMotionFinishesAfterGrace(t *testing.T) {
	fc := newFakeController(t)
	g := newCommandGripper(t, fc, frameWith(t, gripperMotionStopped, 100))
	stop := make(chan struct{})
	defer close(stop)
	go func() { // a live feed: fresh frames, fingers never moving
		for {
			select {
			case <-stop:
				return
			case <-time.After(20 * time.Millisecond):
				setTestFrame(g.status, frameWith(t, gripperMotionStopped, 100))
			}
		}
	}()
	start := time.Now()
	if err := g.Open(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	if elapsed := time.Since(start); elapsed < gripperStartGrace {
		t.Errorf("Open returned after %v, before the %v grace", elapsed, gripperStartGrace)
	}
}

// A status feed that freezes mid-move gives no way to see the fingers stop,
// so the move fails rather than waiting out max_time.
func TestGripperMoveFailsWhenTheFeedFreezes(t *testing.T) {
	fc := newFakeController(t)
	g := newCommandGripper(t, fc, frameWith(t, gripperMotionStopped, 100))
	setTestFrame(g.status, frameWith(t, gripperMotionMoving, 50)) // then nothing more
	start := time.Now()
	if err := g.Open(context.Background(), nil); err == nil {
		t.Fatal("expected an error")
	}
	if elapsed := time.Since(start); elapsed > 3*statusFeedLostTimeout {
		t.Errorf("took %v to notice", elapsed)
	}
}
