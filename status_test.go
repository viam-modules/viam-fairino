package viamfairino

import (
	"bytes"
	"context"
	"encoding/binary"
	"errors"
	"math"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
)

// Test values chosen to avoid 0x5A ('Z') byte pairs appearing in the payload
// except where a test plants them deliberately.
var (
	testJoints  = [6]float64{1.5, -30.25, 90, 0.001, -179.99, 45.5}
	testTCP     = [6]float64{300.5, -120.75, 450, 12.5, -90, 179}
	testTorques = [6]float64{0.5, 12.25, -3.75, 0, 1.125, -0.25}
)

// buildTestPayload constructs a minStatusPayloadLen-byte payload with known
// values in every Phase 1 field.
func buildTestPayload() []byte {
	p := make([]byte, minStatusPayloadLen)
	p[offProgramState] = programStateRun
	p[offErrorCode] = 3 // collision failure
	p[offRobotMode] = robotModeManual
	for i := 0; i < 6; i++ {
		binary.LittleEndian.PutUint64(p[offJointPos+i*8:], math.Float64bits(testJoints[i]))
		binary.LittleEndian.PutUint64(p[offTCPPose+i*8:], math.Float64bits(testTCP[i]))
		binary.LittleEndian.PutUint64(p[offJointTorque+i*8:], math.Float64bits(testTorques[i]))
	}
	binary.LittleEndian.PutUint32(p[offToolNum:], 2)
	copy(p[offProgramName:], "test_prog")
	p[offProgTotalLine] = 12
	p[offProgCurLine] = 7
	p[offEmergencyStop] = 1
	binary.LittleEndian.PutUint32(p[offMotionDone:], 1)
	return p
}

// buildFrame wraps a payload in the 8083 wire framing with a valid checksum.
func buildFrame(cnt byte, payload []byte) []byte {
	f := []byte{frameHeaderByte, frameHeaderByte, cnt, 0, 0}
	binary.LittleEndian.PutUint16(f[frameHeaderLen+frameCountLen:], uint16(len(payload)))
	f = append(f, payload...)
	var cs [2]byte
	binary.LittleEndian.PutUint16(cs[:], sumChecksum(f))
	return append(f, cs[:]...)
}

func TestParseStatusGolden(t *testing.T) {
	fr, ok := parseStatus(buildTestPayload(), 42)
	if !ok {
		t.Fatal("parseStatus rejected a valid payload")
	}
	if fr.ProgramState != programStateRun || fr.ErrorCode != 3 || fr.RobotMode != robotModeManual {
		t.Errorf("state fields wrong: %+v", fr)
	}
	for i := 0; i < 6; i++ {
		if fr.JointDegs[i] != testJoints[i] {
			t.Errorf("joint %d: got %v want %v", i, fr.JointDegs[i], testJoints[i])
		}
		if fr.TCPPose[i] != testTCP[i] {
			t.Errorf("tcp %d: got %v want %v", i, fr.TCPPose[i], testTCP[i])
		}
		if fr.JointTorques[i] != testTorques[i] {
			t.Errorf("torque %d: got %v want %v", i, fr.JointTorques[i], testTorques[i])
		}
	}
	if fr.ToolNum != 2 {
		t.Errorf("toolNum: got %d want 2", fr.ToolNum)
	}
	if fr.ProgramName != "test_prog" {
		t.Errorf("program name: got %q", fr.ProgramName)
	}
	if !fr.EmergencyStop {
		t.Error("EmergencyStop not set")
	}
	if !fr.MotionDone {
		t.Error("MotionDone not set")
	}
	if fr.FrameCount != 42 {
		t.Errorf("frame count: got %d want 42", fr.FrameCount)
	}
	if len(fr.RawPayload) != minStatusPayloadLen {
		t.Errorf("raw payload len: got %d want %d", len(fr.RawPayload), minStatusPayloadLen)
	}
}

func TestParseStatusShortPayload(t *testing.T) {
	if _, ok := parseStatus(make([]byte, minStatusPayloadLen-1), 0); ok {
		t.Error("parseStatus accepted a short payload")
	}
}

func TestExtractFramesSingle(t *testing.T) {
	frame := buildFrame(1, buildTestPayload())
	frames, rest := extractFrames(frame)
	if len(frames) != 1 {
		t.Fatalf("got %d frames, want 1", len(frames))
	}
	if !bytes.Equal(frames[0], frame) {
		t.Error("extracted frame differs from input")
	}
	if len(rest) != 0 {
		t.Errorf("rest not empty: %d bytes", len(rest))
	}
}

func TestExtractFramesGarbagePrefix(t *testing.T) {
	frame := buildFrame(1, buildTestPayload())
	buf := append([]byte{0x01, 0x02, 0xFF, 0x00, 0x5A, 0x00}, frame...)
	frames, rest := extractFrames(buf)
	if len(frames) != 1 || !bytes.Equal(frames[0], frame) {
		t.Fatalf("frame not recovered after garbage prefix: %d frames", len(frames))
	}
	if len(rest) != 0 {
		t.Errorf("rest not empty: %d bytes", len(rest))
	}
}

// TestExtractFramesSplitReads feeds two frames one byte at a time (the worst
// case: every boundary, including mid-header, is split across reads).
func TestExtractFramesSplitReads(t *testing.T) {
	f1 := buildFrame(1, buildTestPayload())
	f2 := buildFrame(2, buildTestPayload())
	stream := append(append([]byte(nil), f1...), f2...)

	var buffer []byte
	var got [][]byte
	for _, b := range stream {
		buffer = append(buffer, b)
		frames, rest := extractFrames(buffer)
		got = append(got, frames...)
		buffer = rest
	}
	if len(got) != 2 {
		t.Fatalf("got %d frames, want 2", len(got))
	}
	if !bytes.Equal(got[0], f1) || !bytes.Equal(got[1], f2) {
		t.Error("reassembled frames differ from originals")
	}
}

// TestExtractFramesChecksumReject corrupts frame 1 and checks that frame 2 in
// the same buffer still parses — i.e. resync recovers past the bad frame.
func TestExtractFramesChecksumReject(t *testing.T) {
	f1 := buildFrame(1, buildTestPayload())
	f1[frameOverheadLen+10] ^= 0xFF // corrupt a payload byte; checksum now fails
	f2 := buildFrame(2, buildTestPayload())
	frames, rest := extractFrames(append(f1, f2...))
	if len(frames) != 1 {
		t.Fatalf("got %d frames, want 1", len(frames))
	}
	if !bytes.Equal(frames[0], f2) {
		t.Error("recovered frame is not the valid one")
	}
	if len(rest) != 0 {
		t.Errorf("rest not empty: %d bytes", len(rest))
	}
}

// TestExtractFramesHeaderInPayload starts the buffer mid-frame (the real
// header was missed) with 0x5A5A bytes inside the visible payload, followed
// by a valid frame. The false header's garbage LEN must not derail resync.
func TestExtractFramesHeaderInPayload(t *testing.T) {
	payload := buildTestPayload()
	copy(payload[offProgramName:], "ZZfake") // 'Z' == 0x5A → false header
	f1 := buildFrame(1, payload)
	f2 := buildFrame(2, buildTestPayload())
	// Drop f1's real header so only its payload's ZZ looks like a frame start.
	buf := append(f1[frameHeaderLen:], f2...)
	frames, _ := extractFrames(buf)
	if len(frames) != 1 {
		t.Fatalf("got %d frames, want 1", len(frames))
	}
	if !bytes.Equal(frames[0], f2) {
		t.Error("recovered frame is not the valid one")
	}
}

// TestExtractFramesCorruptLen plants a false header whose LEN field exceeds
// maxFrameDataLen; extraction must skip it rather than wait forever for
// bytes that will never come.
func TestExtractFramesCorruptLen(t *testing.T) {
	bad := []byte{frameHeaderByte, frameHeaderByte, 0x00, 0xFF, 0xFF} // LEN = 65535
	f := buildFrame(1, buildTestPayload())
	frames, rest := extractFrames(append(bad, f...))
	if len(frames) != 1 || !bytes.Equal(frames[0], f) {
		t.Fatalf("frame not recovered past corrupt LEN: %d frames", len(frames))
	}
	if len(rest) != 0 {
		t.Errorf("rest not empty: %d bytes", len(rest))
	}
}

// TestExtractFramesTrailingHeaderByte checks that a lone 0x5A at the end of
// the buffer is kept — it may be the first half of a header split across
// reads.
func TestExtractFramesTrailingHeaderByte(t *testing.T) {
	f := buildFrame(1, buildTestPayload())
	frames, rest := extractFrames(append(f, frameHeaderByte))
	if len(frames) != 1 {
		t.Fatalf("got %d frames, want 1", len(frames))
	}
	if len(rest) != 1 || rest[0] != frameHeaderByte {
		t.Errorf("trailing 0x5A not kept: rest=%v", rest)
	}
}

func TestSumChecksumMatchesTestPy(t *testing.T) {
	// Same definition as test.py's calc_checksum: sum of bytes & 0xFFFF.
	data := []byte{0x5A, 0x5A, 0xFF, 0x01, 0x02}
	want := uint16((0x5A + 0x5A + 0xFF + 0x01 + 0x02) & 0xFFFF)
	if got := sumChecksum(data); got != want {
		t.Errorf("checksum: got %#x want %#x", got, want)
	}
}

// bad_joints needs a position at startup; a reader still reconnecting after a
// dial error must be given the chance to deliver one.
func TestWaitForFrameThroughErrors(t *testing.T) {
	sc := newStatusClient("127.0.0.1", statusPort, logging.NewTestLogger(t))
	sc.recordError(errors.New("status port dial: connection refused"))
	go func() {
		time.Sleep(100 * time.Millisecond)
		setTestFrame(sc, restingFrame([6]float64{1, 2, 3, 4, 5, 6}))
	}()
	if _, err := sc.waitForFrame(context.Background()); err == nil {
		t.Error("waitForFrame should report the dial error at once")
	}
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	fr, err := sc.waitForFrameThroughErrors(ctx)
	if err != nil || fr.JointDegs[0] != 1 {
		t.Errorf("got %v, %v", fr.JointDegs, err)
	}
}
