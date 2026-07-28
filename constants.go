// Wire-format definitions for the Fairino controller's status feedback port
// (TCP 8083), per "Robot 8083 Port Status Feedback User Manual" V3.9.7.
//
// The controller pushes one frame per period (default 100 ms,
// configurable 8–100 ms). Frame layout (manual Table 2-1, little-endian):
//
//	| Frame Header | Frame Count | Data Length | Data Content | Sum Check |
//	|  0x5A5A u16  |   CNT u8    |   LEN u16   |  LEN bytes   | CHKSUM u16 |

package viamfairino

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"math"
	"time"
)

const (
	defaultHost = "192.168.58.2"
	statusPort  = 8083
	commandPort = 8080
)

// Frame framing constants (manual Table 2-1).
const (
	frameHeaderByte  = 0x5A
	frameHeaderLen   = 2 // 0x5A5A
	frameCountLen    = 1 // rolling counter 0–255
	frameDataLenLen  = 2 // payload length, uint16
	frameOverheadLen = frameHeaderLen + frameCountLen + frameDataLenLen
	frameChecksumLen = 2

	// manual's full payload sums to 649 bytes; 4096 leaves generous headroom
	maxFrameDataLen = 4096
)

// Payload offsets in bytes (manual Table 2-2, packed little-endian)
const (
	offProgramState  = 0   // uint8
	offErrorCode     = 1   // uint8, fault table below
	offRobotMode     = 2   // uint8
	offJointPos      = 3   // float64[6], degrees
	offTCPPose       = 51  // float64[6], x,y,z mm + a,b,c deg
	offToolNum       = 99  // int32, active tool frame index
	offJointTorque   = 103 // float64[6], N·m
	offProgramName   = 151 // char[20], NUL-padded
	offProgTotalLine = 171 // uint8
	offProgCurLine   = 172 // uint8
	offClDgtOutputH  = 173 // uint8, control-box digital outputs 15–8
	offClDgtOutputL  = 174 // uint8, control-box digital outputs 7–0
	offTlDgtOutputL  = 175 // uint8, tool digital outputs (bits 0–1 only)
	offClDgtInputH   = 176 // uint8, control-box digital inputs 15–8
	offClDgtInputL   = 177 // uint8, control-box digital inputs 7–0
	offTlDgtInputL   = 178 // uint8, tool digital inputs (bits 0–1 only)
	offFTData        = 179 // float64[6], F/T sensor Fx,Fy,Fz N + Tx,Ty,Tz N·m
	offFTActStatus   = 227 // uint8, 0 reset / 1 activated
	offEmergencyStop = 228 // uint8, 1 = e-stopped
	offMotionDone    = 229 // int32, 1 = in place, 0 = not in place

	// minStatusPayloadLen is the end of the field set above; shorter payloads
	// are rejected.
	minStatusPayloadLen = offMotionDone + 4
)

// program_state values (Table 2-2 row 1).
const (
	programStateStop      = 1
	programStateRun       = 2
	programStateSuspended = 3
	programStateDrag      = 4
)

// robot_mode values (Table 2-2 row 3).
const (
	robotModeAuto   = 0
	robotModeManual = 1
	robotModeDrag   = 2
)

var programStateNames = map[uint8]string{
	programStateStop:      "stopped",
	programStateRun:       "running",
	programStateSuspended: "suspended",
	programStateDrag:      "drag",
}

var robotModeNames = map[uint8]string{
	robotModeAuto:   "automatic",
	robotModeManual: "manual",
	robotModeDrag:   "drag",
}

var faultCodeNames = map[uint8]string{
	0:  "no fault",
	1:  "drive failure",
	2:  "soft limit exceeded",
	3:  "collision failure",
	4:  "singular pose",
	5:  "slave error",
	6:  "command point incorrect",
	7:  "IO error",
	8:  "axle device error",
	9:  "file error",
	10: "parameter incorrect",
	11: "extension shaft exceeds soft limit",
	12: "joint configuration warning",
}

func programStateString(v uint8) string {
	if s, ok := programStateNames[v]; ok {
		return s
	}
	return fmt.Sprintf("unknown program state %d", v)
}

func robotModeString(v uint8) string {
	if s, ok := robotModeNames[v]; ok {
		return s
	}
	return fmt.Sprintf("unknown robot mode %d", v)
}

func faultCodeString(v uint8) string {
	if s, ok := faultCodeNames[v]; ok {
		return s
	}
	return fmt.Sprintf("unknown fault code %d", v)
}

// statusFrame is a decoded snapshot of one status payload. Values keep the
// controller's native units (degrees, millimeters, N·m); module.go converts
// to radians/SI at the arm.Arm boundary.
type statusFrame struct {
	ProgramState  uint8
	ErrorCode     uint8
	RobotMode     uint8
	JointDegs     [6]float64 // jt_cur_pos, degrees
	TCPPose       [6]float64 // tl_cur_pos, x,y,z mm + a,b,c deg
	ToolNum       int32
	JointTorques  [6]float64 // jt_cur_tor, N·m
	ProgramName   string
	EmergencyStop bool
	MotionDone    bool // robot_motion_done == 1 ("in place")

	// RawPayload is a copy of the full payload so the DoCommand status dump
	// can decode fields the hot path doesn't, without re-reading the wire.
	RawPayload []byte

	FrameCount uint8
	ReceivedAt time.Time
}

// parseStatus decodes the Phase 1 field set from a verified frame payload.
// Returns false if the payload is too short to contain those fields.
func parseStatus(payload []byte, frameCount uint8) (statusFrame, bool) {
	if len(payload) < minStatusPayloadLen {
		return statusFrame{}, false
	}
	fr := statusFrame{
		ProgramState:  payload[offProgramState],
		ErrorCode:     payload[offErrorCode],
		RobotMode:     payload[offRobotMode],
		ToolNum:       readI32(payload, offToolNum),
		ProgramName:   readCString(payload[offProgramName : offProgramName+20]),
		EmergencyStop: payload[offEmergencyStop] != 0,
		MotionDone:    readI32(payload, offMotionDone) == 1,
		RawPayload:    append([]byte(nil), payload...),
		FrameCount:    frameCount,
		ReceivedAt:    time.Now(),
	}
	for i := 0; i < 6; i++ {
		fr.JointDegs[i] = readF64(payload, offJointPos+i*8)
		fr.TCPPose[i] = readF64(payload, offTCPPose+i*8)
		fr.JointTorques[i] = readF64(payload, offJointTorque+i*8)
	}
	return fr, true
}

func readF64(buf []byte, off int) float64 {
	return math.Float64frombits(binary.LittleEndian.Uint64(buf[off : off+8]))
}

func readI32(buf []byte, off int) int32 {
	return int32(binary.LittleEndian.Uint32(buf[off : off+4]))
}

// readCString returns the bytes up to the first NUL as a string.
func readCString(buf []byte) string {
	if i := bytes.IndexByte(buf, 0); i >= 0 {
		buf = buf[:i]
	}
	return string(buf)
}
