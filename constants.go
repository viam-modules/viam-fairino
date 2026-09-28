// Wire-format definitions for the Fairino controller's status feedback port
// (TCP 8083), per "Robot 8083 Port Status Feedback User Manual" V3.9.7. The
// bottom of the file also has the command port (TCP 8080) constants and
// error table, per "Collaborative Robot Controller Communication Command
// Protocol User Manual" V3.9.7 — command.go is to that section what
// status.go is to the rest of this file.
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

// Gripper payload offsets (manual Table 2-2 rows 41 and 56-63). These are the
// realtime source for everything the gripper component reads back: the
// controller publishes gripper state here, not over the command port, which is
// why chapter 7's command set has no GetGripperMotionDone.
//
// The offsets are derived by walking Table 2-2's byte counts from row 41
// (which starts right after robot_motion_done's int32):
//
//	41 gripper_motion_done  1 @233   50 exaxis_active_flag     1 @376
//	42 servo_id             1 @234   51 exaxis_motion_status   1 @377
//	43 servo_errcode        4 @235   52 cl_analog_input[2]     4 @378
//	44 servo_state          4 @239   53 tl_analog_input        2 @382
//	45 servo_actual_pos     8 @243   54 cl_analog_output[2]    4 @384
//	46 servo_actual_speed   4 @251   55 tl_analog_output       2 @388
//	47 servo_actual_torque  4 @255   56 gripper_fault_id       1 @390
//	48 exaxis_out_slimit    1 @259   57 gripper_fault          2 @391
//	49 exaxis_status[4]   116 @260   58 gripper_active         2 @393
//	                                 59 gripper_position       1 @395
//
// Row 49's 116 bytes is 4 axes x the 29-byte Table 2-3 struct (8+8+4+9x1).
const (
	offGripperMotionDone = 233 // uint8, see gripperMotionDone* below
	offGripperFaultID    = 390 // uint8, index of the faulted gripper
	offGripperFault      = 391 // uint16
	offGripperActive     = 393 // uint16, 0 = not activated
	offGripperPosition   = 395 // uint8, 0-100%
	offGripperSpeed      = 396 // int8, percent
	offGripperCurrent    = 397 // int8, percent
	offGripperTemp       = 398 // int32
	offGripperVoltage    = 402 // int32

	// minGripperPayloadLen is the end of the gripper field set. Frames shorter
	// than this (older firmware, or a truncated table) still parse — the
	// gripper fields are just marked invalid, so the arm keeps working.
	minGripperPayloadLen = offGripperVoltage + 4
)

// gripper_motion_done values for DH/Dahuan grippers (Table 2-2 row 41). Each
// vendor encodes this field differently; the AG-105-145 this module was brought
// up against is a DH, and the DH column is the one implemented here. Robotiq
// and Huiling/Tianji use different encodings — revisit if another vendor is
// configured.
const (
	gripperMotionMoving  = 0 // movement not completed
	gripperMotionStopped = 1 // stopped, object NOT clamped
	gripperMotionClamped = 2 // stopped, object clamped
	gripperMotionDropped = 3 // object was clamped and then dropped
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

// warningCodes are error_code values the controller raises without stopping
// motion. waitForMotionComplete doesn't fail a move on these; a move that
// reaches its target despite one still succeeds.
var warningCodes = map[uint8]bool{
	12: true, // joint configuration warning
}

func isWarningCode(v uint8) bool { return warningCodes[v] }

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

	// Gripper fields (Table 2-2 rows 41, 56-63). GripperValid is false when the
	// payload was too short to contain them, in which case they are all zero
	// and callers must not read them as real state.
	GripperValid      bool
	GripperMotionDone uint8 // see gripperMotion* constants (DH encoding)
	GripperFaultID    uint8
	GripperFault      uint16
	GripperActive     uint16 // 0 = not activated
	GripperPosition   uint8  // 0-100%
	GripperSpeed      int8
	GripperCurrent    int8
	GripperTemp       int32
	GripperVoltage    int32

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
	if len(payload) >= minGripperPayloadLen {
		fr.GripperValid = true
		fr.GripperMotionDone = payload[offGripperMotionDone]
		fr.GripperFaultID = payload[offGripperFaultID]
		fr.GripperFault = readU16(payload, offGripperFault)
		fr.GripperActive = readU16(payload, offGripperActive)
		fr.GripperPosition = payload[offGripperPosition]
		fr.GripperSpeed = int8(payload[offGripperSpeed])
		fr.GripperCurrent = int8(payload[offGripperCurrent])
		fr.GripperTemp = readI32(payload, offGripperTemp)
		fr.GripperVoltage = readI32(payload, offGripperVoltage)
	}
	return fr, true
}

func readF64(buf []byte, off int) float64 {
	return math.Float64frombits(binary.LittleEndian.Uint64(buf[off : off+8]))
}

func readI32(buf []byte, off int) int32 {
	return int32(binary.LittleEndian.Uint32(buf[off : off+4]))
}

func readU16(buf []byte, off int) uint16 {
	return binary.LittleEndian.Uint16(buf[off : off+2])
}

// readCString returns the bytes up to the first NUL as a string.
func readCString(buf []byte) string {
	if i := bytes.IndexByte(buf, 0); i >= 0 {
		buf = buf[:i]
	}
	return string(buf)
}

// ---------------------------------------------------------------------------
// Command port (TCP 8080) wire-format definitions, per "Collaborative Robot
// Controller Communication Command Protocol User Manual" V3.9.7. I/O and
// frame encode/decode live in command.go; this section is the readable
// single source of truth for the constants that file uses, mirroring how
// the status-port constants above relate to status.go.
// ---------------------------------------------------------------------------

// Frame markers (manual Table 1-1-1). Unlike the status port's binary
// 0x5A5A header, these are literal ASCII text.
const (
	cmdFrameStart = "/f/b"
	cmdFrameEnd   = "/b/f"
	cmdFieldSep   = "III"
)

// Command numbers (CMD_ID) this module sends.
const (
	cmdIDMoveJ         = 201 // manual §2.1, Table 2-1
	cmdIDStop          = 102 // manual §2.35, Table 2-48
	cmdIDGetForwardKin = 377 // manual §6.3.24, Table 6-3-24

	// Gripper commands, manual chapter 7.1 ("Robot peripheral instructions").
	cmdIDSetGripperConfig = 226 // §7.1.1, Table 7-1-1
	cmdIDActGripper       = 227 // §7.1.3, Table 7-1-3
	cmdIDMoveGripper      = 228 // §7.1.4, Table 7-1-4
	cmdIDGetGripperConfig = 229 // §7.1.2, Table 7-1-2

	cmdIDGetRobotErrorCode = 1163 // §6.3.16, Table 6-3-16

	// cmdIDGetSoftwareVersion sends "GetSoftwareVersion()"; the response DATA
	// is "<robotModel>,<webVersion>,<controllerVersion>" (verified against an
	// FR5 on real hardware: "FR5-V1-002(V6.0),v3.9.4,V3.9.22-QX"). robotModel
	// drives model auto-detection (kinematics.go's modelFromRobotModel). Same
	// CMD_ID the vendor SDK uses over this port — see fairino-python-sdk
	// testUDP1.py's raw "/f/b...905...GetSoftwareVersion()...".
	cmdIDGetSoftwareVersion = 905

	// cmdIDScriptError is NOT documented anywhere in the manual — found on
	// real hardware. When a command handler fails, the controller's response
	// frame doesn't echo the request's CMD_ID; it comes back as this CMD_ID
	// instead, with DATA holding a Lua traceback of the form
	// `[string "<the command text we sent>"]:1: robot errcode:<N>` (<N> is a
	// normal Table 10-1 code — e.g. sending MoveJ() with a desc_pos that
	// doesn't correspond to the joint target came back errcode 154, "the
	// joint command point is incorrect"). This strongly suggests the
	// controller runs incoming commands through a Lua interpreter rather
	// than a hand-rolled parser, and 500 is its generic "script execution
	// error" response — not specific to MoveJ. See scriptErrorFromData in
	// command.go, which extracts the embedded errcode.
	cmdIDScriptError = 500
)

// cmdErrorNames is the full controller error-code table from manual chapter
// 10 ("Robot instruction interface error code", Table 10-1). Codes not
// listed there (gaps in the manual's own numbering, e.g. 1-2, 53-58, 65,
// 71-72, ...) fall through to cmdErrorString's "unknown error code N".
var cmdErrorNames = map[int]string{
	0:   "success",
	3:   "the number of parameters is abnormal",
	4:   "the parameter value is abnormal",
	5:   "the tpd timer failed to start",
	6:   "the tpd timer failed to close",
	7:   "the tpd file creation failed",
	8:   "the tpd file does not exist",
	9:   "the tpd file name was not sent successfully",
	10:  "the content of tpd file failed to send",
	11:  "the program is abnormal and parsing stops",
	12:  "the contents of the tpd file are inconsistent",
	13:  "the number of points in the tpd file is abnormal",
	14:  "the instruction execution failed",
	15:  "the number of tpd record points is out of limit",
	16:  "the protocol has already been loaded",
	17:  "the protocol is not loaded",
	18:  "the program is running",
	19:  "the posture sensor communication is abnormal",
	20:  "the tool is not set for welding",
	21:  "the external shaft is not deactivated",
	22:  "the three-point method does not set up the tool",
	23:  "position sensor data acquisition failed",
	24:  "eight-point method: the first four points have too much posture change",
	25:  "calculation failed",
	26:  "eight-point method: not switched to the base coordinate system",
	27:  "eight-point method: the calculation result is abnormal",
	28:  "the results of inverse kinematics calculation are abnormal",
	29:  "ServoJ joint over limit",
	30:  "reset failed, please restart the control box",
	31:  "emergency stop released — please disconnect power and restart the control box",
	32:  "joint over-limit",
	33:  "the external axis is not at zero, so the guide/resolution settings failed",
	34:  "workpiece number error",
	35:  "please switch to workpiece number 0",
	36:  "the file name is too long",
	37:  "tool number error",
	38:  "strange posture",
	39:  "socket name is invalid",
	40:  "speed percentage exceeds limit",
	41:  "the external shaft has not returned to zero",
	42:  "the posture changes too much",
	43:  "the conveyor belt detection switch DI is not configured",
	44:  "the robot's attitude angle is out of limit",
	45:  "the external axis is not activated",
	46:  "the synchronization function requires calibration of the external axis",
	47:  "external driver information configuration failed",
	48:  "external shaft drive information configuration timed out",
	49:  "external shaft error, cannot be enabled",
	50:  "failed to obtain information from the external axis driver",
	51:  "external shaft driver information acquisition timed out",
	52:  "the synchronization function cannot be used for single-step operation",
	59:  "the force/torque sensor is not activated",
	60:  "the force/torque sensor's reference coordinate system has not been switched to the tool",
	61:  "the force/torque sensor is not set to zero",
	62:  "the load of the torque sensor is not set to zero",
	63:  "failed to obtain system time",
	64:  "not added to instruction queue",
	66:  "the midpoint 1 of the circular/spiral instruction is incorrect",
	67:  "the midpoint 2 of the circular/spiral instruction is incorrect",
	68:  "the midpoint 3 of the circular/spiral instruction is incorrect",
	69:  "the midpoint of the arc instruction is incorrect",
	70:  "the target point of the arc instruction is incorrect",
	73:  "claw movement error",
	74:  "the linear command point is incorrect",
	75:  "channel error",
	76:  "waiting for timeout",
	82:  "TPD instruction point error",
	83:  "the TPD directive tool is not compatible with the current tool",
	84:  "weld seam location failed",
	85:  "line directive error",
	90:  "external axis configuration file check failed",
	91:  "the peripheral configuration file version does not match",
	92:  "the peripheral configuration file read failed",
	93:  "the number of spline instruction points is out of limit",
	94:  "the spline instruction point is incorrect",
	95:  "the spline parameters are incorrect",
	96:  "wire positioning failed",
	97:  "the recorded data is empty",
	98:  "the main program is not configured",
	99:  "the security stop has been triggered",
	100: "the job origin has not been configured",
	101: "the robot is not enabled",
	106: "the external shaft is not enabled",
	108: "the starting point of the helix command is incorrect",
	110: "close the drag configuration / impedance callback to enter the interference area",
	111: "please turn off the drag teaching lock-free-degree function",
	112: "the positioning posture cannot be reached",
	114: "the DMP instruction point is incorrect",
	115: "circular motion instruction point error",
	116: "the extended axis peripheral communication driver is not loaded",
	118: "the load weight of the force sensor is incorrect",
	119: "the load center of gravity is wrong under the force sensor",
	120: "the trajectory command point is incorrect",
	121: "the number of trajectory points is 0",
	122: "joint torque exceeds limit",
	123: "please exit drag mode first",
	124: "the extended axis parameters are not configured",
	125: "recovery of welding failed: the original program failed",
	126: "recovery welding failed: unable to obtain the restart arc point",
	127: "restore welding failed: unable to generate restore program",
	128: "not in manual mode / stopped",
	129: "please switch to the new dynamics mode first",
	130: "the point table does not exist",
	133: "please switch to drag mode / force-sensor-assisted drag mode first",
	134: "the force/torque sensor is unloaded",
	135: "the inverse of the force/torque sensor's matrix is abnormal",
	138: "the detection switch DI is not configured",
	139: "please apply the extended axis coordinate system first",
	140: "please activate the current extension axis first",
	143: "the teaching point information does not exist",
	144: "the LUA file does not exist",
	151: "the joint configuration changed",
	152: "the spacing between welding instruction points is too small",
	153: "the spacing between arc instructions is too small",
	154: "the joint command point is incorrect",
	155: "the background program failed to start",
	156: "the background program failed to pause",
	157: "the background program failed to resume",
	158: "the background program deletion failed",
	159: "the number of background programs running exceeds 8",
	160: "the speed profile validation failed",
	161: "load profile validation failed",
	162: "the extended axis in laser tracking cannot move asynchronously",
	163: "the welding procedure contains multiple while statements",
	164: "segment welding information not returned",
	165: "CNC communication error",
	166: "CNC timeout",
	167: "CNC communication connection timed out",
	168: "the number of monitored numeric variables exceeds the limit",
	169: "the number of monitored character variables exceeds the limit",
	170: "workpiece coordinate system not applied",
	171: "all joints must be moved within the soft limit range",
	172: "motion speed cannot be 0",
	173: "acceleration smoothing must be enabled for extended axis blend motion",
	174: "current robot model does not support this function",
	175: "socket not configured",
	176: "socket communication connected",
	177: "socket communication not connected",
	178: "firmware version too high, current software version not supported",
	179: "firmware version too low, current software version not supported",
	180: "G-code file cannot be opened",
	181: "G-code parsing failed",
	182: "given physical speed exceeds limit",
	183: "given physical acceleration exceeds limit",
	184: "external axis enable failed",
	185: "motion stopped by error signal",
	186: "motion stopped by emergency stop signal",
	187: "collision level too low",
	188: "force control and impedance control both enabled",
	189: "kernel upgrade version verification failed",
	190: "kernel version upgrade failed",
	191: "linear rack guide not activated",
	192: "linear rack guide collision detection is only supported for upright installation",
	193: "torque control speed exceeds limit",
	194: "torque control power exceeds limit",
	195: "FR30L base tilt angle must be 0",
	196: "the photoelectric sensor signal has not been triggered",
	197: "the joint torque sensor has not completed zero-point or sensitivity calibration",
	198: "the joint torque sensor exhibits an anomaly",
	199: "error in the interference range set for the multi-interference zone (multiple cubes can only be configured within the specified range)",
	200: "the robot is operating in intermittent mode",
	201: "swinging configuration file check failed",
	202: "interference configuration file check failed",
	203: "end 485 communication error",
	// 204: MoveJ sent while the controller's motion queue is full (e.g.
	// waypoints sent faster than it can drain). Surfaces as a normal MoveJ
	// error; no retry/backoff is implemented for it.
	204: "the instruction queue is full; check the instruction sending frequency",
	205: "the endpoint protocol is configured; close it first",
	206: "the adapted devices are configured; clear the current adapted devices first",
	207: "automatic mode safe stop requires three-position enablement with the teaching device",
	208: "the current DI configuration is occupied and cannot be changed",
	209: "not in tracking mode",
	210: "conveyor belt calibration error",
}

func cmdErrorString(code int) string {
	if s, ok := cmdErrorNames[code]; ok {
		return s
	}
	return fmt.Sprintf("unknown error code %d", code)
}
