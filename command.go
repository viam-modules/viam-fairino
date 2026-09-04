// Command port (TCP 8080) client for Fairino controllers.
//
// This speaks the ASCII framing documented in the "Collaborative Robot
// Controller Communication Command Protocol User Manual" (V3.9.7), §1.1 —
// deliberately NOT whatever fairino-python-sdk's Robot.py actually puts on
// the wire. That SDK binds to a native library (libfairino) rather than
// opening this socket itself, and bring-up showed its bytes don't match the
// documented framing at all. Per Paige, this module targets the documented
// protocol on port 8080, full stop; command.go and constants.go's cmd*
// definitions are the only source of truth for what goes on the wire here.
//
// Frame format (manual Table 1-1-1; all fields are ASCII decimal text,
// "III" is the field separator):
//
//	/f/b III CNT III CMD_ID III LEN III DATA III /b/f
//
// CNT is a request/response correlation id ("frame count" in the manual —
// unrelated to status.go's binary CNT byte on port 8083). CMD_ID selects the
// command. LEN is len(DATA) in bytes. DATA is the command's textual form on
// the way out (e.g. "MoveJ(1.0,2.0,...)") and the comma-separated return
// values on the way back (usually just an errcode).
//
// Unlike the status port, this is a synchronous request/response protocol —
// one command in flight at a time, ack'd before the next is sent — so
// commandClient looks like dobot's dashClient (arm/client.go there) rather
// than status.go's async reader goroutine. Framing differs because dobot's
// port is newline/semicolon delimited while Fairino's is a fixed pair of
// text markers; readFrame here scans for the "/b/f" terminator instead of
// using a bufio.Scanner split function.
package viamfairino

import (
	"bufio"
	"bytes"
	"context"
	"errors"
	"fmt"
	"net"
	"regexp"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/logging"
)

const (
	cmdDialTimeout  = 5 * time.Second
	cmdWriteTimeout = 5 * time.Second
	// cmdReadTimeout bounds the ack wait when the caller's context carries no
	// deadline. It only covers the command being queued, not the physical
	// motion — completion is tracked separately by polling the status port.
	cmdReadTimeout = 10 * time.Second

	// cmdMaxFrameLen caps how much we'll buffer in readFrame while scanning
	// for a "/b/f" terminator that never arrives (corrupt stream, wrong
	// port, etc.) — real responses here are a handful of bytes.
	cmdMaxFrameLen = 4096
)

// commandClient is a synchronous request/response client for the command
// port. Safe for concurrent callers; requests are serialized like dobot's
// dashClient, since the controller is a single ASCII stream with no
// per-request multiplexing.
type commandClient struct {
	host   string
	port   int
	logger logging.Logger

	mu sync.Mutex // serializes one in-flight request

	connMu sync.RWMutex
	conn   net.Conn
	reader *bufio.Reader

	cnt atomic.Uint32 // rolling request id; wraps harmlessly
}

func newCommandClient(host string, port int, logger logging.Logger) *commandClient {
	return &commandClient{host: host, port: port, logger: logger}
}

// connect opens (or reopens) the TCP connection to the command port.
func (c *commandClient) connect(ctx context.Context) error {
	c.connMu.Lock()
	defer c.connMu.Unlock()

	// TEMP verbose bring-up logging — pare back to Debug once the command
	// port is confirmed working against real hardware.
	c.logger.Infow("command port: dialing", "host", c.host, "port", c.port)

	if c.conn != nil {
		_ = c.conn.Close()
		c.conn = nil
	}
	d := net.Dialer{Timeout: cmdDialTimeout}
	conn, err := d.DialContext(ctx, "tcp", net.JoinHostPort(c.host, strconv.Itoa(c.port)))
	if err != nil {
		c.logger.Infow("command port: dial failed", "host", c.host, "port", c.port, "err", err)
		return fmt.Errorf("command port dial %s:%d: %w", c.host, c.port, err)
	}
	c.logger.Infow("command port: connected", "host", c.host, "port", c.port, "local_addr", conn.LocalAddr())
	c.conn = conn
	c.reader = bufio.NewReaderSize(conn, 4096)
	return nil
}

func (c *commandClient) close() error {
	c.connMu.Lock()
	defer c.connMu.Unlock()
	if c.conn != nil {
		err := c.conn.Close()
		c.conn = nil
		return err
	}
	return nil
}

// send transmits one command frame and returns the parsed response DATA
// field. data is the full command text, e.g. "MoveJ(1,2,3,...)" or "STOP" —
// callers build that string (see moveJ/stop below), send just handles
// framing, the connection, and response correlation.
func (c *commandClient) send(ctx context.Context, cmdID int, data string) (string, error) {
	c.mu.Lock()
	defer c.mu.Unlock()

	c.connMu.RLock()
	conn := c.conn
	c.connMu.RUnlock()
	if conn == nil {
		if err := c.connect(ctx); err != nil {
			return "", err
		}
		c.connMu.RLock()
		conn = c.conn
		c.connMu.RUnlock()
	}

	reqCnt := int(c.cnt.Add(1))
	frame := encodeCmdFrame(reqCnt, cmdID, data)

	// TEMP verbose bring-up logging — every frame in both directions, plus
	// timing. This is the main thing we need to watch closely while
	// figuring out the real errcode convention (see parseErrcode) and
	// generally confirming the documented protocol actually matches what
	// the controller does. Pare back to Debug once that's settled.
	sendStart := time.Now()
	c.logger.Infow("command port: sending frame", "cnt", reqCnt, "cmd_id", cmdID, "data", data, "frame", frame)

	// No deadline on ctx typically means the caller means to bound the whole
	// operation (queue + physical motion), not just this ack — same
	// fallback dobot's dashClient uses for its comparable read.
	deadline, ok := ctx.Deadline()
	if !ok {
		deadline = time.Now().Add(cmdReadTimeout)
	}
	_ = conn.SetWriteDeadline(time.Now().Add(cmdWriteTimeout))
	_ = conn.SetReadDeadline(deadline)

	if _, err := conn.Write([]byte(frame)); err != nil {
		c.logger.Infow("command port: write failed", "cnt", reqCnt, "cmd_id", cmdID, "err", err)
		_ = c.close()
		return "", fmt.Errorf("command write %q: %w", data, err)
	}

	c.connMu.RLock()
	reader := c.reader
	c.connMu.RUnlock()
	raw, err := readCmdFrame(reader)
	if err != nil {
		c.logger.Infow("command port: read failed", "cnt", reqCnt, "cmd_id", cmdID, "elapsed", time.Since(sendStart), "err", err)
		_ = c.close()
		return "", fmt.Errorf("command read response to %q: %w", data, err)
	}
	c.logger.Infow("command port: received raw frame", "cnt", reqCnt, "cmd_id", cmdID, "raw", string(raw), "elapsed", time.Since(sendStart))

	respCnt, respCmdID, respData, err := decodeCmdFrame(raw)
	if err != nil {
		c.logger.Infow("command port: response decode failed", "cnt", reqCnt, "cmd_id", cmdID, "raw", string(raw), "err", err)
		return "", fmt.Errorf("malformed response to %q: %w", data, err)
	}
	c.logger.Infow("command port: parsed response", "sent_cnt", reqCnt, "resp_cnt", respCnt, "cmd_id", respCmdID, "data", respData)
	if respCnt != reqCnt {
		// The manual defines CNT as the request/response correlation id, but
		// we serialize one request at a time, so a mismatch just means the
		// controller doesn't echo it faithfully — worth knowing about, not
		// worth failing over, since CMD_ID already confirms we got the right
		// reply.
		c.logger.Debugw("command response CNT mismatch", "sent", reqCnt, "got", respCnt, "cmd", data)
	}
	if respCmdID == cmdIDScriptError {
		// See cmdIDScriptError's doc comment: the controller ran our command
		// through what looks like a Lua interpreter and the handler errored.
		scriptErr := scriptErrorFromData(respData)
		c.logger.Infow("command port: controller reported a script error", "cnt", reqCnt, "cmd_id", cmdID, "data", respData, "err", scriptErr)
		return "", fmt.Errorf("command %q: %w", data, scriptErr)
	}
	if respCmdID != cmdID {
		return "", fmt.Errorf("command %q: response CMD_ID %d does not match request %d (raw=%q)", data, respCmdID, cmdID, raw)
	}
	return respData, nil
}

// scriptErrorPattern extracts the embedded Table 10-1 code from a
// cmdIDScriptError response, e.g. [string "MoveJ(...)"]:1: robot errcode:154
// -> 154. Found empirically; not documented in the manual — see
// cmdIDScriptError's doc comment.
var scriptErrorPattern = regexp.MustCompile(`robot errcode:(\d+)`)

// scriptErrorFromData turns a cmdIDScriptError response's DATA into an error
// using the same chapter-10 table parseErrcode uses, when the "robot
// errcode:N" pattern matches; otherwise it falls back to the raw text so
// nothing is silently swallowed.
func scriptErrorFromData(data string) error {
	if m := scriptErrorPattern.FindStringSubmatch(data); m != nil {
		if code, err := strconv.Atoi(m[1]); err == nil {
			return &ControllerError{Code: code, Raw: data}
		}
	}
	return fmt.Errorf("controller reported a script error: %s", data)
}

// readCmdFrame reads bytes until it has seen a full "/b/f"-terminated frame.
// Unlike status.go's binary reassembly, this doesn't need the LEN field to
// know where the frame ends — the terminator marker is enough — so LEN is
// trusted as informational only, both here and in encodeCmdFrame.
func readCmdFrame(r *bufio.Reader) ([]byte, error) {
	var buf []byte
	for {
		b, err := r.ReadByte()
		if err != nil {
			return nil, err
		}
		buf = append(buf, b)
		if bytes.HasSuffix(buf, []byte(cmdFrameEnd)) {
			return buf, nil
		}
		if len(buf) > cmdMaxFrameLen {
			return nil, fmt.Errorf("response exceeded %d bytes without a %q terminator", cmdMaxFrameLen, cmdFrameEnd)
		}
	}
}

// encodeCmdFrame builds one request frame per manual Table 1-1-1.
// getSoftwareVersion queries the controller's GetSoftwareVersion (CMD_ID 905)
// and returns its three comma-separated fields. Used for model auto-detection
// (see module.go's detectModel); robotModel is the only field callers act on.
func (c *commandClient) getSoftwareVersion(ctx context.Context) (robotModel, webVersion, controllerVersion string, err error) {
	data, err := c.send(ctx, cmdIDGetSoftwareVersion, "GetSoftwareVersion()")
	if err != nil {
		return "", "", "", err
	}
	robotModel, webVersion, controllerVersion = parseSoftwareVersion(data)
	return robotModel, webVersion, controllerVersion, nil
}

// parseSoftwareVersion splits GetSoftwareVersion's response DATA
// ("<robotModel>,<webVersion>,<controllerVersion>") into its fields, tolerating
// a short/oversized field count (missing fields come back empty). Verified
// against an FR5: "FR5-V1-002(V6.0),v3.9.4,V3.9.22-QX".
func parseSoftwareVersion(data string) (robotModel, webVersion, controllerVersion string) {
	parts := strings.SplitN(strings.TrimSpace(data), ",", 3)
	get := func(i int) string {
		if i < len(parts) {
			return strings.TrimSpace(parts[i])
		}
		return ""
	}
	return get(0), get(1), get(2)
}

func encodeCmdFrame(cnt, cmdID int, data string) string {
	return cmdFrameStart + cmdFieldSep + strconv.Itoa(cnt) + cmdFieldSep +
		strconv.Itoa(cmdID) + cmdFieldSep + strconv.Itoa(len(data)) + cmdFieldSep +
		data + cmdFieldSep + cmdFrameEnd
}

// decodeCmdFrame parses a full "/f/b...III.../b/f" frame into its fields.
// DATA is rejoined from any remaining parts rather than taken as parts[3]
// alone, in case a response value legitimately contains the "III"
// separator (not expected for the numeric responses this module sends, but
// cheap to be defensive about).
func decodeCmdFrame(raw []byte) (cnt, cmdID int, data string, err error) {
	s := string(raw)
	if !strings.HasPrefix(s, cmdFrameStart) || !strings.HasSuffix(s, cmdFrameEnd) {
		return 0, 0, "", fmt.Errorf("missing frame markers: %q", s)
	}
	body := strings.TrimSuffix(strings.TrimPrefix(s, cmdFrameStart+cmdFieldSep), cmdFieldSep+cmdFrameEnd)
	parts := strings.Split(body, cmdFieldSep)
	if len(parts) < 4 {
		return 0, 0, "", fmt.Errorf("too few fields: %q", s)
	}
	cnt, err = strconv.Atoi(parts[0])
	if err != nil {
		return 0, 0, "", fmt.Errorf("bad CNT %q: %w", parts[0], err)
	}
	cmdID, err = strconv.Atoi(parts[1])
	if err != nil {
		return 0, 0, "", fmt.Errorf("bad CMD_ID %q: %w", parts[1], err)
	}
	// parts[2] is LEN — intentionally not validated against len(DATA); see
	// readCmdFrame's comment on why we don't rely on it for framing.
	data = strings.Join(parts[3:], cmdFieldSep)
	return cnt, cmdID, data, nil
}

// parseErrcode interprets a response DATA string as a bare command ack.
//
// Confirmed on real hardware, not just inferred from the manual: this is
// NOT the chapter 10 errcode table despite the manual calling the returned
// value "errcode" in every command's protocol table. A MoveJ that the
// pendant showed completing with no error, and that this module's own
// completion poll (statusFrame.MotionDone) confirmed reached the target,
// still came back with DATA "1" here — which matches every worked example
// in the manual for this response position (MoveJ, MoveC, MoveL, Mode,
// RobotEnable, SetSpeed, ...), all showing the identical "1". So "1" is
// success for this ack position. This is a genuinely different convention
// from chapter 10's numeric fault codes (0 = success, 3+ = specific
// faults), which do show up for real — but via the separate
// cmdIDScriptError response path (see scriptErrorFromData), not here. Kept
// permissive (0 also accepted) since chapter 10's own success value is 0
// and nothing rules out seeing it in this position for some command.
func parseErrcode(data string) error {
	v, err := strconv.Atoi(strings.TrimSpace(data))
	if err != nil {
		return fmt.Errorf("non-numeric errcode in response %q: %w", data, err)
	}
	if v == 0 || v == 1 {
		return nil
	}
	return &ControllerError{Code: v}
}

// ControllerError is a non-zero errcode returned by the controller, from either
// a command's own errcode response or an embedded "robot errcode:N" script
// error. Callers can inspect Code via errors.As to react to specific codes —
// e.g. gripper.go retries on 73 ("gripper movement error").
type ControllerError struct {
	Code int

	// Raw is the response text a script error was extracted from, kept so the
	// message stays as informative as the untyped version was. Empty for a
	// plain errcode response.
	Raw string
}

func (e *ControllerError) Error() string {
	if e.Raw != "" {
		return fmt.Sprintf("controller returned error %d (%s) [raw: %s]", e.Code, cmdErrorString(e.Code), e.Raw)
	}
	return fmt.Sprintf("controller returned error %d (%s)", e.Code, cmdErrorString(e.Code))
}

// isControllerErrcode reports whether err is a *ControllerError with the given
// code, at any depth in the wrap chain.
func isControllerErrcode(err error, code int) bool {
	var ce *ControllerError
	return errors.As(err, &ce) && ce.Code == code
}

// moveJParams mirrors manual Table 2-1 (MoveJ() instruction protocol 1) in
// field order. All values are the controller's native units: degrees for
// joint/orientation fields, millimeters for position fields, percent for
// speed/accel/ovl.
type moveJParams struct {
	Joints [6]float64 // J1..J6, degrees — the actual PTP target

	// Pose is the desc_pos (x,y,z,rx,ry,rz) field. Confirmed on real
	// hardware (not just inferred from the manual) that this is checked
	// against the joint target, not just echoed/logged: a MoveJ sent with
	// the arm's *current* pose here (rather than the pose the joint target
	// actually corresponds to) was rejected with errcode 154, "the joint
	// command point is incorrect". Callers must set this from
	// commandClient.getForwardKin(joints) — the controller's own FK, which
	// this module needs regardless of not having a local kinematic model
	// (Phase 3, out of scope here).
	Pose [6]float64

	ToolNum      int
	WorkPieceNum int

	SpeedPercent float64 // 0..100
	// AccelPercent is hardcoded to 0 by every caller in this module: per
	// fairino-python-sdk's MoveJ docstring this parameter is "暂不开放"
	// ("not yet open/functional") and defaults to 0.0 there too, so 0 here
	// matches the vendor's own default rather than an arbitrary choice.
	AccelPercent float64
	OverridePct  int // ovl, 0..100 — additional global speed scale factor

	ExAxisPos [4]float64 // no external axes on this arm; always zero

	// BlendT: -1 blocks (decelerates to a stop) at this point before any
	// next queued motion; [0,500] is a non-blocking smooth-transition time
	// in ms. MoveToJointPositions is a single discrete target, so this is
	// always -1 — the block/no-block distinction only matters when chaining
	// waypoints (MoveThroughJointPositions, not yet implemented).
	BlendT float64

	OffsetFlag uint8      // 0 = no offset; always 0 here (no workpiece/tool offset support yet)
	Offset     [6]float64 // dt_x,dt_y,dt_z,dt_rx,dt_ry,dt_rz — zero, per OffsetFlag above
}

// formatMoveJ renders p as the literal MoveJ(...) command text, in the
// exact 29-argument order of manual Table 2-1.
func formatMoveJ(p moveJParams) string {
	return fmt.Sprintf(
		"MoveJ(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"+ // J1..J6
			"%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"+ // x,y,z,rx,ry,rz
			"%d,%d,"+ // toolNum, workPieceNum
			"%.4f,%.4f,%d,"+ // speed, acc, ovl
			"%.4f,%.4f,%.4f,%.4f,"+ // exaxisPos1..4
			"%.4f,%d,"+ // blendT, offset_flag
			"%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)", // dt_x..dt_rz
		p.Joints[0], p.Joints[1], p.Joints[2], p.Joints[3], p.Joints[4], p.Joints[5],
		p.Pose[0], p.Pose[1], p.Pose[2], p.Pose[3], p.Pose[4], p.Pose[5],
		p.ToolNum, p.WorkPieceNum,
		p.SpeedPercent, p.AccelPercent, p.OverridePct,
		p.ExAxisPos[0], p.ExAxisPos[1], p.ExAxisPos[2], p.ExAxisPos[3],
		p.BlendT, p.OffsetFlag,
		p.Offset[0], p.Offset[1], p.Offset[2], p.Offset[3], p.Offset[4], p.Offset[5],
	)
}

// moveJ sends a MoveJ() PTP joint move (manual §2.1, CMD_ID 201) and
// returns once the controller has acknowledged the command as queued — it
// does NOT wait for the physical move to finish. Callers poll the status
// port (statusFrame.MotionDone) for that, same division of labor as
// dobot's jointMovJ + waitForMotionCompleteLocked.
func (c *commandClient) moveJ(ctx context.Context, p moveJParams) error {
	// TEMP verbose bring-up logging.
	c.logger.Infow("moveJ: params", "joints_deg", p.Joints, "pose", p.Pose,
		"tool_num", p.ToolNum, "workpiece_num", p.WorkPieceNum,
		"speed_pct", p.SpeedPercent, "accel_pct", p.AccelPercent, "ovl_pct", p.OverridePct,
		"blend_t", p.BlendT, "offset_flag", p.OffsetFlag)
	resp, err := c.send(ctx, cmdIDMoveJ, formatMoveJ(p))
	if err != nil {
		c.logger.Infow("moveJ: send failed", "err", err)
		return err
	}
	err = parseErrcode(resp)
	c.logger.Infow("moveJ: result", "resp", resp, "err", err)
	return err
}

// getForwardKin sends GetForwardKin() (manual §6.3.24, CMD_ID 377), which
// asks the controller to compute the tool pose for a given joint position.
// Unlike moveJ's response, this one has no wrapping errcode — the returned
// value table (Table 6-3-24) is just x,y,z,rx,ry,rz directly, comma
// separated with no braces (e.g. "952.551575,-306.108032,...").
//
// This exists so MoveJ's desc_pos field (moveJParams.Pose) can be filled
// with the real pose for the joint target instead of a placeholder: hardware
// bring-up showed the controller rejects a MoveJ whose desc_pos doesn't
// correspond to its joint target (errcode 154, "the joint command point is
// incorrect" — see cmdIDScriptError's doc comment for how that surfaces).
// This module has no local kinematic model (Phase 3, out of scope), but it
// doesn't need one for this — the controller computes FK itself.
func (c *commandClient) getForwardKin(ctx context.Context, jointsDeg [6]float64) ([6]float64, error) {
	data := fmt.Sprintf("GetForwardKin(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)",
		jointsDeg[0], jointsDeg[1], jointsDeg[2], jointsDeg[3], jointsDeg[4], jointsDeg[5])
	c.logger.Infow("getForwardKin: request", "joints_deg", jointsDeg)
	resp, err := c.send(ctx, cmdIDGetForwardKin, data)
	if err != nil {
		c.logger.Infow("getForwardKin: send failed", "err", err)
		return [6]float64{}, err
	}
	pose, err := parsePose6(resp)
	c.logger.Infow("getForwardKin: result", "resp", resp, "pose", pose, "err", err)
	return pose, err
}

// parsePose6 parses a bare "x,y,z,rx,ry,rz" response (no errcode, no
// braces) into a [6]float64, e.g. GetForwardKin's return value.
func parsePose6(data string) ([6]float64, error) {
	parts := strings.Split(strings.TrimSpace(data), ",")
	if len(parts) != 6 {
		return [6]float64{}, fmt.Errorf("expected 6 comma-separated values in pose response, got %d: %q", len(parts), data)
	}
	var pose [6]float64
	for i, p := range parts {
		v, err := strconv.ParseFloat(strings.TrimSpace(p), 64)
		if err != nil {
			return [6]float64{}, fmt.Errorf("non-numeric value %q in pose response %q: %w", p, data, err)
		}
		pose[i] = v
	}
	return pose, nil
}

// stop sends STOP (manual §2.35, CMD_ID 102), which halts in-flight motion.
func (c *commandClient) stop(ctx context.Context) error {
	c.logger.Infow("stop: sending STOP")
	resp, err := c.send(ctx, cmdIDStop, "STOP")
	if err != nil {
		c.logger.Infow("stop: send failed", "err", err)
		return err
	}
	err = parseErrcode(resp)
	c.logger.Infow("stop: result", "resp", resp, "err", err)
	return err
}

// ---------------------------------------------------------------------------
// Gripper commands (manual chapter 7.1, "Robot peripheral instructions").
//
// These are the same commands the vendor SDK issues over XML-RPC on TCP 20003,
// but on the documented command port with documented framing — same names,
// same argument order. Gripper *state* (position, motion-done, fault, activation)
// is deliberately NOT read back here: chapter 7.1 has no GetGripperMotionDone
// because the controller publishes that on the status port instead, so
// gripper.go reads it from statusFrame (see constants.go's gripper offsets).
// ---------------------------------------------------------------------------

// setGripperConfig sends SetGripperConfig() (§7.1.1, CMD_ID 226), recording
// which physical gripper is attached. Per hardware bring-up this only records
// the vendor selection — it does not enable the end-tool RS-485 bus, which
// still has to be done once on the controller (WebApp/teach pendant).
func (c *commandClient) setGripperConfig(ctx context.Context, company, device, softwareVersion, bus int) error {
	data := fmt.Sprintf("SetGripperConfig(%d,%d,%d,%d)", company, device, softwareVersion, bus)
	resp, err := c.send(ctx, cmdIDSetGripperConfig, data)
	if err != nil {
		return err
	}
	return parseErrcode(resp)
}

// gripperConfig is GetGripperConfig()'s return value (§7.1.2, Table 7-1-2).
type gripperConfig struct {
	Company         int
	Device          int
	SoftwareVersion int

	// Bus is a fourth value the controller actually returns but Table 7-1-2's
	// return-value list omits (the table stops at id_softversion, while its own
	// example frame shows four fields, and hardware returns four: a DH gripper
	// came back "03 00 00 01"). Treated as optional so a controller that
	// returns only three still parses.
	Bus    int
	HasBus bool
}

// getGripperConfig sends GetGripperConfig() (§7.1.2, CMD_ID 229). Unlike the
// action commands its response carries no leading errcode — Table 7-1-2 lists
// the return values directly.
func (c *commandClient) getGripperConfig(ctx context.Context) (gripperConfig, error) {
	resp, err := c.send(ctx, cmdIDGetGripperConfig, "GetGripperConfig()")
	if err != nil {
		return gripperConfig{}, err
	}
	vals, err := parseIntList(resp)
	if err != nil {
		return gripperConfig{}, fmt.Errorf("GetGripperConfig: %w", err)
	}
	if len(vals) < 3 {
		return gripperConfig{}, fmt.Errorf("GetGripperConfig: expected at least 3 values, got %d: %q", len(vals), resp)
	}
	conf := gripperConfig{Company: vals[0], Device: vals[1], SoftwareVersion: vals[2]}
	if len(vals) >= 4 {
		conf.Bus, conf.HasBus = vals[3], true
	}
	return conf, nil
}

// actGripper sends ActGripper() (§7.1.3, CMD_ID 227). action is 0 to reset,
// 1 to activate. Activation drives a physical homing motion that takes seconds;
// callers must wait for it before issuing a move (see gripper.go's activate).
func (c *commandClient) actGripper(ctx context.Context, index, action int) error {
	data := fmt.Sprintf("ActGripper(%d,%d)", index, action)
	resp, err := c.send(ctx, cmdIDActGripper, data)
	if err != nil {
		return err
	}
	return parseErrcode(resp)
}

// moveGripperParams mirrors manual Table 7-1-4 in field order.
type moveGripperParams struct {
	Index    int // clamp number
	Pos      int // position percentage, 0-100
	Vel      int // speed percentage, 0-100
	Force    int // torque percentage, 0-100
	MaxTime  int // max wait, 0-30000 ms
	Block    int // 0 = block, 1 = non-block
	Type     int // 0 = parallel, 1 = rotary
	RotNum   float64
	RotSpeed int
	RotTorq  int
}

// formatMoveGripper renders p as the literal MoveGripper(...) command text, in
// the exact 10-argument order of manual Table 7-1-4.
func formatMoveGripper(p moveGripperParams) string {
	return fmt.Sprintf("MoveGripper(%d,%d,%d,%d,%d,%d,%d,%g,%d,%d)",
		p.Index, p.Pos, p.Vel, p.Force, p.MaxTime, p.Block, p.Type, p.RotNum, p.RotSpeed, p.RotTorq)
}

// moveGripper sends MoveGripper() (§7.1.4, CMD_ID 228). With Block=0 the
// controller does not acknowledge until the fingers have stopped, so the
// caller's context deadline must cover the motion, not just the queueing.
func (c *commandClient) moveGripper(ctx context.Context, p moveGripperParams) error {
	resp, err := c.send(ctx, cmdIDMoveGripper, formatMoveGripper(p))
	if err != nil {
		return err
	}
	return parseErrcode(resp)
}

// getRobotErrorCode sends GetRobotErrorCode() (§6.3.16, CMD_ID 1163) and
// returns (mainCode, subCode). Its response has no leading errcode either.
func (c *commandClient) getRobotErrorCode(ctx context.Context) (mainCode, subCode int, err error) {
	resp, err := c.send(ctx, cmdIDGetRobotErrorCode, "GetRobotErrorCode()")
	if err != nil {
		return 0, 0, err
	}
	vals, err := parseIntList(resp)
	if err != nil {
		return 0, 0, fmt.Errorf("GetRobotErrorCode: %w", err)
	}
	if len(vals) < 2 {
		return 0, 0, fmt.Errorf("GetRobotErrorCode: expected 2 values, got %d: %q", len(vals), resp)
	}
	return vals[0], vals[1], nil
}

// parseIntList parses a response DATA field holding a list of integers. The
// manual's examples are inconsistent about the separator — GetRobotErrorCode
// shows "0,0" while GetGripperConfig shows "30 00 00 00" — so both commas and
// whitespace are accepted as separators.
func parseIntList(data string) ([]int, error) {
	fields := strings.FieldsFunc(strings.TrimSpace(data), func(r rune) bool {
		return r == ',' || r == ' ' || r == '\t'
	})
	out := make([]int, 0, len(fields))
	for _, f := range fields {
		v, err := strconv.Atoi(f)
		if err != nil {
			return nil, fmt.Errorf("non-numeric value %q in response %q: %w", f, data, err)
		}
		out = append(out, v)
	}
	return out, nil
}
