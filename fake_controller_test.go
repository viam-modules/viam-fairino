package viamfairino

import (
	"bufio"
	"net"
	"strings"
	"sync"
	"testing"
	"time"

	arm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
)

// fakeController is a minimal stand-in for the controller's command port
// (8080). It records every command's DATA and answers with reply(cmdID, data),
// which defaults to the "1" success ack every documented command returns.
type fakeController struct {
	t  *testing.T
	ln net.Listener

	mu       sync.Mutex
	commands []string
	at       []time.Time // when each command arrived
	reply    func(cmdID int, data string) string
}

func newFakeController(t *testing.T) *fakeController {
	t.Helper()
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	f := &fakeController{t: t, ln: ln}
	t.Cleanup(func() { _ = ln.Close() })
	go f.serve()
	return f
}

func (f *fakeController) port() int { return f.ln.Addr().(*net.TCPAddr).Port }

func (f *fakeController) serve() {
	for {
		conn, err := f.ln.Accept()
		if err != nil {
			return
		}
		go f.handle(conn)
	}
}

func (f *fakeController) handle(conn net.Conn) {
	defer conn.Close()
	r := bufio.NewReader(conn)
	for {
		raw, err := readCmdFrame(r)
		if err != nil {
			return
		}
		cnt, cmdID, data, err := decodeCmdFrame(raw)
		if err != nil {
			f.t.Errorf("fake controller: bad frame %q: %v", raw, err)
			return
		}
		f.mu.Lock()
		f.commands = append(f.commands, data)
		f.at = append(f.at, time.Now())
		reply := f.reply
		f.mu.Unlock()

		resp := "1"
		if reply != nil {
			resp = reply(cmdID, data)
		}
		if _, err := conn.Write([]byte(encodeCmdFrame(cnt, cmdID, resp))); err != nil {
			return
		}
	}
}

func (f *fakeController) setReply(reply func(cmdID int, data string) string) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.reply = reply
}

// sent returns the command names (DATA up to the first "(") received so far.
func (f *fakeController) sent() []string {
	f.mu.Lock()
	defer f.mu.Unlock()
	out := make([]string, len(f.commands))
	for i, c := range f.commands {
		out[i], _, _ = strings.Cut(c, "(")
	}
	return out
}

// received returns every command's full DATA and arrival time so far.
func (f *fakeController) received() ([]string, []time.Time) {
	f.mu.Lock()
	defer f.mu.Unlock()
	return append([]string(nil), f.commands...), append([]time.Time(nil), f.at...)
}

func (f *fakeController) count(name string) int {
	n := 0
	for _, c := range f.sent() {
		if c == name {
			n++
		}
	}
	return n
}

// fakeFKReply answers GetForwardKin with a fixed pose and everything else with
// the success ack, which is all a MoveJ round trip needs.
func fakeFKReply(cmdID int, _ string) string {
	if cmdID == cmdIDGetForwardKin {
		return "100,200,300,0,0,0"
	}
	return "1"
}

// newControllerTestArm builds an arm wired to fc's command port, with a status client
// seeded with fr and never started, so tests control what the arm "sees".
func newControllerTestArm(t *testing.T, fc *fakeController, fr statusFrame) (*viamFairinoFairino, *statusClient) {
	t.Helper()
	logger := logging.NewTestLogger(t)
	model, err := makeModelFrame("test", "fr5", false, nil)
	if err != nil {
		t.Fatal(err)
	}
	sc := newStatusClient("127.0.0.1", statusPort, logger)
	setTestFrame(sc, fr)
	cmd := newCommandClient("127.0.0.1", fc.port(), logger)
	t.Cleanup(func() { _ = cmd.close() })
	return &viamFairinoFairino{
		name:   arm.Named("test"),
		logger: logger,
		cfg:    &Config{},
		model:  model,
		status: sc,
		cmd:    cmd,
		opMgr:  operation.NewSingleOperationManager(),
	}, sc
}

func setTestFrame(sc *statusClient, fr statusFrame) {
	fr.ReceivedAt = time.Now()
	sc.mu.Lock()
	sc.frame = fr
	sc.have = true
	sc.mu.Unlock()
}

// restingFrame is a healthy, stationary arm at the given joint angles.
func restingFrame(jointsDeg [6]float64) statusFrame {
	return statusFrame{JointDegs: jointsDeg, MotionDone: true}
}
