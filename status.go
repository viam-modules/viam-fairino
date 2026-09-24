// Status feedback (TCP port 8083) reader for Fairino controllers.
//
// The controller pushes framed status data every feedback period (default
// 100 ms). Unlike Dobot's fixed-size packets, Fairino frames are variable
// length (header + count + LEN + payload + checksum), and TCP gives no
// message boundaries, so extractFrames reassembles frames from the byte
// stream and resynchronizes past corrupt data.
//
// Lifecycle mirrors viam-dobot's feedback.go: the reader runs in its own
// goroutine, reconnects with exponential backoff, and publishes the most
// recent valid frame behind a sync.RWMutex. On read/connect error the cached
// frame is invalidated so callers see the failure instead of stale data.
package viamfairino

import (
	"bytes"
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"net"
	"strconv"
	"sync"
	"time"

	"go.viam.com/rdk/logging"
)

const (
	dialTimeout = 5 * time.Second

	// readDeadline is comfortably above the slowest feedback period (100 ms)
	readDeadline = 200 * time.Millisecond

	reconnectBackoffMin = 200 * time.Millisecond
	reconnectBackoffMax = 5 * time.Second

	// maxBufferLen caps the reassembly buffer; if it grows this large without
	// yielding a frame the stream is hopelessly corrupt and is dropped
	maxBufferLen = 64 * 1024
)

type statusClient struct {
	host   string
	port   int
	logger logging.Logger

	mu    sync.RWMutex
	frame statusFrame
	have  bool
	err   error // last connection error, cleared on successful parse

	cancelFn context.CancelFunc
	doneCh   chan struct{}
}

func newStatusClient(host string, port int, logger logging.Logger) *statusClient {
	return &statusClient{host: host, port: port, logger: logger}
}

// start launches the background reader. ctx controls overall lifetime.
func (c *statusClient) start(parent context.Context) {
	ctx, cancel := context.WithCancel(parent)
	c.cancelFn = cancel
	c.doneCh = make(chan struct{})
	go c.run(ctx)
}

func (c *statusClient) stop() {
	if c.cancelFn != nil {
		c.cancelFn()
	}
	if c.doneCh != nil {
		<-c.doneCh
	}
}

// latest returns a snapshot of the last valid frame and whether one is live.
func (c *statusClient) latest() (statusFrame, bool, error) {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.frame, c.have, c.err
}

// current returns the latest frame, or an error saying why there is none,
// without waiting: for status and health checks, which should report a dead
// feed at once.
func (c *statusClient) current() (statusFrame, error) {
	fr, ok, err := c.latest()
	if ok {
		return fr, nil
	}
	if err == nil {
		err = errors.New("no status frame received yet")
	}
	return statusFrame{}, fmt.Errorf("status stream down: %w", err)
}

// await returns the latest frame, waiting briefly (waitForFrame) if none has
// arrived yet, as is typical right after startup.
func (c *statusClient) await(ctx context.Context) (statusFrame, error) {
	if fr, ok, _ := c.latest(); ok {
		return fr, nil
	}
	return c.waitForFrame(ctx)
}

// waitForFrameThroughErrors waits for a frame until ctx is done, riding out
// connection errors (the reader keeps retrying) that waitForFrame returns at
// once, and returns the last one if no frame arrives.
func (c *statusClient) waitForFrameThroughErrors(ctx context.Context) (statusFrame, error) {
	for {
		fr, ok, err := c.latest()
		if ok {
			return fr, nil
		}
		select {
		case <-ctx.Done():
			if err == nil {
				err = ctx.Err()
			}
			return statusFrame{}, fmt.Errorf("no status frame: %w", err)
		case <-time.After(20 * time.Millisecond):
		}
	}
}

func (c *statusClient) waitForFrame(ctx context.Context) (statusFrame, error) {
	deadline := time.Now().Add(2 * time.Second)
	if d, ok := ctx.Deadline(); ok && d.Before(deadline) {
		deadline = d
	}
	for {
		fr, ok, err := c.latest()
		if ok {
			return fr, nil
		}
		if err != nil {
			return statusFrame{}, err
		}
		if !time.Now().Before(deadline) {
			return statusFrame{}, errors.New("timed out waiting for status frame")
		}
		select {
		case <-ctx.Done():
			return statusFrame{}, ctx.Err()
		case <-time.After(20 * time.Millisecond):
		}
	}
}

func (c *statusClient) run(ctx context.Context) {
	defer close(c.doneCh)
	backoff := reconnectBackoffMin

	for ctx.Err() == nil {
		d := net.Dialer{Timeout: dialTimeout}
		conn, err := d.DialContext(ctx, "tcp", net.JoinHostPort(c.host, strconv.Itoa(c.port)))
		if err != nil {
			c.recordError(fmt.Errorf("status port dial: %w", err))
			if !sleepCtx(ctx, backoff) {
				return
			}
			backoff = min(backoff*2, reconnectBackoffMax)
			continue
		}
		backoff = reconnectBackoffMin
		c.readLoop(ctx, conn)
		_ = conn.Close()
	}
}

func (c *statusClient) readLoop(ctx context.Context, conn net.Conn) {
	readBuf := make([]byte, maxFrameDataLen)
	var buffer []byte
	loggedLen := false

	// Dropped frames are acceptable (the next frame supersedes anyway) but should be
	// visible, so a CNT jump logs a warning.
	var lastCount uint8
	haveLast := false

	for ctx.Err() == nil {
		_ = conn.SetReadDeadline(time.Now().Add(readDeadline))
		n, err := conn.Read(readBuf)
		if err != nil {
			c.recordError(fmt.Errorf("status read: %w", err))
			return
		}
		buffer = append(buffer, readBuf[:n]...)

		frames, rest := extractFrames(buffer)
		buffer = rest
		if len(buffer) > maxBufferLen {
			c.logger.Warnw("status stream corrupt, dropping reassembly buffer", "bytes", len(buffer))
			buffer = nil
		}

		for _, f := range frames {
			payload := f[frameOverheadLen : len(f)-frameChecksumLen]
			if !loggedLen {
				c.logger.Infow("first status frame received", "payload_len", len(payload))
				loggedLen = true
			}
			fr, ok := parseStatus(payload, f[frameHeaderLen])
			if !ok {
				c.logger.Debugw("status payload too short", "payload_len", len(payload))
				continue
			}
			if expected := lastCount + 1; haveLast && fr.FrameCount != expected {
				c.logger.Debugf("status frame count jumped — frames dropped",
					"expected", expected, "got", fr.FrameCount,
					"missed", uint8(fr.FrameCount-expected))
			}
			lastCount = fr.FrameCount
			haveLast = true
			c.mu.Lock()
			c.frame = fr
			c.have = true
			c.err = nil
			c.mu.Unlock()
		}
	}
}

// recordError marks the connection as failed and invalidates the cached frame
// so callers don't keep treating the last good frame as live state.
func (c *statusClient) recordError(err error) {
	c.mu.Lock()
	c.err = err
	c.have = false
	c.mu.Unlock()
}

func extractFrames(buf []byte) (frames [][]byte, rest []byte) {
	header := []byte{frameHeaderByte, frameHeaderByte}
	for {
		idx := bytes.Index(buf, header)
		if idx == -1 {
			// Keep a trailing 0x5A: it may be the first half of a header
			// split across reads.
			if len(buf) > 0 && buf[len(buf)-1] == frameHeaderByte {
				return frames, buf[len(buf)-1:]
			}
			return frames, nil
		}
		buf = buf[idx:]
		if len(buf) < frameOverheadLen {
			return frames, buf
		}
		dataLen := int(binary.LittleEndian.Uint16(buf[frameHeaderLen+frameCountLen:]))
		if dataLen > maxFrameDataLen {
			// Corrupt length — this wasn't a real header. Skip it and rescan.
			buf = buf[frameHeaderLen:]
			continue
		}
		total := frameOverheadLen + dataLen + frameChecksumLen
		if len(buf) < total {
			return frames, buf
		}
		frame := buf[:total]
		received := binary.LittleEndian.Uint16(frame[total-frameChecksumLen:])
		if received != sumChecksum(frame[:total-frameChecksumLen]) {
			buf = buf[frameHeaderLen:]
			continue
		}
		frames = append(frames, frame)
		buf = buf[total:]
	}
}

// sumChecksum is the uint16 sum of all bytes (manual Table 2-1, Sum Check).
func sumChecksum(data []byte) uint16 {
	var sum uint16
	for _, b := range data {
		sum += uint16(b)
	}
	return sum
}

func sleepCtx(ctx context.Context, d time.Duration) bool {
	t := time.NewTimer(d)
	defer t.Stop()
	select {
	case <-ctx.Done():
		return false
	case <-t.C:
		return true
	}
}

// ---------------------------------------------------------------------------
// Shared status clients
//
// The controller tolerates only one client on the status port at a time. On
// real hardware a second connection attempt hangs while one is live, and even
// after a client disconnects the controller never completes the close — the
// socket sits in FIN_WAIT_2 and blocks the next connection for the better part
// of a minute.
//
// That makes an independent status connection per resource unusable: an arm and
// a gripper on the same controller would fight over the single slot, and
// whichever lost would report a dead status feed. So resources share one reader
// per host:port, reference-counted, started on the first acquire and stopped
// statusReleaseGrace after the last release. One connection, one reader
// goroutine, every resource reading the same frames.
//
// The grace period is for reconfiguration: RDK closes a resource before it
// builds the replacement, and a reader stopped in between would leave the new
// one redialing into the FIN_WAIT_2 block above — with no frames for up to a
// minute, and a failed construction for anything that needs one at startup
// (bad_joints).
// ---------------------------------------------------------------------------

var (
	sharedStatusMu      sync.Mutex
	sharedStatusClients = map[string]*sharedStatusEntry{}
)

type sharedStatusEntry struct {
	client *statusClient
	refs   int
	idle   *time.Timer // the pending stop while refs == 0
}

// statusReleaseGrace is how long the shared reader outlives its last
// reference (see above). A variable so tests can shorten it.
var statusReleaseGrace = 10 * time.Second

// acquireStatusClient returns the shared reader for host:port, starting it on
// first use. Every acquire must be paired with a releaseStatusClient.
func acquireStatusClient(ctx context.Context, host string, port int, logger logging.Logger) *statusClient {
	key := net.JoinHostPort(host, strconv.Itoa(port))

	sharedStatusMu.Lock()
	defer sharedStatusMu.Unlock()

	if e, ok := sharedStatusClients[key]; ok {
		if e.idle != nil {
			// If it already fired, its func sees refs > 0 and leaves the
			// reader running.
			e.idle.Stop()
			e.idle = nil
		}
		e.refs++
		logger.Debugw("reusing shared status client", "addr", key, "refs", e.refs)
		return e.client
	}
	// The reader outlives the acquiring resource's construction context, so it
	// gets a background context; releaseStatusClient is what stops it.
	c := newStatusClient(host, port, logger)
	c.start(context.Background())
	sharedStatusClients[key] = &sharedStatusEntry{client: c, refs: 1}
	logger.Debugw("started shared status client", "addr", key)
	return c
}

// releaseStatusClient drops one reference to the shared reader for host:port,
// stopping it statusReleaseGrace after the last resource using it goes away,
// unless one acquires it again first.
func releaseStatusClient(host string, port int) {
	key := net.JoinHostPort(host, strconv.Itoa(port))

	sharedStatusMu.Lock()
	defer sharedStatusMu.Unlock()
	e, ok := sharedStatusClients[key]
	if !ok || e.refs == 0 {
		return
	}
	e.refs--
	if e.refs > 0 {
		return
	}
	var timer *time.Timer
	timer = time.AfterFunc(statusReleaseGrace, func() {
		sharedStatusMu.Lock()
		if sharedStatusClients[key] != e || e.refs > 0 || e.idle != timer {
			sharedStatusMu.Unlock()
			return
		}
		delete(sharedStatusClients, key)
		sharedStatusMu.Unlock()
		// Stop outside the lock: it waits for the reader goroutine to exit.
		e.client.stop()
	})
	e.idle = timer
}
