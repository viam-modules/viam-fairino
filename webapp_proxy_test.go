package viamfairino

import (
	"bufio"
	"context"
	"errors"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"net/url"
	"os"
	"strconv"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
)

// The proxy must reach an HTTPS upstream with a self-signed certificate (as
// the controller's WebApp is) and pass paths through unchanged.
func TestWebAppProxyForwardsToSelfSignedHTTPS(t *testing.T) {
	upstream := httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		_, _ = io.WriteString(w, "webapp:"+r.URL.Path)
	}))
	defer upstream.Close()
	target, err := url.Parse(upstream.URL)
	if err != nil {
		t.Fatal(err)
	}

	p, err := startWebAppProxy(context.Background(), target, 0, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	defer p.stop()

	port := strconv.Itoa(p.addr.(*net.TCPAddr).Port)
	resp, err := http.Get("http://127.0.0.1:" + port + "/robot/status")
	if err != nil {
		t.Fatal(err)
	}
	defer resp.Body.Close()
	body, _ := io.ReadAll(resp.Body)
	if string(body) != "webapp:/robot/status" {
		t.Errorf("got %d %q", resp.StatusCode, body)
	}
}

func TestWebAppProxyTarget(t *testing.T) {
	if u, err := webAppProxyTarget("", "192.168.58.2"); err != nil || u.String() != "https://192.168.58.2" {
		t.Errorf("default: got %v, %v", u, err)
	}
	if u, err := webAppProxyTarget("", "fd00::2"); err != nil || u.String() != "https://[fd00::2]" {
		t.Errorf("IPv6 host: got %v, %v", u, err)
	}
	if u, err := webAppProxyTarget("http://10.0.0.5:8080", "ignored"); err != nil || u.Host != "10.0.0.5:8080" {
		t.Errorf("override: got %v, %v", u, err)
	}
	for _, bad := range []string{"ftp://x", "not a url", "https://"} {
		if _, err := webAppProxyTarget(bad, "h"); err == nil {
			t.Errorf("%q: expected an error", bad)
		}
	}
}

// stop must close websocket tunnels too: the proxy hijacks them, and
// http.Server.Shutdown leaves hijacked connections running, so a WebApp tab
// left open could otherwise keep driving the arm through a "stopped" proxy.
func TestWebAppProxyStopClosesWebsockets(t *testing.T) {
	upstream := httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		conn, rw, err := w.(http.Hijacker).Hijack()
		if err != nil {
			return
		}
		defer conn.Close()
		_, _ = rw.WriteString("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n\r\n")
		_ = rw.Flush()
		_, _ = io.Copy(conn, rw) // echo until the proxy hangs up
	}))
	defer upstream.Close()
	target, err := url.Parse(upstream.URL)
	if err != nil {
		t.Fatal(err)
	}
	p, err := startWebAppProxy(context.Background(), target, 0, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}

	conn, err := net.Dial("tcp", "127.0.0.1:"+strconv.Itoa(p.addr.(*net.TCPAddr).Port))
	if err != nil {
		t.Fatal(err)
	}
	defer conn.Close()
	_, _ = io.WriteString(conn, "GET /ws HTTP/1.1\r\nHost: x\r\nConnection: Upgrade\r\nUpgrade: websocket\r\n\r\n")
	br := bufio.NewReader(conn)
	resp, err := http.ReadResponse(br, nil)
	if err != nil || resp.StatusCode != http.StatusSwitchingProtocols {
		t.Fatalf("upgrade: %v, %v", resp, err)
	}
	_, _ = io.WriteString(conn, "ping")
	buf := make([]byte, 4)
	if _, err := io.ReadFull(br, buf); err != nil || string(buf) != "ping" {
		t.Fatalf("tunnel not working: %q, %v", buf, err)
	}

	p.stop()
	_ = conn.SetReadDeadline(time.Now().Add(3 * time.Second))
	if _, err := br.ReadByte(); err == nil || errors.Is(err, os.ErrDeadlineExceeded) {
		t.Errorf("websocket tunnel still open after stop (read: %v)", err)
	}
}
