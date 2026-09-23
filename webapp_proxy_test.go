package viamfairino

import (
	"context"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"net/url"
	"strconv"
	"testing"

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
	if u, err := webAppProxyTarget("http://10.0.0.5:8080", "ignored"); err != nil || u.Host != "10.0.0.5:8080" {
		t.Errorf("override: got %v, %v", u, err)
	}
	for _, bad := range []string{"ftp://x", "not a url", "https://"} {
		if _, err := webAppProxyTarget(bad, "h"); err == nil {
			t.Errorf("%q: expected an error", bad)
		}
	}
}
