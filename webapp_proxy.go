package viamfairino

import (
	"context"
	"crypto/tls"
	"fmt"
	"net"
	"net/http"
	"net/http/httputil"
	"net/url"
	"time"

	"go.viam.com/rdk/logging"
)

// defaultWebAppProxyPort is where the WebApp proxy listens when
// Config.WebAppProxyPort is unset.
const defaultWebAppProxyPort = 18080

// webAppProxy is a reverse proxy to the controller's WebApp, for when the
// browser can't reach the controller directly (e.g. it sits on a private link
// to the viam-server host) — the counterpart of viam-ufactory-xarm's UFactory
// Studio proxy. It serves plain HTTP locally and has no authentication.
type webAppProxy struct {
	server *http.Server
	addr   net.Addr
	logger logging.Logger
}

// startWebAppProxy listens on port and forwards everything, websocket upgrades
// included, to target. The controller's WebApp serves HTTPS with a
// certificate it signed itself, so the upstream certificate isn't verified:
// the proxy only ever talks to the configured controller address.
func startWebAppProxy(ctx context.Context, target *url.URL, port int, logger logging.Logger) (*webAppProxy, error) {
	proxy := &httputil.ReverseProxy{
		Rewrite: func(r *httputil.ProxyRequest) {
			r.SetURL(target)
			r.Out.Host = target.Host
		},
		Transport: &http.Transport{
			Proxy: nil,
			//nolint:gosec // the WebApp's certificate is self-signed; see the doc comment.
			TLSClientConfig:     &tls.Config{InsecureSkipVerify: true},
			TLSHandshakeTimeout: 10 * time.Second,
		},
	}
	server := &http.Server{
		Addr:              fmt.Sprintf("[::]:%d", port), // '[::]' listens on IPv4 and IPv6
		Handler:           proxy,
		ReadHeaderTimeout: 10 * time.Second,
	}
	var lc net.ListenConfig
	ln, err := lc.Listen(ctx, "tcp", server.Addr)
	if err != nil {
		return nil, fmt.Errorf("WebApp proxy: failed to listen on port %d: %w", port, err)
	}
	logger.CInfow(ctx, "WebApp proxy listening", "port", port, "target", target.String())
	go func() {
		if err := server.Serve(ln); err != nil && err != http.ErrServerClosed {
			logger.Errorw("WebApp proxy stopped", "err", err)
		}
	}()
	return &webAppProxy{server: server, addr: ln.Addr(), logger: logger}, nil
}

// webAppProxyTarget resolves Config.WebAppURL, defaulting to https://<host>.
func webAppProxyTarget(configured, host string) (*url.URL, error) {
	if configured == "" {
		return &url.URL{Scheme: "https", Host: host}, nil
	}
	u, err := url.Parse(configured)
	if err != nil || (u.Scheme != "http" && u.Scheme != "https") || u.Host == "" {
		return nil, fmt.Errorf("webapp_url must be an http(s) URL with a host, got %q", configured)
	}
	return u, nil
}

func (p *webAppProxy) stop() {
	if p == nil {
		return
	}
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	if err := p.server.Shutdown(ctx); err != nil {
		p.logger.Warnw("WebApp proxy shutdown error", "err", err)
	}
}
