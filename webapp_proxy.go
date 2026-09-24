package viamfairino

import (
	"context"
	"crypto/tls"
	"errors"
	"fmt"
	"net"
	"net/http"
	"net/http/httputil"
	"net/url"
	"strings"
	"time"

	"go.viam.com/rdk/logging"
)

// defaultWebAppProxyPort is where the WebApp proxy listens when
// Config.WebAppProxyPort is unset.
const defaultWebAppProxyPort = 18080

// webAppProxyShutdownTimeout bounds how long stop waits for requests in flight
// to finish before closing their connections outright.
const webAppProxyShutdownTimeout = 2 * time.Second

// webAppProxy is a reverse proxy to the controller's WebApp, for when the
// browser can't reach the controller directly (e.g. it sits on a private link
// to the viam-server host) — the counterpart of viam-ufactory-xarm's UFactory
// Studio proxy. It serves plain HTTP locally and has no authentication.
type webAppProxy struct {
	server    *http.Server
	transport *http.Transport
	addr      net.Addr
	logger    logging.Logger

	// cancel ends every request's context, which is what closes the
	// websocket tunnels the proxy hijacked: http.Server.Shutdown neither
	// tracks nor closes hijacked connections.
	cancel context.CancelFunc
}

// startWebAppProxy listens on port and forwards everything, websocket upgrades
// included, to target. The controller's WebApp serves HTTPS with a
// certificate it signed itself, so the upstream certificate isn't verified:
// the proxy only ever talks to the configured controller address.
func startWebAppProxy(ctx context.Context, target *url.URL, port int, logger logging.Logger) (*webAppProxy, error) {
	transport := &http.Transport{
		Proxy: nil,
		//nolint:gosec // the WebApp's certificate is self-signed; see the doc comment.
		TLSClientConfig:     &tls.Config{InsecureSkipVerify: true},
		TLSHandshakeTimeout: 10 * time.Second,
		IdleConnTimeout:     90 * time.Second,
	}
	proxy := &httputil.ReverseProxy{
		Rewrite: func(r *httputil.ProxyRequest) {
			r.SetURL(target)
			r.Out.Host = target.Host
		},
		Transport: transport,
	}
	baseCtx, cancel := context.WithCancel(context.Background())
	server := &http.Server{
		// No host: IPv4 and IPv6 where the host has both, IPv4 alone where
		// IPv6 is disabled (an explicit "[::]" would fail to bind there).
		Addr:              fmt.Sprintf(":%d", port),
		Handler:           proxy,
		ReadHeaderTimeout: 10 * time.Second,
		BaseContext:       func(net.Listener) context.Context { return baseCtx },
	}
	var lc net.ListenConfig
	ln, err := lc.Listen(ctx, "tcp", server.Addr)
	if err != nil {
		cancel()
		return nil, fmt.Errorf("WebApp proxy: failed to listen on port %d: %w", port, err)
	}
	logger.CInfow(ctx, "WebApp proxy listening", "port", port, "target", target.String())
	go func() {
		if err := server.Serve(ln); err != nil && !errors.Is(err, http.ErrServerClosed) {
			logger.Errorw("WebApp proxy stopped", "err", err)
		}
	}()
	return &webAppProxy{server: server, transport: transport, addr: ln.Addr(), logger: logger, cancel: cancel}, nil
}

// validateWebAppURL checks Config.WebAppURL; empty is valid (the default).
func validateWebAppURL(configured string) error {
	_, err := webAppProxyTarget(configured, "")
	return err
}

// webAppProxyTarget resolves Config.WebAppURL, defaulting to https://<host>.
func webAppProxyTarget(configured, host string) (*url.URL, error) {
	if configured == "" {
		if strings.Contains(host, ":") && !strings.HasPrefix(host, "[") {
			host = "[" + host + "]" // an IPv6 address
		}
		return &url.URL{Scheme: "https", Host: host}, nil
	}
	u, err := url.Parse(configured)
	if err != nil || (u.Scheme != "http" && u.Scheme != "https") || u.Host == "" {
		return nil, fmt.Errorf("webapp_url must be an http(s) URL with a host, got %q", configured)
	}
	return u, nil
}

// stop closes the proxy for good: the listener, requests in flight, the
// websocket tunnels it hijacked (which Shutdown alone would leave relaying to
// the controller), and its idle upstream connections.
func (p *webAppProxy) stop() {
	if p == nil {
		return
	}
	p.cancel()
	ctx, cancel := context.WithTimeout(context.Background(), webAppProxyShutdownTimeout)
	defer cancel()
	if err := p.server.Shutdown(ctx); err != nil {
		p.logger.Warnw("WebApp proxy shutdown timed out; closing its connections", "err", err)
		_ = p.server.Close()
	}
	p.transport.CloseIdleConnections()
}
