package vmodutils

import (
	"context"
	"fmt"
	"net/http"
	"net/url"
	"strings"

	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

var HTTPRedirectModel = NamespaceFamily.WithModel("http-redirect")

func init() {
	resource.RegisterComponent(
		generic.API,
		HTTPRedirectModel,
		resource.Registration[resource.Resource, *HTTPRedirectConfig]{
			Constructor: newHTTPRedirect,
		})
}

type HTTPRedirectConfig struct {
	// Port to listen on.
	Port int

	// Host to redirect all traffic to. May include a scheme
	// (e.g. "https://example.com"); if no scheme is given, "https" is used.
	Host string
}

func (c *HTTPRedirectConfig) Validate(path string) ([]string, []string, error) {
	if c.Port == 0 {
		return nil, nil, fmt.Errorf("must specify a port")
	}
	if c.Host == "" {
		return nil, nil, fmt.Errorf("must specify a host to redirect to")
	}
	return nil, nil, nil
}

func newHTTPRedirect(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (resource.Resource, error) {
	newConf, err := resource.NativeConfig[*HTTPRedirectConfig](config)
	if err != nil {
		return nil, err
	}

	host := newConf.Host
	if !strings.Contains(host, "://") {
		host = "https://" + host
	}
	base, err := url.Parse(host)
	if err != nil {
		return nil, fmt.Errorf("invalid host %q: %w", newConf.Host, err)
	}

	r := &httpRedirect{
		name:   config.ResourceName(),
		logger: logger,
		base:   base,
	}

	r.server = &http.Server{
		Addr:    fmt.Sprintf(":%d", newConf.Port),
		Handler: r,
	}

	logger.Infof("going to listen on %v and redirect to %v", r.server.Addr, base)
	go func() {
		err := r.server.ListenAndServe()
		if err != nil && err != http.ErrServerClosed {
			logger.Errorf("error ListenAndServe: %v", err)
		}
	}()

	return r, nil
}

type httpRedirect struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger

	base   *url.URL
	server *http.Server
}

func (r *httpRedirect) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	target := *r.base
	target.Path = req.URL.Path
	target.RawQuery = req.URL.RawQuery

	r.logger.Debugf("redirecting %s %s -> %s", req.Method, req.URL, target.String())
	http.Redirect(w, req, target.String(), http.StatusFound)
}

func (r *httpRedirect) Name() resource.Name {
	return r.name
}

func (r *httpRedirect) Close(ctx context.Context) error {
	return r.server.Close()
}

func (r *httpRedirect) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (r *httpRedirect) Status(ctx context.Context) (map[string]interface{}, error) {
	return nil, nil
}
