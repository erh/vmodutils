package vmodutils

import (
	"context"
	"fmt"
	"sync"
	"time"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

var SessionCaptureModel = NamespaceFamily.WithModel("session-capture")

func init() {
	resource.RegisterComponent(
		sensor.API,
		SessionCaptureModel,
		resource.Registration[sensor.Sensor, *SessionCaptureConfig]{
			Constructor: newSessionCapture,
		})
}

type SessionCaptureComponent struct {
	ResourceName       string  `json:"resource_name"`
	Method             string  `json:"method"`
	CaptureFrequencyHz float64 `json:"capture_frequency_hz,omitempty"`
}

func (c *SessionCaptureComponent) frequency() float64 {
	if c.CaptureFrequencyHz > 0 {
		return c.CaptureFrequencyHz
	}
	return 10.0
}

type SessionCaptureConfig struct {
	Components []SessionCaptureComponent `json:"components"`
	TagPrefix  string                    `json:"tag_prefix,omitempty"`
}

func (c *SessionCaptureConfig) Validate(path string) ([]string, []string, error) {
	if len(c.Components) == 0 {
		return nil, nil, fmt.Errorf("need at least one component")
	}
	optionalDeps := make([]string, 0, len(c.Components))
	for i, comp := range c.Components {
		if comp.ResourceName == "" {
			return nil, nil, fmt.Errorf("components[%d]: resource_name is required", i)
		}
		if comp.Method == "" {
			return nil, nil, fmt.Errorf("components[%d]: method is required", i)
		}
		optionalDeps = append(optionalDeps, comp.ResourceName)
	}
	return nil, optionalDeps, nil
}

func newSessionCapture(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (sensor.Sensor, error) {
	config, err := resource.NativeConfig[*SessionCaptureConfig](conf)
	if err != nil {
		return nil, err
	}
	return &sessionCapture{
		name:   conf.ResourceName(),
		config: config,
		logger: logger,
		active: false,
	}, nil
}

type sessionCapture struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	config *SessionCaptureConfig
	logger logging.Logger

	mu      sync.Mutex
	active  bool
	capTags []string
}

func (s *sessionCapture) Name() resource.Name {
	return s.name
}

func (s *sessionCapture) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (s *sessionCapture) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	res := map[string]interface{}{
		"active": s.active,
	}

	if s.active {
		tags := make([]interface{}, len(s.capTags))
		for i, t := range s.capTags {
			tags[i] = t
		}
		overrides := []interface{}{}
		for _, comp := range s.config.Components {
			overrides = append(overrides, map[string]interface{}{
				"resource_name":        comp.ResourceName,
				"method":               comp.Method,
				"capture_frequency_hz": comp.frequency(),
				"tags":                 tags,
			})
		}
		res["overrides"] = overrides
	}

	return res, nil
}

func (s *sessionCapture) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if cmd["start"] == true {
		s.active = true
		now := time.Now().Format("20060102_150405.000")
		s.capTags = []string{s.config.TagPrefix + now}
		return map[string]interface{}{"status": "capturing", "tags": s.capTags}, nil
	}
	if cmd["stop"] == true {
		s.active = false
		s.capTags = []string{}
		return map[string]interface{}{"status": "stopped"}, nil
	}
	return nil, fmt.Errorf("not implemented")
}
