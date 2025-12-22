package touch

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/utils/trace"

	"github.com/erh/vmodutils"
	"github.com/erh/vmodutils/file_utils"
)

const extraParamsKeyGoalState = "goal_state"

var MultiArmPositionSwitchModel = vmodutils.NamespaceFamily.WithModel("multi-arm-position-switch")

func init() {
	resource.RegisterComponent(
		toggleswitch.API,
		MultiArmPositionSwitchModel,
		resource.Registration[toggleswitch.Switch, *MultiArmPositionSwitchConfig]{
			Constructor: newMultiArmPositionSwitch,
		})
}

type MultiArmPositionSwitchConfig struct {
	Arm                          string         `json:"arm,omitempty"`
	JointsList                   [][]float64    `json:"joints_list,omitempty"`
	Motion                       string         `json:"motion,omitempty"`
	VisionServices               []string       `json:"vision_services,omitempty"`
	Extra                        map[string]any `json:"extra,omitempty"`
	WriteFilesToCaptureDirectory bool           `json:"write_files_to_capture_directory,omitempty"`
}

func (c *MultiArmPositionSwitchConfig) Validate(path string) ([]string, []string, error) {
	reqDeps := []string{}

	if c.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	reqDeps = append(reqDeps, c.Arm)

	if c.Motion != "" {
		if c.Motion == "builtin" {
			reqDeps = append(reqDeps, motion.Named("builtin").String())
		} else {
			reqDeps = append(reqDeps, c.Motion)
		}
	}

	reqDeps = append(reqDeps, c.VisionServices...)

	if len(c.JointsList) == 0 {
		return nil, nil, ErrMustSpecifyAtLeastOneJointPosition
	}

	if c.Extra != nil && c.Extra[extraParamsKeyGoalState] != nil {
		return nil, nil, ErrCannotSpecifyGoalStateInExtra
	}

	return reqDeps, nil, nil
}

func newMultiArmPositionSwitch(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (toggleswitch.Switch, error) {
	newConf, err := resource.NativeConfig[*MultiArmPositionSwitchConfig](config)
	if err != nil {
		return nil, err
	}

	arm, err := arm.FromProvider(deps, newConf.Arm)
	if err != nil {
		return nil, err
	}

	maps := &MultiArmPositionSwitch{
		name:   config.ResourceName(),
		cfg:    newConf,
		logger: logger,
		arm:    arm,
	}

	if newConf.Motion != "" {
		maps.motion, err = motion.FromProvider(deps, newConf.Motion)
		if err != nil {
			return nil, err
		}
	}

	for _, name := range newConf.VisionServices {
		v, err := vision.FromProvider(deps, name)
		if err != nil {
			return nil, err
		}
		maps.visionServices = append(maps.visionServices, v)
	}

	maps.fsSvc, err = framesystem.FromDependencies(deps)
	if err != nil {
		return nil, err
	}

	return maps, nil
}

type MultiArmPositionSwitch struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	cfg    *MultiArmPositionSwitchConfig
	logger logging.Logger

	arm            arm.Arm
	motion         motion.Service
	visionServices []vision.Service
	fsSvc          framesystem.Service

	// 'mu' protects access to 'position'
	mu       sync.Mutex
	position uint32

	// 'executing' ensures only one goToPosition call is active at a time
	executing atomic.Bool
}

func (maps *MultiArmPositionSwitch) Name() resource.Name {
	return maps.name
}

func (maps *MultiArmPositionSwitch) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, resource.ErrDoUnimplemented
}

func (maps *MultiArmPositionSwitch) updatePosition(position uint32) {
	maps.mu.Lock()
	defer maps.mu.Unlock()
	maps.position = position
}

func (maps *MultiArmPositionSwitch) SetPosition(ctx context.Context, position uint32, extra map[string]interface{}) error {
	if position > uint32(len(maps.cfg.JointsList))-1 {
		return fmt.Errorf("requested position %d is greater than highest possible position %d", position, len(maps.cfg.JointsList)-1)
	}

	err := maps.goToPosition(ctx, position)
	if err != nil {
		return err
	}

	return nil
}

func (maps *MultiArmPositionSwitch) GetPosition(ctx context.Context, extra map[string]interface{}) (uint32, error) {
	maps.mu.Lock()
	defer maps.mu.Unlock()
	return maps.position, nil
}

func (maps *MultiArmPositionSwitch) GetNumberOfPositions(ctx context.Context, extra map[string]interface{}) (uint32, []string, error) {
	var positionStrs []string
	for i := range maps.cfg.JointsList {
		positionStrs = append(positionStrs, fmt.Sprintf("go to %d", i))
	}
	return uint32(len(maps.cfg.JointsList)), positionStrs, nil
}

func (maps *MultiArmPositionSwitch) goToPosition(ctx context.Context, position uint32) error {
	if !maps.executing.CompareAndSwap(false, true) {
		return errors.New("switch is currently executing")
	}
	defer maps.executing.Store(false)
	if maps.cfg.WriteFilesToCaptureDirectory {
		traceID := ""
		if span := trace.FromContext(ctx); span != nil {
			traceID = span.SpanContext().TraceID().String()
		}
		dirPath := file_utils.GetPathInCaptureDir(traceID)

		if traceID == "" {
			maps.logger.Warnf("no traceID set, writing resource config file for %s without traceID in capture directory", maps.name.Name)
		}

		fileName := fmt.Sprintf("%s_%s", maps.name.Name, "config.json")
		file_utils.SaveJsonFile(maps.cfg, dirPath, fileName, time.Now())
	}

	maps.updatePosition(position)

	joints := maps.cfg.JointsList[position]

	if maps.motion != nil {
		return goToPositionUsingJointToJointMotion(ctx, joints, maps.arm.Name().Name, maps.motion, maps.visionServices, maps.cfg.Extra, maps.logger)
	}
	return goToPositionUsingMoveToJointPositions(ctx, joints, maps.arm, maps.cfg.Extra, maps.logger)
}
