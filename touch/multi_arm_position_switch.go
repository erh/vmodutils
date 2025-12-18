package touch

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/arm"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/vision"

	"github.com/erh/vmodutils"
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
	Arm            string         `json:"arm,omitempty"`
	JointsList     [][]float64    `json:"joints_list,omitempty"`
	Motion         string         `json:"motion,omitempty"`
	VisionServices []string       `json:"vision_services,omitempty"`
	Extra          map[string]any `json:"extra,omitempty"`
}

func (c *MultiArmPositionSwitchConfig) Validate(path string) ([]string, []string, error) {
	if c.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}

	deps := []string{c.Arm}

	if c.Motion != "" {
		if c.Motion == "builtin" {
			deps = append(deps, motion.Named("builtin").String())
		} else {
			deps = append(deps, c.Motion)
		}
	}

	deps = append(deps, c.VisionServices...)

	if c.Extra != nil && c.Extra[extraParamsKeyGoalState] != nil {
		return nil, nil, ErrCannotSpecifyGoalStateInExtra
	}

	return deps, nil, nil
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
}

func (maps *MultiArmPositionSwitch) Name() resource.Name {
	return maps.name
}

func (maps *MultiArmPositionSwitch) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, resource.ErrDoUnimplemented
}

func (maps *MultiArmPositionSwitch) SetPosition(ctx context.Context, position uint32, extra map[string]interface{}) error {
	if position > uint32(len(maps.cfg.JointsList))-1 {
		return fmt.Errorf("requested position %d is greater than highest possible position %d", position, len(maps.cfg.JointsList)-1)
	}
	return maps.goToPosition(ctx, maps.cfg.JointsList[position])
}

func (maps *MultiArmPositionSwitch) GetPosition(ctx context.Context, extra map[string]interface{}) (uint32, error) {
	return 0, nil
}

func (maps *MultiArmPositionSwitch) GetNumberOfPositions(ctx context.Context, extra map[string]interface{}) (uint32, []string, error) {
	var positionStrs []string
	for i := range maps.cfg.JointsList {
		positionStrs = append(positionStrs, fmt.Sprintf("go to %d", i))
	}
	return uint32(len(maps.cfg.JointsList)), positionStrs, nil
}

func (maps *MultiArmPositionSwitch) goToPosition(ctx context.Context, joints []float64) error {
	if maps.motion != nil {
		return goToPositionUsingJointToJointMotion(ctx, joints, maps.arm.Name().Name, maps.motion, maps.visionServices, maps.cfg.Extra, maps.logger)
	}
	return goToPositionUsingMoveToJointPositions(ctx, joints, maps.arm, maps.cfg.Extra, maps.logger)
}
