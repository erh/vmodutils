package touch

import (
	"context"
	"encoding/json"
	"fmt"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/arm"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"

	"github.com/erh/vmodutils"
)

var ArmPositionSaverModel = vmodutils.NamespaceFamily.WithModel("arm-position-saver")

func init() {
	resource.RegisterComponent(
		toggleswitch.API,
		ArmPositionSaverModel,
		resource.Registration[toggleswitch.Switch, *ArmPositionSaverConfig]{
			Constructor: newArmPositionSaver,
		})
}

type ArmPositionSaverConfig struct {
	Arm            string                               `json:"arm,omitempty"`
	Joints         []float64                            `json:"joints,omitempty"`
	Motion         string                               `json:"motion,omitempty"`
	Point          r3.Vector                            `json:"point,omitzero"`
	Orientation    spatialmath.OrientationVectorDegrees `json:"orientation,omitzero"`
	VisionServices []string                             `json:"vision_services,omitempty"`
	Extra          map[string]interface{}               `json:"extra,omitempty"`
}

func (c *ArmPositionSaverConfig) Validate(path string) ([]string, []string, error) {
	if c.Arm == "" {
		return nil, nil, fmt.Errorf("no arm specificed")
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

func newArmPositionSaver(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (toggleswitch.Switch, error) {
	newConf, err := resource.NativeConfig[*ArmPositionSaverConfig](config)
	if err != nil {
		return nil, err
	}

	arm, err := arm.FromProvider(deps, newConf.Arm)
	if err != nil {
		return nil, err
	}

	aps := &ArmPositionSaver{
		name:   config.ResourceName(),
		cfg:    newConf,
		logger: logger,
		arm:    arm,
	}

	if newConf.Motion != "" {
		aps.motion, err = motion.FromProvider(deps, newConf.Motion)
		if err != nil {
			return nil, err
		}
	}

	if len(newConf.VisionServices) > 0 {
		for _, name := range newConf.VisionServices {
			v, err := vision.FromProvider(deps, name)
			if err != nil {
				return nil, err
			}
			aps.visionServices = append(aps.visionServices, v)
		}
	}

	aps.fsSvc, err = framesystem.FromDependencies(deps)
	if err != nil {
		return nil, err
	}

	return aps, nil
}

type ArmPositionSaver struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	cfg    *ArmPositionSaverConfig
	logger logging.Logger

	arm            arm.Arm
	motion         motion.Service
	visionServices []vision.Service
	fsSvc          framesystem.Service
}

func (aps *ArmPositionSaver) Name() resource.Name {
	return aps.name
}

func (aps *ArmPositionSaver) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if cmd["cfg"] == true {
		jsonData, err := json.Marshal(aps.cfg)
		if err != nil {
			return nil, err
		}

		return map[string]interface{}{
			"joints":      aps.cfg.Joints,
			"point":       aps.cfg.Point,
			"orientation": aps.cfg.Orientation,
			"as_json":     string(jsonData),
		}, nil
	}
	return nil, fmt.Errorf("unknown command %v", cmd)
}

func (aps *ArmPositionSaver) SetPosition(ctx context.Context, position uint32, extra map[string]interface{}) error {
	if position == 0 {
		return nil
	}

	if position == 1 {
		return aps.saveCurrentPosition(ctx)
	}

	if position == 2 {
		return aps.goToSavePosition(ctx)
	}

	return fmt.Errorf("bad position: %d", position)
}

func (aps *ArmPositionSaver) GetPosition(ctx context.Context, extra map[string]interface{}) (uint32, error) {
	return 0, nil
}

func (aps *ArmPositionSaver) GetNumberOfPositions(ctx context.Context, extra map[string]interface{}) (uint32, []string, error) {
	return 3, []string{"idle", "update config", "go to"}, nil
}

func (aps *ArmPositionSaver) saveCurrentPosition(ctx context.Context) error {
	newConfig := utils.AttributeMap{
		"arm":    aps.cfg.Arm,
		"motion": aps.cfg.Motion,
	}

	if aps.cfg.Motion == "" {
		inputs, err := aps.arm.JointPositions(ctx, nil)
		if err != nil {
			return err
		}

		newConfig["joints"] = inputs
	} else {
		p, err := aps.fsSvc.GetPose(ctx, aps.cfg.Arm, "world", nil, nil)
		if err != nil {
			return err
		}
		newConfig["point"] = p.Pose().Point()
		newConfig["orientation"] = p.Pose().Orientation().OrientationVectorDegrees()
	}

	return vmodutils.UpdateComponentCloudAttributesFromModuleEnv(ctx, aps.name, newConfig, aps.logger)
}

func (aps *ArmPositionSaver) goToSavePosition(ctx context.Context) error {
	if len(aps.cfg.Joints) > 0 {
		if aps.motion != nil {
			goToPositionUsingJointToJointMotion(ctx, aps.cfg.Joints, aps.arm.Name().Name, aps.motion, aps.visionServices, aps.cfg.Extra, aps.logger)
		} else {
			return goToPositionUsingMoveToJointPositions(ctx, aps.cfg.Joints, aps.arm, aps.cfg.Extra, aps.logger)
		}
	}

	if aps.motion != nil {
		return goToPositionUsingCartesianMotion(ctx, aps.cfg.Point, aps.cfg.Orientation, aps.motion, aps.visionServices, aps.fsSvc, aps.arm.Name().Name, aps.cfg.Extra, aps.logger)
	}

	return fmt.Errorf("need to configure where to go")
}
