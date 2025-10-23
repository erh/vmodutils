package touch

import (
	"context"
	"encoding/json"
	"fmt"
	"maps"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/arm"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
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

	if len(c.VisionServices) > 0 {
		deps = append(deps, c.VisionServices...)
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

	aps.fsSvc, err = resource.FromProvider[framesystem.Service](deps, framesystem.PublicServiceName)
	if err != nil {
		return nil, err
	}

	aps.fs, err = framesystem.NewFromService(ctx, aps.fsSvc, nil)
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

	fsSvc          framesystem.Service
	fs             *referenceframe.FrameSystem
	arm            arm.Arm
	motion         motion.Service
	visionServices []vision.Service
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

		newConfig["joints"] = referenceframe.InputsToFloats(inputs)
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
	if len(aps.cfg.Joints) > 0 && aps.motion != nil {
		// if you specify both joints & a motion service
		// move from joint to joint avoiding obstacles
		worldState := referenceframe.NewEmptyWorldState()
		if len(aps.visionServices) > 0 {
			var obstacles []*referenceframe.GeometriesInFrame
			for _, v := range aps.visionServices {
				// add obstacles to the world state from the vision service
				vizs, err := v.GetObjectPointClouds(ctx, "", nil)
				if err != nil {
					return err
				}
				for _, viz := range vizs {
					if viz.Geometry != nil {
						gif := referenceframe.NewGeometriesInFrame(v.Name().Name, []spatialmath.Geometry{viz.Geometry})
						obstacles = append(obstacles, gif)
					}
				}
			}
			var err error
			worldState, err = referenceframe.NewWorldState(obstacles, []*referenceframe.LinkInFrame{})
			if err != nil {
				return fmt.Errorf("couldn't create world state: %w", err)
			}
		}

		extra := map[string]any{}
		maps.Copy(extra, aps.cfg.Extra)

		// specify start state to ensure the arm is in the joint configuration
		// fsSvc.CurrentInputs lead us to believe it was in
		currentInputs, err := aps.fsSvc.CurrentInputs(ctx)
		if err != nil {
			return err
		}
		if _, ok := currentInputs[aps.arm.Name().Name]; !ok {
			return fmt.Errorf("expected arm %s to be returned from framesystem service CurrentInputs", aps.arm.Name().Name)
		}
		extra["start_state"] = serialize(currentInputs)

		// goal state expressed in joint positions so we are confident the arm is not blocking the
		// view of the imaging target
		goalFrameSystemInputs := currentInputs
		goalFrameSystemInputs[aps.arm.Name().Name] = referenceframe.FloatsToInputs(aps.cfg.Joints)
		extra["goal_state"] = serialize(goalFrameSystemInputs)
		_, err = aps.motion.Move(ctx, motion.MoveReq{
			ComponentName: aps.cfg.Arm,
			WorldState:    worldState,
			Extra:         extra,
		})
		return err
	}

	if len(aps.cfg.Joints) > 0 {
		return aps.arm.MoveToJointPositions(ctx, referenceframe.FloatsToInputs(aps.cfg.Joints), aps.cfg.Extra)
	}

	if aps.motion != nil {
		current, err := aps.fsSvc.GetPose(ctx, aps.cfg.Arm, "world", nil, nil)
		if err != nil {
			return err
		}

		linearDelta := current.Pose().Point().Distance(aps.cfg.Point)
		orientationDelta := spatialmath.QuatToR3AA(spatialmath.OrientationBetween(current.Pose().Orientation(), &aps.cfg.Orientation).Quaternion()).Norm2()

		aps.logger.Debugf("goToSavePosition linearDelta: %v orientationDelta: %v", linearDelta, orientationDelta)
		if linearDelta < .1 && orientationDelta < .01 {
			aps.logger.Debugf("close enough, not moving - linearDelta: %v orientationDelta: %v", linearDelta, orientationDelta)
			return nil
		}

		pif := referenceframe.NewPoseInFrame(
			"world",
			spatialmath.NewPose(aps.cfg.Point, &aps.cfg.Orientation),
		)

		done, err := aps.motion.Move(
			ctx,
			motion.MoveReq{
				ComponentName: aps.cfg.Arm,
				Destination:   pif,
				WorldState:    nil,
				Extra:         aps.cfg.Extra,
			},
		)
		if err != nil {
			return err
		}
		if !done {
			return fmt.Errorf("move didn't finish")
		}
		return nil
	}

	return fmt.Errorf("need to configure where to go")
}

func serialize(inputs referenceframe.FrameSystemInputs) map[string]any {
	m := map[string]interface{}{}
	confMap := map[string]interface{}{}
	for fName, conf := range inputs {
		confMap[fName] = referenceframe.InputsToFloats(conf)
	}
	m["configuration"] = confMap
	return m
}
