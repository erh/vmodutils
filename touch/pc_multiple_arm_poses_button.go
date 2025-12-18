package touch

import (
	"context"
	"errors"
	"fmt"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/button"
	"go.viam.com/rdk/components/camera"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"

	"github.com/erh/vmodutils"
)

var MultipleArmPosesButtonModel = vmodutils.NamespaceFamily.WithModel("pc-multiple-arm-poses-button")

func init() {
	resource.RegisterComponent(
		button.API,
		MultipleArmPosesButtonModel,
		resource.Registration[button.Button, *MultipleArmPosesButtonConfig]{
			Constructor: newMultipleArmPosesButton,
		})
}

type MultipleArmPosesButtonConfig struct {
	Src                            string
	SleepSeconds                   float64 `json:"sleep_seconds"`
	Positions                      []string
	MultiPositionSwitch            string `json:"multi_position_switch"`
	PCMultipleArmPosesButtonCamera string `json:"pc_multiple_arm_poses_button_camera"`
}

func (c *MultipleArmPosesButtonConfig) sleepTime() time.Duration {
	if c.SleepSeconds <= 0 {
		return time.Second
	}
	return time.Duration(c.SleepSeconds * float64(time.Second))
}

func (c *MultipleArmPosesButtonConfig) Validate(path string) ([]string, []string, error) {
	var deps []string
	if c.Src == "" {
		return nil, nil, fmt.Errorf("need a src camera")
	}
	deps = append(deps, c.Src)

	if c.PCMultipleArmPosesButtonCamera == "" {
		return nil, nil, fmt.Errorf("need pc_merge_button_camera")
	}
	deps = append(deps, c.PCMultipleArmPosesButtonCamera)

	if len(c.Positions) > 0 && c.MultiPositionSwitch != "" {
		return nil, nil, fmt.Errorf("only one of: [positions, multi_position_switch] may be set")
	}

	if len(c.Positions) == 0 && c.MultiPositionSwitch == "" {
		return nil, nil, fmt.Errorf("at least one of: [positions, multi_position_switch must be set")
	}

	if c.MultiPositionSwitch != "" {
		return append(deps, c.MultiPositionSwitch), nil, nil
	}

	return append(deps, c.Positions...), nil, nil
}

func newMultipleArmPosesButton(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (button.Button, error) {
	newConf, err := resource.NativeConfig[*MultipleArmPosesButtonConfig](config)
	if err != nil {
		return nil, err
	}

	mapb := &MultipleArmPosesButton{
		name: config.ResourceName(),
		cfg:  newConf,
	}

	mapb.src, err = camera.FromProvider(deps, newConf.Src)
	if err != nil {
		return nil, err
	}

	for _, p := range newConf.Positions {
		s, err := toggleswitch.FromProvider(deps, p)
		if err != nil {
			return nil, err
		}
		mapb.positions = append(mapb.positions, s)
	}
	mapb.fsSvc, err = framesystem.FromDependencies(deps)
	if err != nil {
		return nil, err
	}
	if newConf.MultiPositionSwitch != "" {
		mapb.multiPositionSwitch, err = toggleswitch.FromProvider(deps, newConf.MultiPositionSwitch)
		if err != nil {
			return nil, err
		}
		nPos, _, err := mapb.multiPositionSwitch.GetNumberOfPositions(ctx, nil)
		if err != nil {
			return nil, errors.Join(errors.New("failled calling GetNumberOfPositions on the multi_position_switch resource"), err)
		}
		if nPos == 0 {
			return nil, errors.New("multi_position_switch has 0 positions")
		}
	}

	cam, err := camera.FromProvider(deps, newConf.PCMultipleArmPosesButtonCamera)
	if err != nil {
		return nil, err
	}

	multipleArmPosesButtonCameraModelRegistryMu.Lock()
	mapbc, ok := multipleArmPosesButtonCameraModelRegistry[cam.Name().String()]
	multipleArmPosesButtonCameraModelRegistryMu.Unlock()

	if !ok {
		return nil, errors.New("pc_multiple_arm_poses_button_camera must have the model erh:vmodutils:pc-multiple-arm-poses-button-camera")
	}

	mapb.pcMultipleArmPosesButtonCamera = mapbc
	mapb.pcMultipleArmPosesButtonCamera.ClearPointCloud()

	return mapb, nil
}

type MultipleArmPosesButton struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name resource.Name
	cfg  *MultipleArmPosesButtonConfig

	fsSvc framesystem.Service

	src                            camera.Camera
	positions                      []toggleswitch.Switch
	multiPositionSwitch            toggleswitch.Switch
	pcMultipleArmPosesButtonCamera *MultipleArmPosesButtonCamera
	executing                      atomic.Bool
}

func (mapb *MultipleArmPosesButton) Name() resource.Name {
	return mapb.name
}

func (mapb *MultipleArmPosesButton) Push(ctx context.Context, extra map[string]any) error {
	if !mapb.executing.CompareAndSwap(false, true) {
		return errors.New("button is currently executing")
	}
	defer mapb.executing.Store(false)
	mapb.pcMultipleArmPosesButtonCamera.ClearPointCloud()
	if len(mapb.positions) > 0 {
		pc, err := GetMergedPointCloudFromPositions(ctx, mapb.positions, mapb.cfg.sleepTime(), mapb.src, extra, mapb.fsSvc)
		if err != nil {
			return err
		}
		mapb.pcMultipleArmPosesButtonCamera.SetPointCloud(pc)
		return nil
	}

	pc, err := GetMergedPointCloudFromMultiPositionSwitch(ctx, mapb.multiPositionSwitch, mapb.cfg.sleepTime(), mapb.src, extra, mapb.fsSvc)
	if err != nil {
		return err
	}
	mapb.pcMultipleArmPosesButtonCamera.SetPointCloud(pc)
	return nil
}

func (mapb *MultipleArmPosesButton) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	return nil, nil
}
