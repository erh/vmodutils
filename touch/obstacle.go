package touch

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"github.com/erh/vmodutils"
)

var ObstacleModel = vmodutils.NamespaceFamily.WithModel("obstacle")

func init() {
	resource.RegisterComponent(
		gripper.API,
		ObstacleModel,
		resource.Registration[gripper.Gripper, *ObstacleConfig]{
			Constructor: newObstacle,
		})
}

type ObstacleConfig struct {
	Geometries []spatialmath.GeometryConfig
}

func (c *ObstacleConfig) ParseGeometries() ([]spatialmath.Geometry, error) {
	gs := []spatialmath.Geometry{}

	for _, gc := range c.Geometries {
		g, err := gc.ParseConfig()
		if err != nil {
			return nil, err
		}
		gs = append(gs, g)
	}

	return gs, nil
}

func (c *ObstacleConfig) Validate(path string) ([]string, []string, error) {
	_, err := c.ParseGeometries()
	return nil, nil, err
}

func newObstacle(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	newConf, err := resource.NativeConfig[*ObstacleConfig](config)
	if err != nil {
		return nil, err
	}

	gs, err := newConf.ParseGeometries()
	if err != nil {
		return nil, err
	}

	o := &Obstacle{
		name:      config.ResourceName(),
		obstacles: gs,
	}

	o.mf, err = gripper.MakeModel(config.ResourceName().ShortName(), gs)
	if err != nil {
		return nil, err
	}
	return o, nil
}

type Obstacle struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	mf referenceframe.Model

	name      resource.Name
	obstacles []spatialmath.Geometry
}

func (o *Obstacle) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	return false, fmt.Errorf("obstacle can't grab")
}

func (o *Obstacle) Open(ctx context.Context, extra map[string]interface{}) error {
	return fmt.Errorf("obstacle can't open")
}

func (o *Obstacle) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return o.obstacles, nil
}

func (o *Obstacle) Name() resource.Name {
	return o.name
}

func (o *Obstacle) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (o *Obstacle) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (o *Obstacle) IsMoving(context.Context) (bool, error) {
	return false, nil
}

func (o *Obstacle) IsHoldingSomething(ctx context.Context, extra map[string]interface{}) (gripper.HoldingStatus, error) {
	return gripper.HoldingStatus{false, nil}, nil
}

func (o *Obstacle) Stop(context.Context, map[string]interface{}) error {
	return nil
}

func (g *Obstacle) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return []referenceframe.Input{}, nil
}

func (g *Obstacle) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return nil
}

func (g *Obstacle) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return nil, fmt.Errorf("for now Kinematics errors to work around bug?")
}
