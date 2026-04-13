package touch

import (
	"context"
	"fmt"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"

	"github.com/erh/vmodutils"
)

var ObstacleOpenBoxModel = vmodutils.NamespaceFamily.WithModel("obstacle-open-box")

func init() {
	resource.RegisterComponent(
		gripper.API,
		ObstacleOpenBoxModel,
		resource.Registration[gripper.Gripper, *ObstacleOpenBoxConfig]{
			Constructor: newObstacleOpenBox,
		})
}

type ObstacleOpenBoxConfig struct {
	Length, Width, Height float64
	Thickness             float64

	ToMove string `json:"to_move"`
	Motion string
	Offset float64
}

func (c *ObstacleOpenBoxConfig) motion() string {
	if c.Motion == "" {
		return "builtin"
	}
	return c.Motion
}

func (c *ObstacleOpenBoxConfig) Validate(path string) ([]string, []string, error) {
	if c.Length == 0 || c.Width == 0 || c.Height == 0 {
		return nil, nil, fmt.Errorf("need length, width, and height")
	}

	deps := []string{}

	if c.ToMove != "" {
		deps = append(deps, c.ToMove)
		deps = append(deps, motion.Named(c.motion()).String())
	}

	return deps, nil, nil
}

func (c *ObstacleOpenBoxConfig) offset() float64 {
	if c.Offset == 0 {
		return 50
	}
	return c.Offset
}

func (c *ObstacleOpenBoxConfig) thickness() float64 {
	if c.Thickness <= 0 {
		return 1
	}
	return c.Thickness
}

func (c *ObstacleOpenBoxConfig) Geometries(name string) ([]spatialmath.Geometry, error) {
	bottom := c.Height / -2

	floor, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{0, 0, bottom}), r3.Vector{c.Length, c.Width, c.thickness()}, name+"-floor")
	if err != nil {
		return nil, err
	}

	front, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{c.Length / 2, 0, 0}), r3.Vector{c.thickness(), c.Width, c.Height}, name+"-front")
	if err != nil {
		return nil, err
	}
	back, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{c.Length / -2, 0, 0}), r3.Vector{c.thickness(), c.Width, c.Height}, name+"-back")
	if err != nil {
		return nil, err
	}

	left, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{0, c.Width / 2, 0}), r3.Vector{c.Length, c.thickness(), c.Height}, name+"-left")
	if err != nil {
		return nil, err
	}
	right, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{0, c.Width / -2, 0}), r3.Vector{c.Length, c.thickness(), c.Height}, name+"-right")
	if err != nil {
		return nil, err
	}

	return []spatialmath.Geometry{floor, front, back, left, right}, nil
}

func newObstacleOpenBox(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	newConf, err := resource.NativeConfig[*ObstacleOpenBoxConfig](config)
	if err != nil {
		return nil, err
	}

	gs, err := newConf.Geometries(config.ResourceName().ShortName())
	if err != nil {
		return nil, err
	}

	o := &ObstacleOpenBox{
		name:      config.ResourceName(),
		logger:    logger,
		obstacles: gs,
		conf:      newConf,
	}

	o.mf, err = gripper.MakeModel(config.ResourceName().ShortName(), gs)
	if err != nil {
		return nil, err
	}

	if newConf.ToMove != "" {
		var ok bool
		o.toMove, ok = vmodutils.FindDep(deps, newConf.ToMove)
		if !ok {
			return nil, fmt.Errorf("cannot find ToMove [%s]", newConf.ToMove)
		}
		o.motion, err = motion.FromProvider(deps, newConf.motion())
		if err != nil {
			return nil, err
		}
	}

	return o, nil
}

type ObstacleOpenBox struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	mf referenceframe.Model

	name      resource.Name
	logger    logging.Logger
	conf      *ObstacleOpenBoxConfig
	obstacles []spatialmath.Geometry

	toMove resource.Resource
	motion motion.Service
}

func (o *ObstacleOpenBox) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	if o.toMove == nil {
		return false, fmt.Errorf("obstacle opem box has no to_move specified")
	}

	p, err := o.motion.GetPose(ctx, o.name.ShortName(), "world", nil, nil)
	if err != nil {
		return false, err
	}

	p2, err := o.motion.GetPose(ctx, o.conf.ToMove, "world", nil, nil)
	if err != nil {
		return false, err
	}

	p = referenceframe.NewPoseInFrame("world",
		spatialmath.NewPose(
			r3.Vector{
				X: p.Pose().Point().X,
				Y: p.Pose().Point().Y,
				Z: p.Pose().Point().Z + o.conf.offset(),
			},
			&spatialmath.OrientationVectorDegrees{
				OZ:    -1,
				Theta: p2.Pose().Orientation().OrientationVectorDegrees().Theta,
			},
		),
	)

	o.logger.Infof("want to move %s to %v", o.toMove.Name().ShortName(), p)

	_, err = o.motion.Move(ctx,
		motion.MoveReq{
			ComponentName: o.conf.ToMove,
			Destination:   p,
			Constraints: &motionplan.Constraints{
				OrientationConstraint: []motionplan.OrientationConstraint{{180}},
			},
		})
	if err != nil {
		return false, fmt.Errorf("cannot move to %v because: %w", p, err)
	}

	return false, nil
}

func (o *ObstacleOpenBox) Open(ctx context.Context, extra map[string]interface{}) error {
	return fmt.Errorf("obstacle can't open")
}

func (o *ObstacleOpenBox) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return o.obstacles, nil
}

func (o *ObstacleOpenBox) Name() resource.Name {
	return o.name
}

func (o *ObstacleOpenBox) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (o *ObstacleOpenBox) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (o *ObstacleOpenBox) IsMoving(context.Context) (bool, error) {
	return false, nil
}

func (o *ObstacleOpenBox) IsHoldingSomething(ctx context.Context, extra map[string]interface{}) (gripper.HoldingStatus, error) {
	return gripper.HoldingStatus{false, nil}, nil
}

func (o *ObstacleOpenBox) Stop(context.Context, map[string]interface{}) error {
	return nil
}

func (g *ObstacleOpenBox) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return []referenceframe.Input{}, nil
}

func (g *ObstacleOpenBox) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return nil
}

func (g *ObstacleOpenBox) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.mf, fmt.Errorf("for now Kinematics errors to work around bug?")
}
