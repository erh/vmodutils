package touch

import (
	"context"
	"fmt"
	"image"
	"image/color"
	"math"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
)

func PCFindHighestInRegion(pc pointcloud.PointCloud, box image.Rectangle) r3.Vector {

	best := r3.Vector{Z: -100000}

	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if p.Z > best.Z {
			if p.X >= float64(box.Min.X) && p.Y >= float64(box.Min.Y) {
				if p.X <= float64(box.Max.X) && p.Y <= float64(box.Max.Y) {
					best = p
				}
			}
		}

		return true
	})

	return best
}

func PCFindLowestInRegion(pc pointcloud.PointCloud, box image.Rectangle) r3.Vector {

	best := r3.Vector{Z: 100000}

	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if p.Z < best.Z {
			if p.X >= float64(box.Min.X) && p.Y >= float64(box.Min.Y) {
				if p.X <= float64(box.Max.X) && p.Y <= float64(box.Max.Y) {
					best = p
				}
			}
		}

		return true
	})

	return best
}

func PrepBoundingRectForSearch() *image.Rectangle {
	return &image.Rectangle{
		Min: image.Point{1000000, 1000000},
		Max: image.Point{-1000000, -1000000},
	}
}

func BoundingRectMinMax(r *image.Rectangle, p r3.Vector) {
	x := int(p.X)
	y := int(p.Y)

	if x < r.Min.X {
		r.Min.X = x
	}
	if x > r.Max.X {
		r.Max.X = x
	}
	if y < r.Min.Y {
		r.Min.Y = y
	}
	if y > r.Max.Y {
		r.Max.Y = y
	}

}

func InBox(pt, min, max r3.Vector) bool {
	if pt.X < min.X || pt.X > max.X {
		return false
	}

	if pt.Y < min.Y || pt.Y > max.Y {
		return false
	}

	if pt.Z < min.Z || pt.Z > max.Z {
		return false
	}

	return true
}

func PCCrop(pc pointcloud.PointCloud, min, max r3.Vector) pointcloud.PointCloud {
	return PCCropWithColor(pc, min, max, nil)
}

type ColorFilter struct {
	Color    color.RGBA
	Distance float64
}

func EuclideanRGB(c1, c2 color.Color) float64 {
	r1, g1, b1, _ := c1.RGBA()
	r2, g2, b2, _ := c2.RGBA()

	// RGBA() returns uint32 in range [0, 65535], convert to [0, 255]
	r1, g1, b1 = r1>>8, g1>>8, b1>>8
	r2, g2, b2 = r2>>8, g2>>8, b2>>8

	dr := int(r1) - int(r2)
	dg := int(g1) - int(g2)
	db := int(b1) - int(b2)

	return math.Sqrt(float64(dr*dr + dg*dg + db*db))
}

func PCCropWithColor(pc pointcloud.PointCloud, min, max r3.Vector, colorFilters []ColorFilter) pointcloud.PointCloud {

	fixed := pointcloud.NewBasicEmpty()

	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if !InBox(p, min, max) {
			return true
		}

		for _, cf := range colorFilters {
			dis := EuclideanRGB(cf.Color, d.Color())
			if dis > cf.Distance {
				return true
			}
		}

		fixed.Set(p, d)
		return true
	})

	return fixed
}

func PCToImage(pc pointcloud.PointCloud) image.Image {

	md := pc.MetaData()

	r := image.Rect(
		int(math.Floor(md.MinX)),
		int(math.Floor(md.MinY)),
		int(math.Ceil(md.MaxX)),
		int(math.Ceil(md.MaxY)),
	)

	xScale := 0
	yScale := 0

	if r.Min.X < 0 {
		xScale = -1 * r.Min.X
		r.Min.X += xScale
		r.Max.X += xScale
	}

	if r.Min.Y < 0 {
		yScale = -1 * r.Min.Y
		r.Min.Y += yScale
		r.Max.Y += yScale
	}

	if r.Max.X <= 0 {
		r.Max.X = 1
	}

	if r.Max.Y <= 0 {
		r.Max.Y = 1
	}

	img := image.NewRGBA(r)

	bestZ := make([]float64, (1+r.Max.X)*(1+r.Max.Y)) //map[int]float64{}
	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		x := int(p.X) + xScale
		y := int(p.Y) + yScale

		key := (y * r.Max.X) + x
		oldZ := bestZ[key]
		ok := oldZ != 0
		if ok {
			if p.Z < oldZ {
				return true
			}
		}

		img.Set(x, y, d.Color())

		if p.Z == 0 {
			p.Z = .0001
		}
		bestZ[key] = p.Z

		return true
	})

	return img
}

// GetApproachPoint
func GetApproachPoint(p r3.Vector, deltaLinear float64, o *spatialmath.OrientationVectorDegrees) r3.Vector {
	d := math.Pow((o.OX*o.OX)+(o.OY*o.OY)+(o.OZ*o.OZ), .5)

	xLinear := (o.OX * deltaLinear / d)
	yLinear := (o.OY * deltaLinear / d)
	zLinear := (o.OZ * deltaLinear / d)

	approachPoint := r3.Vector{
		X: p.X - xLinear,
		Y: p.Y - yLinear,
		Z: p.Z - zLinear,
	}

	return approachPoint
}

func GetMergedPointCloud(ctx context.Context, positions []toggleswitch.Switch, sleepTime time.Duration, srcCamera camera.Camera, extraForCamera map[string]interface{}, fsSvc framesystem.Service) (pointcloud.PointCloud, error) {
	pcsInWorld := []pointcloud.PointCloud{}
	totalSize := 0

	for _, p := range positions {
		err := p.SetPosition(ctx, 2, nil)
		if err != nil {
			return nil, err
		}

		// Sleep between movements to allow for any vibrations to settle
		time.Sleep(sleepTime)

		pc, err := srcCamera.NextPointCloud(ctx, extraForCamera)
		if err != nil {
			return nil, err
		}

		totalSize += pc.Size()

		// Transform this point cloud into the world frame
		pif, err := fsSvc.GetPose(ctx, srcCamera.Name().Name, "", nil, nil)
		if err != nil {
			return nil, err
		}
		pcInWorld := pointcloud.NewBasicPointCloud(pc.Size())
		err = pointcloud.ApplyOffset(pc, pif.Pose(), pcInWorld)
		if err != nil {
			return nil, err
		}

		pcsInWorld = append(pcsInWorld, pcInWorld)
	}

	big := pointcloud.NewBasicPointCloud(totalSize)
	for _, pcInWorld := range pcsInWorld {
		err := pointcloud.ApplyOffset(pcInWorld, nil, big)
		if err != nil {
			return nil, err
		}
	}

	return big, nil
}

func buildWorldStateWithObstacles(ctx context.Context, visionSvcs []vision.Service) (*referenceframe.WorldState, error) {
	var obstacles []*referenceframe.GeometriesInFrame
	for _, v := range visionSvcs {
		vizs, err := v.GetObjectPointClouds(ctx, "", nil)
		if err != nil {
			return nil, fmt.Errorf("error while calling GetObjectPointClouds on vision service %s, %w", v.Name(), err)
		}
		for _, viz := range vizs {
			if viz.Geometry != nil {
				gif := referenceframe.NewGeometriesInFrame(referenceframe.World, []spatialmath.Geometry{viz.Geometry})
				obstacles = append(obstacles, gif)
			}
		}
	}
	return referenceframe.NewWorldState(obstacles, []*referenceframe.LinkInFrame{} /* no additional transforms */)
}

func goToPositionUsingJointToJointMotion(
	ctx context.Context,
	joints []float64,
	armName string,
	motionSvc motion.Service,
	visionSvcs []vision.Service,
	extra map[string]any,
	logger logging.Logger,
) error {
	logger.Debugf("going to position using joint to joint motion")

	// Add obstacles to the world state from the configured vision services
	worldState, err := buildWorldStateWithObstacles(ctx, visionSvcs)
	if err != nil {
		return err
	}

	// Express the goal state in joint positions
	goalFrameSystemInputs := make(referenceframe.FrameSystemInputs)
	goalFrameSystemInputs[armName] = joints
	if extra == nil {
		extra = make(map[string]any)
	} else if extra[extraParamsKeyGoalState] != nil {
		return fmt.Errorf("cannot provide '%s' in 'extra' when using joint to joint motion", extraParamsKeyGoalState)
	}
	extra[extraParamsKeyGoalState] = serialize(goalFrameSystemInputs)

	// Call Motion.Move
	_, err = motionSvc.Move(ctx, motion.MoveReq{
		ComponentName: armName,
		WorldState:    worldState,
		Extra:         extra,
	})
	return err
}

func goToPositionUsingMoveToJointPositions(
	ctx context.Context,
	joints []float64,
	arm arm.Arm,
	extra map[string]any,
	logger logging.Logger,
) error {
	logger.Debugf("going to position using MoveToJointPositions")
	return arm.MoveToJointPositions(ctx, joints, extra)
}

func goToPositionUsingCartesianMotion(
	ctx context.Context,
	point r3.Vector,
	orientation spatialmath.OrientationVectorDegrees,
	motionSvc motion.Service,
	visionSvcs []vision.Service,
	fsSvc framesystem.Service,
	armName string,
	extra map[string]any,
	logger logging.Logger,
) error {
	logger.Debugf("going to position using cartesian motion")

	// Check if we are already close enough
	current, err := fsSvc.GetPose(ctx, armName, referenceframe.World, nil, nil)
	if err != nil {
		return err
	}

	linearDelta := current.Pose().Point().Distance(point)
	orientationDelta := spatialmath.QuatToR3AA(spatialmath.OrientationBetween(current.Pose().Orientation(), &orientation).Quaternion()).Norm2()

	logger.Debugf("goToSavePosition linearDelta: %v orientationDelta: %v", linearDelta, orientationDelta)
	if linearDelta < .1 && orientationDelta < .01 {
		logger.Debugf("close enough, not moving - linearDelta: %v orientationDelta: %v", linearDelta, orientationDelta)
		return nil
	}

	// Add obstacles to the world state from the configured vision services
	worldState, err := buildWorldStateWithObstacles(ctx, visionSvcs)
	if err != nil {
		return err
	}

	// Express the goal state in cartesian pose
	pif := referenceframe.NewPoseInFrame(
		referenceframe.World,
		spatialmath.NewPose(point, &orientation),
	)

	// Call Motion.Move
	done, err := motionSvc.Move(
		ctx,
		motion.MoveReq{
			ComponentName: armName,
			Destination:   pif,
			WorldState:    worldState,
			Extra:         extra,
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

func serialize(inputs referenceframe.FrameSystemInputs) map[string]any {
	m := map[string]interface{}{}
	confMap := map[string]interface{}{}
	for fName, input := range inputs {
		confMap[fName] = input
	}
	m["configuration"] = confMap
	return m
}
