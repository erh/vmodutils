package touch

import (
	"context"
	"fmt"
	"math"

	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot"

	"github.com/erh/vmodutils"
)

var CalibrationCheckerModel = vmodutils.NamespaceFamily.WithModel("calibration-checker")

func init() {
	resource.RegisterComponent(
		sensor.API,
		CalibrationCheckerModel,
		resource.Registration[sensor.Sensor, *CalibrationCheckerConfig]{
			Constructor: newCalibrationChecker,
		})
}

type CalibrationCheckerConfig struct {
	PoseTrackers []string `json:"pose_trackers"`
	TagID        string   `json:"tag_id,omitempty"`
	ToleranceMM  float64  `json:"tolerance_mm,omitempty"`
}

func (c *CalibrationCheckerConfig) Validate(path string) ([]string, []string, error) {
	if len(c.PoseTrackers) < 2 {
		return nil, nil, fmt.Errorf("need at least 2 pose_trackers, got %d", len(c.PoseTrackers))
	}
	deps := make([]string, len(c.PoseTrackers))
	copy(deps, c.PoseTrackers)
	return deps, nil, nil
}

func (c *CalibrationCheckerConfig) tagID() string {
	if c.TagID != "" {
		return c.TagID
	}
	return "0"
}

func (c *CalibrationCheckerConfig) toleranceMM() float64 {
	if c.ToleranceMM > 0 {
		return c.ToleranceMM
	}
	return 10
}

func newCalibrationChecker(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (sensor.Sensor, error) {
	config, err := resource.NativeConfig[*CalibrationCheckerConfig](conf)
	if err != nil {
		return nil, err
	}

	trackers := make([]posetracker.PoseTracker, len(config.PoseTrackers))
	for i, name := range config.PoseTrackers {
		trackers[i], err = posetracker.FromDependencies(deps, name)
		if err != nil {
			return nil, err
		}
	}

	r, err := vmodutils.ConnectToMachineFromEnv(ctx, logger)
	if err != nil {
		return nil, err
	}

	return &calibrationChecker{
		name:     conf.ResourceName(),
		config:   config,
		trackers: trackers,
		robot:    r,
		logger:   logger,
	}, nil
}

type calibrationChecker struct {
	resource.AlwaysRebuild

	name     resource.Name
	config   *CalibrationCheckerConfig
	trackers []posetracker.PoseTracker
	robot    robot.Robot
	logger   logging.Logger
}

func (c *calibrationChecker) Close(ctx context.Context) error {
	return c.robot.Close(ctx)
}

func (c *calibrationChecker) Name() resource.Name {
	return c.name
}

func (c *calibrationChecker) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	return c.check(ctx)
}

func (c *calibrationChecker) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return c.check(ctx)
}

func (c *calibrationChecker) check(ctx context.Context) (map[string]interface{}, error) {
	tagID := c.config.tagID()
	tolerance := c.config.toleranceMM()
	result := map[string]interface{}{}

	type worldPoint struct {
		name string
		x, y, z float64
	}
	var points []worldPoint

	for i, tracker := range c.trackers {
		name := c.config.PoseTrackers[i]
		poses, err := tracker.Poses(ctx, nil, nil)
		if err != nil {
			result[name+"_error"] = err.Error()
			continue
		}
		tagPose, ok := poses[tagID]
		if !ok {
			result[name+"_visible"] = false
			continue
		}
		result[name+"_visible"] = true

		worldPose, err := c.robot.TransformPose(ctx, tagPose, "world", nil)
		if err != nil {
			result[name+"_transform_error"] = err.Error()
			continue
		}
		pt := worldPose.Pose().Point()
		result[name+"_x"] = pt.X
		result[name+"_y"] = pt.Y
		result[name+"_z"] = pt.Z
		points = append(points, worldPoint{name, pt.X, pt.Y, pt.Z})
	}

	if len(points) < 2 {
		result["calibration_ok"] = true
		result["reason"] = "fewer than 2 trackers could see the tag, skipping check"
		return result, nil
	}

	var maxDist float64
	var maxPairA, maxPairB string
	for i := 0; i < len(points); i++ {
		for j := i + 1; j < len(points); j++ {
			dx := points[i].x - points[j].x
			dy := points[i].y - points[j].y
			dz := points[i].z - points[j].z
			dist := math.Sqrt(dx*dx + dy*dy + dz*dz)
			if dist > maxDist {
				maxDist = dist
				maxPairA = points[i].name
				maxPairB = points[j].name
			}
		}
	}

	result["max_distance_mm"] = maxDist
	result["tolerance_mm"] = tolerance

	if maxDist > tolerance {
		result["calibration_ok"] = false
		msg := fmt.Sprintf("%s and %s see tag %q %.1fmm apart in world frame (tolerance %.1fmm)",
			maxPairA, maxPairB, tagID, maxDist, tolerance)
		result["error"] = msg
		c.logger.Warnf("calibration check failed: %s", msg)
	} else {
		result["calibration_ok"] = true
		c.logger.Debugf("calibration check OK: max distance %.1fmm (tolerance %.1fmm)", maxDist, tolerance)
	}

	return result, nil
}
