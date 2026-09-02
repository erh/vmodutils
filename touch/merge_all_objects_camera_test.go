package touch

import (
	"context"
	"errors"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/utils"
	viz "go.viam.com/rdk/vision"
	"go.viam.com/test"

	"github.com/erh/vmodutils/pcclean"
)

func TestParseVisionServices(t *testing.T) {
	t.Run("legacy string list is optional", func(t *testing.T) {
		list, err := parseVisionServices([]any{"left", "right"})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, list, test.ShouldResemble, []VisionServiceSource{
			{Name: "left"},
			{Name: "right"},
		})
	})

	t.Run("object list with min_objects", func(t *testing.T) {
		list, err := parseVisionServices([]any{
			map[string]any{"name": "left", "min_objects": 1},
			map[string]any{"name": "right", "min_objects": float64(0)},
		})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, list, test.ShouldResemble, []VisionServiceSource{
			{Name: "left", MinObjects: 1},
			{Name: "right", MinObjects: 0},
		})
	})

	t.Run("object list omits min_objects as zero", func(t *testing.T) {
		list, err := parseVisionServices([]any{map[string]any{"name": "only"}})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, list[0].MinObjects, test.ShouldEqual, 0)
	})

	t.Run("rejects missing name", func(t *testing.T) {
		_, err := parseVisionServices([]any{map[string]any{"min_objects": 1}})
		test.That(t, err, test.ShouldNotBeNil)
	})

	t.Run("mixed string and object entries", func(t *testing.T) {
		list, err := parseVisionServices([]any{
			"left",
			map[string]any{"name": "right", "min_objects": 1},
		})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, list, test.ShouldResemble, []VisionServiceSource{
			{Name: "left"},
			{Name: "right", MinObjects: 1},
		})
	})
}

func TestMergeAllObjectsConfigTransformAttributeMapLegacyStrings(t *testing.T) {
	// RDK validates modules via mapstructure, not encoding/json. Legacy string
	// lists must decode without "expected a map or struct, got string".
	attrs := utils.AttributeMap{
		"vision_services": []any{
			"sam3-segmenter-left",
			"sam3-segmenter-right",
		},
		"label":                  "stemless wine glass",
		"max_radius_from_center": 100,
	}
	cfg, err := resource.TransformAttributeMap[*MergeAllObjectsConfig](attrs)
	test.That(t, err, test.ShouldBeNil)
	deps, _, err := cfg.Validate("components.0")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"sam3-segmenter-left", "sam3-segmenter-right"})

	attrsObj := utils.AttributeMap{
		"vision_services": []any{
			map[string]any{"name": "left", "min_objects": 1.0},
			map[string]any{"name": "right", "min_objects": 1.0},
		},
	}
	cfgObj, err := resource.TransformAttributeMap[*MergeAllObjectsConfig](attrsObj)
	test.That(t, err, test.ShouldBeNil)
	sources, err := parseVisionServices(cfgObj.VisionServices)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, sources, test.ShouldResemble, []VisionServiceSource{
		{Name: "left", MinObjects: 1},
		{Name: "right", MinObjects: 1},
	})
}

func TestMergeAllObjectsConfigValidate(t *testing.T) {
	cfg := &MergeAllObjectsConfig{
		VisionServices: []any{
			map[string]any{"name": "left", "min_objects": 1},
			map[string]any{"name": "right", "min_objects": 1},
		},
	}
	deps, opt, err := cfg.Validate("components.0")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, opt, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"left", "right"})
}

func TestMergeAllObjectsNextPointCloudMinObjects(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()

	makePC := func(offset float64) pointcloud.PointCloud {
		pc := pointcloud.NewBasicEmpty()
		test.That(t, pc.Set(r3.Vector{X: 1 + offset, Y: 2, Z: 3}, pointcloud.NewBasicData()), test.ShouldBeNil)
		test.That(t, pc.Set(r3.Vector{X: 4 + offset, Y: 5, Z: 6}, pointcloud.NewBasicData()), test.ShouldBeNil)
		return pc
	}

	makeObj := func(offset float64) *viz.Object {
		obj, err := viz.NewObject(makePC(offset))
		test.That(t, err, test.ShouldBeNil)
		return obj
	}

	left := inject.NewVisionService("left")
	right := inject.NewVisionService("right")
	deps := resource.Dependencies{
		left.Name():  left,
		right.Name(): right,
	}

	build := func(sources []any) *MergeAllObjectsCamera {
		cfg := &MergeAllObjectsConfig{
			VisionServices: sources,
			Config:         pcclean.Config{Disable: true},
		}
		cam, err := newMergeAllObjects(ctx, deps, resource.Config{
			Name:                "merged",
			ConvertedAttributes: cfg,
		}, logger)
		test.That(t, err, test.ShouldBeNil)
		return cam.(*MergeAllObjectsCamera)
	}

	t.Run("required sources both present", func(t *testing.T) {
		left.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return []*viz.Object{makeObj(0)}, nil
		}
		right.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return []*viz.Object{makeObj(10)}, nil
		}
		cam := build([]any{
			map[string]any{"name": "left", "min_objects": 1},
			map[string]any{"name": "right", "min_objects": 1},
		})
		pc, err := cam.NextPointCloud(ctx, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pc.Size(), test.ShouldEqual, 4)
	})

	t.Run("required source missing objects fails closed", func(t *testing.T) {
		left.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return []*viz.Object{makeObj(0)}, nil
		}
		right.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return []*viz.Object{}, nil
		}
		cam := build([]any{
			map[string]any{"name": "left", "min_objects": 1},
			map[string]any{"name": "right", "min_objects": 1},
		})
		_, err := cam.NextPointCloud(ctx, nil)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "right")
		test.That(t, err.Error(), test.ShouldContainSubstring, "need at least 1")
	})

	t.Run("required source error fails closed", func(t *testing.T) {
		left.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return []*viz.Object{makeObj(0)}, nil
		}
		right.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return nil, errors.New("boom")
		}
		cam := build([]any{
			map[string]any{"name": "left", "min_objects": 1},
			map[string]any{"name": "right", "min_objects": 1},
		})
		_, err := cam.NextPointCloud(ctx, nil)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "required vision service")
		test.That(t, err.Error(), test.ShouldContainSubstring, "right")
	})

	t.Run("legacy optional source still soft-fails", func(t *testing.T) {
		left.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return []*viz.Object{makeObj(0)}, nil
		}
		right.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return nil, errors.New("boom")
		}
		cam := build([]any{"left", "right"})
		pc, err := cam.NextPointCloud(ctx, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pc.Size(), test.ShouldEqual, 2)
	})
}
