package touch

import (
	"context"
	"errors"
	"testing"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
)

func TestCluster1(t *testing.T) {
	in, err := pointcloud.NewFromFile("data/glass1.pcd", "")
	test.That(t, err, test.ShouldBeNil)

	clusters, err := Cluster(in, 30, 20, 100)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(clusters), test.ShouldEqual, 1)
}

func TestClusterGetDetections3D(t *testing.T) {
	ctx := context.Background()

	in, err := pointcloud.NewFromFile("data/glass1.pcd", "")
	test.That(t, err, test.ShouldBeNil)

	defaultFrame := ""
	cam := inject.NewCamera("cam")
	cam.NextPointCloudFunc = func(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
		return in, nil
	}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{SupportsPCD: true, DefaultReferenceFrame: defaultFrame}, nil
	}

	cs := &ClusterService{
		name: vision.Named("clusterer"),
		cam:  cam,
		conf: &ClusterConfig{Camera: "cam", MaxDistance: 30, MinPointsPerSegment: 20, MinPointsPerCluster: 100},
	}

	t.Run("objects are parented to the camera when it has no default frame", func(t *testing.T) {
		dets, err := cs.GetDetections3D(ctx, "", nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(dets), test.ShouldEqual, 1)
		test.That(t, len(dets[0].Transforms), test.ShouldEqual, 2)

		root, points := dets[0].Transforms[0], dets[0].Transforms[1]
		test.That(t, root.Name(), test.ShouldEqual, "clusterer/object-0")
		test.That(t, root.Parent(), test.ShouldEqual, "cam")
		test.That(t, root.Geometry(), test.ShouldNotBeNil)
		test.That(t, spatialmath.PoseAlmostEqual(root.Geometry().Pose(), spatialmath.NewZeroPose()), test.ShouldBeTrue)
		test.That(t, points.Name(), test.ShouldEqual, "clusterer/object-0/points")
		test.That(t, points.Parent(), test.ShouldEqual, root.Name())

		objs, err := cs.GetObjectPointClouds(ctx, "", nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, spatialmath.PoseAlmostEqual(root.Pose(), objs[0].Geometry.Pose()), test.ShouldBeTrue)

		octree, ok := points.Geometry().(*pointcloud.BasicOctree)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, octree.Size(), test.ShouldEqual, objs[0].PointCloud.Size())
		pointsMeta, objMeta := octree.MetaData(), objs[0].PointCloud.MetaData()
		test.That(t, spatialmath.R3VectorAlmostEqual(
			pointsMeta.Center(), objMeta.Center().Sub(root.Pose().Point()), 1e-6), test.ShouldBeTrue)
	})

	t.Run("objects are parented to the camera's default frame when set", func(t *testing.T) {
		defaultFrame = "depth-sensor"
		defer func() { defaultFrame = "" }()

		dets, err := cs.GetDetections3D(ctx, "", nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(dets), test.ShouldEqual, 1)
		test.That(t, dets[0].Transforms[0].Parent(), test.ShouldEqual, "depth-sensor")
	})

	t.Run("properties errors are returned", func(t *testing.T) {
		cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
			return camera.Properties{}, errors.New("boom")
		}
		_, err := cs.GetDetections3D(ctx, "", nil)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "boom")
	})

	t.Run("other camera names are rejected", func(t *testing.T) {
		_, err := cs.GetDetections3D(ctx, "other", nil)
		test.That(t, err, test.ShouldNotBeNil)
	})

	props, err := cs.GetProperties(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, props.Detections3DSupported, test.ShouldBeTrue)
}

func BenchmarkCluster1(t *testing.B) {
	in, err := pointcloud.NewFromFile("data/glass1.pcd", "")
	test.That(t, err, test.ShouldBeNil)

	t.ResetTimer()
	for range t.N {
		clusters, err := Cluster(in, 30, 20, 100)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(clusters), test.ShouldEqual, 1)
	}
}
