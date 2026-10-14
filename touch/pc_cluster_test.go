package touch

import (
	"context"
	"testing"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/services/vision"
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

	cam := inject.NewCamera("cam")
	cam.NextPointCloudFunc = func(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
		return in, nil
	}

	cs := &ClusterService{
		name: vision.Named("clusterer"),
		cam:  cam,
		conf: &ClusterConfig{Camera: "cam", MaxDistance: 30, MinPointsPerSegment: 20, MinPointsPerCluster: 100},
	}

	dets, err := cs.GetDetections3D(ctx, "", nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(dets), test.ShouldEqual, 1)
	test.That(t, len(dets[0].Transforms), test.ShouldEqual, 2)

	root, points := dets[0].Transforms[0], dets[0].Transforms[1]
	test.That(t, root.Name(), test.ShouldEqual, "clusterer/object-0")
	test.That(t, root.Parent(), test.ShouldEqual, "cam")
	test.That(t, root.Geometry(), test.ShouldNotBeNil)
	test.That(t, points.Parent(), test.ShouldEqual, root.Name())

	objs, err := cs.GetObjectPointClouds(ctx, "", nil)
	test.That(t, err, test.ShouldBeNil)
	octree, ok := points.Geometry().(*pointcloud.BasicOctree)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, octree.Size(), test.ShouldEqual, objs[0].PointCloud.Size())

	_, err = cs.GetDetections3D(ctx, "other", nil)
	test.That(t, err, test.ShouldNotBeNil)

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
