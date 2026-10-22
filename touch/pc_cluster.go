package touch

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	viz "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/classification"
	"go.viam.com/rdk/vision/detection3d"
	"go.viam.com/rdk/vision/objectdetection"
	"go.viam.com/rdk/vision/viscapture"

	"github.com/erh/vmodutils"
)

var ClusterModel = vmodutils.NamespaceFamily.WithModel("pc-cluster")

func init() {
	resource.RegisterService(
		vision.API,
		ClusterModel,
		resource.Registration[vision.Service, *ClusterConfig]{
			Constructor: newCluster,
		})
}

func within(a, b pointcloud.PointCloud, within float64) bool {
	amd := a.MetaData()
	bmd := b.MetaData()

	d := math.Abs(amd.Center().Distance(bmd.Center()))
	if d > (amd.MaxSideLength() + bmd.MaxSideLength()) {
		return false
	}

	good := false
	a.Iterate(0, 0, func(ap r3.Vector, _ pointcloud.Data) bool {
		b.Iterate(0, 0, func(bp r3.Vector, _ pointcloud.Data) bool {
			d := math.Abs(ap.Distance(bp))
			if d < within {
				good = true
				return false
			}
			return true
		})
		return true
	})

	return good
}

func addAll(sink, source pointcloud.PointCloud) error {
	var err error
	source.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		err = sink.Set(p, d)
		return err == nil
	})
	return err
}

func merge(a, b *pointcloud.BasicOctree) (*pointcloud.BasicOctree, error) {
	x := pointcloud.NewBasicPointCloud(a.Size() + b.Size())

	err := addAll(x, a)
	if err != nil {
		return nil, err
	}

	err = addAll(x, b)
	if err != nil {
		return nil, err
	}

	return pointcloud.ToBasicOctree(x, 0)
}

func Cluster(pc pointcloud.PointCloud, maxDistance float64, minPointsPerSegment, minPointsPerCluster int) ([]pointcloud.PointCloud, error) {

	buckets := map[string]pointcloud.PointCloud{}

	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		b := fmt.Sprintf("%d-%d-%d",
			int(math.Ceil(p.X/maxDistance)),
			int(math.Ceil(p.Y/maxDistance)),
			int(math.Ceil(p.Z/maxDistance)))

		pc, ok := buckets[b]
		if !ok {
			pc = pointcloud.NewBasicPointCloud(0)
			buckets[b] = pc
		}
		err := pc.Set(p, d)
		if err != nil {
			panic(err)
		}
		return true
	})

	segments := []*pointcloud.BasicOctree{}

	for _, b := range buckets {
		if minPointsPerSegment > 0 && b.Size() < minPointsPerSegment {
			continue
		}

		o, err := pointcloud.ToBasicOctree(b, 0)
		if err != nil {
			return nil, err
		}

		segments = append(segments, o)
	}

	for {
		start := len(segments)
		for x := 0; x < len(segments); x++ {
			for y := x + 1; y < len(segments); y++ {
				if within(segments[x], segments[y], maxDistance) {
					n, err := merge(segments[x], segments[y])
					if err != nil {
						return nil, err
					}
					segments[x] = n
					segments = append(segments[0:y], segments[y+1:]...)
				}
			}
		}
		if len(segments) == start {
			break
		}
	}

	clusters := []pointcloud.PointCloud{}
	for _, o := range segments {
		if o.Size() > minPointsPerCluster {
			clusters = append(clusters, o)
		}
	}

	return clusters, nil
}

type ClusterConfig struct {
	Camera              string
	MaxDistance         float64 `json:"max-distance"`
	MinPointsPerSegment int     `json:"min-points-per-segment"`
	MinPointsPerCluster int     `json:"min-points-per-cluster"`
}

func (cc *ClusterConfig) Validate(p string) ([]string, []string, error) {
	if cc.Camera == "" {
		return nil, nil, fmt.Errorf("need to specifiy camera")
	}

	if cc.MaxDistance <= 0 {
		return nil, nil, fmt.Errorf("need to spefict max-distance")
	}

	if cc.MinPointsPerSegment <= 0 {
		return nil, nil, fmt.Errorf("need to spefict min-points-per-segment")
	}
	if cc.MinPointsPerCluster <= 0 {
		return nil, nil, fmt.Errorf("need to spefict min-points-per-cluster")
	}

	return []string{cc.Camera}, nil, nil
}

type ClusterService struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name resource.Name
	cam  camera.Camera
	conf *ClusterConfig
}

func newCluster(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (vision.Service, error) {
	newConf, err := resource.NativeConfig[*ClusterConfig](config)
	if err != nil {
		return nil, err
	}

	cs := &ClusterService{
		name: config.ResourceName(),
		conf: newConf,
	}
	cs.cam, err = camera.FromProvider(deps, newConf.Camera)

	if err != nil {
		return nil, err
	}
	return cs, nil
}

func (cs *ClusterService) DetectionsFromCamera(ctx context.Context, cameraName string, extra map[string]interface{}) ([]objectdetection.Detection, error) {
	return nil, fmt.Errorf("n/a")
}

func (cs *ClusterService) Detections(ctx context.Context, img *camera.NamedImage, extra map[string]interface{}) ([]objectdetection.Detection, error) {
	return nil, fmt.Errorf("n/a")
}

func (cs *ClusterService) ClassificationsFromCamera(
	ctx context.Context,
	cameraName string,
	n int,
	extra map[string]interface{},
) (classification.Classifications, error) {
	return nil, fmt.Errorf("n/a")
}

func (cs *ClusterService) Classifications(
	ctx context.Context,
	img *camera.NamedImage,
	n int,
	extra map[string]interface{},
) (classification.Classifications, error) {
	return nil, fmt.Errorf("n/a")
}

func (cs *ClusterService) GetDetections3D(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*detection3d.Detection, error) {
	if cameraName != "" && cameraName != cs.conf.Camera {
		return nil, fmt.Errorf("bad cameraName %s", cameraName)
	}

	parent, err := cs.pointCloudFrame(ctx)
	if err != nil {
		return nil, err
	}

	objects, err := cs.segment(ctx, extra)
	if err != nil {
		return nil, err
	}

	prefix := cs.name.ShortName()
	detections := make([]*detection3d.Detection, 0, len(objects))
	for i, obj := range objects {
		// Names are only unique within one call, since clustering does not track objects across calls.
		name := fmt.Sprintf("%s/object-%d", prefix, i)
		rootPose := spatialmath.NewZeroPose()
		var rootGeom spatialmath.Geometry
		if obj.Geometry != nil {
			rootPose = obj.Geometry.Pose()
			rootGeom = obj.Geometry.Transform(spatialmath.PoseInverse(rootPose))
		}
		transforms := []*referenceframe.LinkInFrame{referenceframe.NewLinkInFrame(parent, rootPose, name, rootGeom)}

		if obj.PointCloud != nil && obj.PointCloud.Size() > 0 {
			octree, err := pointcloud.ToBasicOctree(obj.PointCloud, 0)
			if err != nil {
				return nil, fmt.Errorf("object %d: %w", i, err)
			}
			points := octree.Transform(spatialmath.PoseInverse(rootPose))
			transforms = append(transforms, referenceframe.NewLinkInFrame(name, spatialmath.NewZeroPose(), name+"/points", points))
		}
		detections = append(detections, &detection3d.Detection{Transforms: transforms})
	}
	return detections, nil
}

// pointCloudFrame returns the frame the camera's point clouds are expressed in: its default reference
// frame when it reports one, otherwise the camera's own frame.
func (cs *ClusterService) pointCloudFrame(ctx context.Context) (string, error) {
	props, err := cs.cam.Properties(ctx)
	if err != nil {
		return "", fmt.Errorf("getting properties of camera %s: %w", cs.conf.Camera, err)
	}
	if props.DefaultReferenceFrame != "" {
		return props.DefaultReferenceFrame, nil
	}
	return cs.cam.Name().ShortName(), nil
}

func (cs *ClusterService) GetObjectPointClouds(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
	if cameraName != "" && cameraName != cs.conf.Camera {
		return nil, fmt.Errorf("bad cameraName %s", cameraName)
	}

	return cs.segment(ctx, extra)
}

func (cs *ClusterService) segment(ctx context.Context, extra map[string]interface{}) ([]*viz.Object, error) {
	pc, err := cs.cam.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, err
	}

	clusters, err := Cluster(pc, cs.conf.MaxDistance, cs.conf.MinPointsPerSegment, cs.conf.MinPointsPerCluster)
	if err != nil {
		return nil, err
	}

	os := []*viz.Object{}

	for _, c := range clusters {
		o, err := viz.NewObject(c)
		if err != nil {
			return nil, err
		}
		os = append(os, o)
	}

	return os, nil
}

func (cs *ClusterService) GetProperties(ctx context.Context, extra map[string]interface{}) (*vision.Properties, error) {
	return &vision.Properties{
		ObjectPCDsSupported:   true,
		Detections3DSupported: true,
	}, nil
}

func (cs *ClusterService) CaptureAllFromCamera(ctx context.Context,
	cameraName string,
	opts viscapture.CaptureOptions,
	extra map[string]interface{}) (viscapture.VisCapture, error) {
	return viscapture.VisCapture{}, fmt.Errorf("n/a")
}

func (cs *ClusterService) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (cs *ClusterService) Name() resource.Name {
	return cs.name
}

func (cs *ClusterService) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}
