package touch

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"

	"github.com/erh/vmodutils"
)

var ObjectPCMergeModel = vmodutils.NamespaceFamily.WithModel("object-pc-merge")

func init() {
	resource.RegisterComponent(
		camera.API,
		ObjectPCMergeModel,
		resource.Registration[camera.Camera, *ObjectPCMergeConfig]{
			Constructor: newObjectPCMerge,
		})
}

type ObjectPCMergeConfig struct {
	VisionServices []string `json:"vision_services"`
	Label          string   `json:"label"`

	// Cleaning pipeline. Each stage is skipped when its primary knob is <= 0.
	// Zero values get the documented defaults applied in newObjectPCMerge so
	// existing JSON configs pick up cleaning automatically; set a knob to a
	// negative value to explicitly disable a stage.
	OutlierMeanK        int     `json:"outlier_mean_k,omitempty"`
	OutlierStdDevThresh float64 `json:"outlier_std_dev_thresh,omitempty"`

	ClusterMaxDistance         float64 `json:"cluster_max_distance,omitempty"`
	ClusterMinPointsPerSegment int     `json:"cluster_min_points_per_segment,omitempty"`
	ClusterMinPointsPerCluster int     `json:"cluster_min_points_per_cluster,omitempty"`

	MaxRadiusFromCenter float64 `json:"max_radius_from_center,omitempty"`

	DisableCleaning bool `json:"disable_cleaning,omitempty"`
}

func (c *ObjectPCMergeConfig) Validate(path string) ([]string, []string, error) {
	if len(c.VisionServices) == 0 {
		return nil, nil, fmt.Errorf("need at least one vision service")
	}

	return c.VisionServices, nil, nil
}

func newObjectPCMerge(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (camera.Camera, error) {
	newConf, err := resource.NativeConfig[*ObjectPCMergeConfig](config)
	if err != nil {
		return nil, err
	}
	fillCleaningDefaults(newConf)

	cc := &ObjectPCMergeCamera{
		name:     config.ResourceName(),
		cfg:      newConf,
		services: []vision.Service{},
		logger:   logger,
	}

	for _, sn := range newConf.VisionServices {
		s, err := vision.FromProvider(deps, sn)
		if err != nil {
			return nil, err
		}
		cc.services = append(cc.services, s)
	}

	return cc, nil
}

type ObjectPCMergeCamera struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name     resource.Name
	cfg      *ObjectPCMergeConfig
	logger   logging.Logger
	services []vision.Service
}

func (opc *ObjectPCMergeCamera) Name() resource.Name {
	return opc.name
}

func (opc *ObjectPCMergeCamera) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (opc *ObjectPCMergeCamera) Images(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	pc, err := opc.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	img := PCToImage(pc)

	ni, err := camera.NamedImageFromImage(img, "merged-objects", "image/png", data.Annotations{})
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	return []camera.NamedImage{ni}, resource.ResponseMetadata{CapturedAt: time.Now()}, nil
}

func (opc *ObjectPCMergeCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (opc *ObjectPCMergeCamera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	inputs := []pointcloud.PointCloud{}
	totalSize := 0

	for _, svc := range opc.services {
		objects, err := svc.GetObjectPointClouds(ctx, "", extra)
		if err != nil {
			opc.logger.Warnf("error getting object point clouds from %s: %v", svc.Name(), err)
			continue
		}

		for _, obj := range objects {
			// Filter by label if configured.
			if opc.cfg.Label != "" && obj.Geometry != nil {
				if obj.Geometry.Label() != opc.cfg.Label {
					continue
				}
			}

			if obj.PointCloud == nil || obj.Size() == 0 {
				continue
			}

			totalSize += obj.Size()
			inputs = append(inputs, obj.PointCloud)
		}
	}

	big := pointcloud.NewBasicPointCloud(totalSize)

	for _, pc := range inputs {
		err := pointcloud.ApplyOffset(pc, nil, big)
		if err != nil {
			return nil, err
		}
	}

	cleaned, err := cleanMerged(big, opc.cfg)
	if err != nil {
		return nil, err
	}

	return cleaned, nil
}

func (opc *ObjectPCMergeCamera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: true,
	}, nil
}

func (opc *ObjectPCMergeCamera) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}
