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
	"go.viam.com/rdk/spatialmath"

	"github.com/erh/vmodutils"
)

var MergeModel = vmodutils.NamespaceFamily.WithModel("pc-merge")

func init() {
	resource.RegisterComponent(
		camera.API,
		MergeModel,
		resource.Registration[camera.Camera, *MergeConfig]{
			Constructor: newMerge,
		})
}

type MergeConfig struct {
	Cameras []string
}

func (c *MergeConfig) Validate(path string) ([]string, []string, error) {
	if len(c.Cameras) == 0 {
		return nil, nil, fmt.Errorf("need cameras")
	}

	return c.Cameras, nil, nil
}

func newMerge(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (camera.Camera, error) {
	newConf, err := resource.NativeConfig[*MergeConfig](config)
	if err != nil {
		return nil, err
	}

	cc := &MergeCamera{
		name:    config.ResourceName(),
		cfg:     newConf,
		cameras: []camera.Camera{},
	}

	for _, cn := range newConf.Cameras {
		c, err := camera.FromProvider(deps, cn)
		if err != nil {
			return nil, err
		}
		cc.cameras = append(cc.cameras, c)
	}

	return cc, nil
}

type MergeCamera struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name resource.Name
	cfg  *MergeConfig

	cameras []camera.Camera
}

func (mapc *MergeCamera) Name() resource.Name {
	return mapc.name
}

func (mapc *MergeCamera) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (mapc *MergeCamera) Images(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	pc, err := mapc.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	img := PCToImage(pc)

	ni, err := camera.NamedImageFromImage(img, "cropped", "image/png", data.Annotations{})
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	return []camera.NamedImage{ni}, resource.ResponseMetadata{time.Now()}, nil
}

func (mapc *MergeCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (mapc *MergeCamera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	inputs := []pointcloud.PointCloud{}
	totalSize := 0

	for _, c := range mapc.cameras {

		pc, err := c.NextPointCloud(ctx, extra)
		if err != nil {
			return nil, err
		}

		totalSize += pc.Size()

		inputs = append(inputs, pc)
	}

	big := pointcloud.NewBasicPointCloud(totalSize)

	for _, pc := range inputs {
		err := pointcloud.ApplyOffset(pc, nil, big)
		if err != nil {
			return nil, err
		}
	}

	return big, nil
}

func (mapc *MergeCamera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: true,
	}, nil
}

func (mapc *MergeCamera) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}
