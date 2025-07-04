package touch

import (
	"context"
	"fmt"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/spatialmath"

	"github.com/erh/vmodutils"
)

var CropCameraModel = vmodutils.NamespaceFamily.WithModel("pc-crop-camera")

func init() {
	resource.RegisterComponent(
		camera.API,
		CropCameraModel,
		resource.Registration[camera.Camera, *CropCameraConfig]{
			Constructor: newCropCamera,
		})
}

type CropCameraConfig struct {
	Src      string
	SrcFrame string `json:"src_frame"`
	Min      r3.Vector
	Max      r3.Vector
}

func (ccc *CropCameraConfig) Validate(path string) ([]string, []string, error) {
	if ccc.Src == "" {
		return nil, nil, fmt.Errorf("need a src camera")
	}
	return []string{ccc.Src}, nil, nil
}

func newCropCamera(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (camera.Camera, error) {
	newConf, err := resource.NativeConfig[*CropCameraConfig](config)
	if err != nil {
		return nil, err
	}

	cc := &cropCamera{
		name: config.ResourceName(),
		cfg:  newConf,
	}

	cc.src, err = camera.FromDependencies(deps, newConf.Src)
	if err != nil {
		return nil, err
	}

	cc.client, err = vmodutils.ConnectToMachineFromEnv(ctx, logger)
	if err != nil {
		return nil, err
	}

	return cc, nil
}

type cropCamera struct {
	resource.AlwaysRebuild

	name resource.Name
	cfg  *CropCameraConfig

	src    camera.Camera
	client robot.Robot
}

func (cc *cropCamera) Name() resource.Name {
	return cc.name
}

func (cc *cropCamera) Image(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
	pc, err := cc.NextPointCloud(ctx)
	if err != nil {
		return nil, camera.ImageMetadata{}, err
	}
	img := PCToImage(pc)

	data, err := rimage.EncodeImage(ctx, img, mimeType)
	if err != nil {
		return nil, camera.ImageMetadata{}, err
	}

	return data, camera.ImageMetadata{mimeType}, err
}

func (cc *cropCamera) Images(ctx context.Context) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	pc, err := cc.NextPointCloud(ctx)
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	img := PCToImage(pc)

	return []camera.NamedImage{{img, "cropped"}}, resource.ResponseMetadata{time.Now()}, nil
}

func (cc *cropCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (cc *cropCamera) NextPointCloud(ctx context.Context) (pointcloud.PointCloud, error) {
	pc, err := cc.src.NextPointCloud(ctx)
	if err != nil {
		return nil, err
	}

	srcFrame := cc.cfg.Src
	if cc.cfg.SrcFrame != "" {
		srcFrame = cc.cfg.SrcFrame
	}

	pc, err = cc.client.TransformPointCloud(ctx, pc, srcFrame, "world")
	if err != nil {
		return nil, err
	}

	pc = PCCrop(pc, cc.cfg.Min, cc.cfg.Max)
	return pc, nil
}

func (cc *cropCamera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: true,
	}, nil
}

func (cc *cropCamera) Close(ctx context.Context) error {
	return cc.client.Close(ctx)
}

func (cc *cropCamera) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}
