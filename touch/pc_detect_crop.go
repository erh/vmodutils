package touch

import (
	"context"
	"fmt"
	"image"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/vision/objectdetection"

	"github.com/erh/vmodutils"
)

var DetectCropCameraModel = vmodutils.NamespaceFamily.WithModel("pc-detect-crop-camera")

func init() {
	resource.RegisterComponent(
		camera.API,
		DetectCropCameraModel,
		resource.Registration[camera.Camera, *DetectCropCameraConfig]{
			Constructor: newDetectCropCamera,
		})
}

type DetectCropCameraConfig struct {
	Src     string
	Service string

	Labels []string
	Min    float64
}

func (dccc *DetectCropCameraConfig) Validate(path string) ([]string, []string, error) {
	if dccc.Src == "" {
		return nil, nil, fmt.Errorf("need a src camera")
	}
	if dccc.Service == "" {
		return nil, nil, fmt.Errorf("need a service")
	}

	return []string{dccc.Src, dccc.Service}, nil, nil
}

type detectCropCamera struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	cfg    *DetectCropCameraConfig
	logger logging.Logger

	src     camera.Camera
	service vision.Service

	props camera.Properties
}

func newDetectCropCamera(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (camera.Camera, error) {
	newConf, err := resource.NativeConfig[*DetectCropCameraConfig](config)
	if err != nil {
		return nil, err
	}

	cc := &detectCropCamera{
		name:   config.ResourceName(),
		cfg:    newConf,
		logger: logger,
	}

	cc.src, err = camera.FromProvider(deps, newConf.Src)
	if err != nil {
		return nil, err
	}

	cc.service, err = vision.FromProvider(deps, newConf.Service)
	if err != nil {
		return nil, err
	}

	cc.props, err = cc.src.Properties(ctx)
	if err != nil {
		return nil, err
	}
	if cc.props.IntrinsicParams == nil {
		return nil, fmt.Errorf("no IntrinsicParams on %s", newConf.Src)
	}

	return cc, nil
}

func (dcc *detectCropCamera) Name() resource.Name {
	return dcc.name
}

func (dcc *detectCropCamera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: true,
	}, nil
}

func (dcc *detectCropCamera) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}

func (dcc *detectCropCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (dcc *detectCropCamera) Image(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
	pc, err := dcc.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, camera.ImageMetadata{}, err
	}
	img := PCToImage(pc)

	data, err := rimage.EncodeImage(ctx, img, mimeType)
	if err != nil {
		return nil, camera.ImageMetadata{}, err
	}

	return data, camera.ImageMetadata{MimeType: mimeType}, err
}

func (dcc *detectCropCamera) Images(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	pc, err := dcc.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	start := time.Now()
	img := PCToImage(pc)
	elapsed := time.Since(start)
	if elapsed > (time.Millisecond * 100) {
		dcc.logger.Infof("PCToImage took %v", elapsed)
	}
	ni, err := camera.NamedImageFromImage(img, "cropped", "image/png", data.Annotations{})
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	return []camera.NamedImage{ni}, resource.ResponseMetadata{time.Now()}, nil
}

func (dcc *detectCropCamera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	// todo: maybe paralllize this
	pc, err := dcc.src.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, err
	}

	detections, err := dcc.service.DetectionsFromCamera(ctx, "", nil)
	if err != nil {
		return nil, err
	}

	return PCDetectCrop(pc, detections, dcc.props)
}

func PCDetectCrop(
	pc pointcloud.PointCloud,
	detections []objectdetection.Detection,
	props camera.Properties) (pointcloud.PointCloud, error) {

	if props.IntrinsicParams == nil {
		return nil, fmt.Errorf("intrinsics cannot be null")
	}

	boxes := []*image.Rectangle{}
	for _, d := range detections {
		boxes = append(boxes, d.BoundingBox())
	}

	return PCLimitToImageBoxes(pc, boxes, nil, props)
}

func PCLimitToImageBoxes(
	pc pointcloud.PointCloud,
	boxes []*image.Rectangle,
	colorCheck ColorCheck,
	props camera.Properties) (pointcloud.PointCloud, error) {

	out := pointcloud.NewBasicEmpty()

	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		x, y := props.IntrinsicParams.PointToPixel(p.X, p.Y, p.Z)

		inBox := false
		for _, b := range boxes {
			if int(x) >= b.Min.X &&
				int(x) <= b.Max.X &&
				int(y) >= b.Min.Y &&
				int(y) <= b.Max.Y {
				inBox = true
				break
			}
		}
		if !inBox {
			return true
		}

		if colorCheck != nil {
			good, err := colorCheck(d.Color())
			if err != nil || !good {
				return true
			}
		}

		out.Set(p, d)

		return true
	})

	return out, nil
}
