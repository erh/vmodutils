package touch

import (
	"context"
	"fmt"
	"sync"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
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

	GoodColors []ColorFilter `json:"good_colors"`

	TransformBackToSourceFrame bool `json:"transform_back_to_source_frame"`
	ForwardSourceImages        bool `json:"forward_source_images"`
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
		name:   config.ResourceName(),
		cfg:    newConf,
		logger: logger,
	}

	cc.src, err = camera.FromProvider(deps, newConf.Src)
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

	name   resource.Name
	cfg    *CropCameraConfig
	logger logging.Logger

	src    camera.Camera
	client robot.Robot

	lock               sync.Mutex
	active             bool
	lastPointCloud     pointcloud.PointCloud
	lastPointCloudTime time.Time
	lastPointCloudErr  error
}

func (cc *cropCamera) Name() resource.Name {
	return cc.name
}

func (cc *cropCamera) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (cc *cropCamera) Images(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	pc, err := cc.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	start := time.Now()
	img := PCToImage(pc)
	elapsed := time.Since(start)
	if elapsed > (time.Millisecond * 100) {
		cc.logger.Infof("PCToImage took %v", elapsed)
	}
	ni, err := camera.NamedImageFromImage(img, "cropped", "image/png", data.Annotations{})
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}

	if !cc.cfg.ForwardSourceImages {
		return []camera.NamedImage{ni}, resource.ResponseMetadata{time.Now()}, nil
	}

	srcImgs, srcMeta, err := cc.src.Images(ctx, filterSourceNames, extra)
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}

	return append([]camera.NamedImage{ni}, srcImgs...), srcMeta, nil
}

func (cc *cropCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (cc *cropCamera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {

	start := time.Now()
	cc.lock.Lock()
	if cc.active {
		cc.lock.Unlock()
		return cc.waitForPointCloudAfter(ctx, start)
	}

	cc.active = true
	cc.lock.Unlock()

	pc, err := cc.doNextPointCloud(ctx, extra)

	cc.lock.Lock()
	cc.active = false
	cc.lastPointCloud = pc
	cc.lastPointCloudErr = err
	cc.lastPointCloudTime = time.Now()
	cc.lock.Unlock()

	return pc, err
}

func (cc *cropCamera) waitForPointCloudAfter(ctx context.Context, when time.Time) (pointcloud.PointCloud, error) {
	for {
		if ctx.Err() != nil {
			return nil, ctx.Err()
		}

		if time.Since(when) > time.Minute {
			return nil, fmt.Errorf("waitForPointCloudAfter timed out after %v", time.Since(when))
		}

		cc.lock.Lock()
		if cc.lastPointCloudTime.After(when) {
			pc := cc.lastPointCloud
			err := cc.lastPointCloudErr
			cc.lock.Unlock()
			return pc, err
		}
		cc.lock.Unlock()

		time.Sleep(time.Millisecond * 50)
	}
}

func (cc *cropCamera) doNextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	start := time.Now()

	pc, err := cc.src.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, err
	}

	timeA := time.Since(start)

	srcFrame := cc.cfg.Src
	if cc.cfg.SrcFrame != "" {
		srcFrame = cc.cfg.SrcFrame
	}

	pc, err = cc.client.TransformPointCloud(ctx, pc, srcFrame, "world")
	if err != nil {
		return nil, err
	}

	timeB := time.Since(start)

	pc = PCCropWithColor(pc, cc.cfg.Min, cc.cfg.Max, cc.cfg.GoodColors)
	timeC := time.Since(start)

	if cc.cfg.TransformBackToSourceFrame {
		pc, err = cc.client.TransformPointCloud(ctx, pc, "world", srcFrame)
		if err != nil {
			return nil, err
		}
	}
	timeD := time.Since(start)

	cc.logger.Debugf("cropCamera::NextPointCloud timeA: %v timeB: %v timeC: %v timeD: %v", timeA, timeB, timeC, timeD)

	return pc, nil

}

func (cc *cropCamera) Properties(ctx context.Context) (camera.Properties, error) {
	props := camera.Properties{
		SupportsPCD: true,
	}
	if !cc.cfg.TransformBackToSourceFrame {
		return props, nil
	}
	srcProps, err := cc.src.Properties(ctx)
	if err != nil {
		return props, err
	}
	props.IntrinsicParams = srcProps.IntrinsicParams
	props.DistortionParams = srcProps.DistortionParams
	return props, nil
}

func (cc *cropCamera) Close(ctx context.Context) error {
	return cc.client.Close(ctx)
}

func (cc *cropCamera) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}
