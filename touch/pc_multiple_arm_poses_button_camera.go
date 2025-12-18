package touch

import (
	"context"
	"errors"
	"sync"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"github.com/erh/vmodutils"
)

var MultipleArmPosesButtonCameraModel = vmodutils.NamespaceFamily.WithModel("pc-multiple-arm-poses-button-camera")

var (
	multipleArmPosesButtonCameraModelRegistryMu sync.Mutex
	multipleArmPosesButtonCameraModelRegistry   = map[string]*MultipleArmPosesButtonCamera{}
)

func init() {
	resource.RegisterComponent(
		camera.API,
		MultipleArmPosesButtonCameraModel,
		resource.Registration[camera.Camera, *MultipleArmPosesButtonCameraConfig]{
			Constructor: newMultipleArmPosesButtonCamera,
		})
}

type MultipleArmPosesButtonCameraConfig struct{}

func (c *MultipleArmPosesButtonCameraConfig) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

func newMultipleArmPosesButtonCamera(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (camera.Camera, error) {
	multipleArmPosesButtonCameraModelRegistryMu.Lock()
	defer multipleArmPosesButtonCameraModelRegistryMu.Unlock()
	_, err := resource.NativeConfig[*MultipleArmPosesButtonCameraConfig](config)
	if err != nil {
		return nil, err
	}

	mapbc := &MultipleArmPosesButtonCamera{name: config.ResourceName()}
	multipleArmPosesButtonCameraModelRegistry[mapbc.name.String()] = mapbc

	return mapbc, nil
}

type MultipleArmPosesButtonCamera struct {
	resource.AlwaysRebuild

	name resource.Name

	mu sync.Mutex
	pc pointcloud.PointCloud
}

// BEGIN API for MultipleArmPosesButton
func (mapbc *MultipleArmPosesButtonCamera) SetPointCloud(pc pointcloud.PointCloud) {
	mapbc.mu.Lock()
	defer mapbc.mu.Unlock()
	mapbc.pc = pc
}

func (mapbc *MultipleArmPosesButtonCamera) ClearPointCloud() {
	mapbc.mu.Lock()
	defer mapbc.mu.Unlock()
	mapbc.pc = nil
}

// END API for MultipleArmPosesButton

func (mapbc *MultipleArmPosesButtonCamera) Name() resource.Name {
	return mapbc.name
}

func (mapbc *MultipleArmPosesButtonCamera) Image(ctx context.Context, mimeType string, extra map[string]any) ([]byte, camera.ImageMetadata, error) {
	return nil, camera.ImageMetadata{}, errors.New("Image unimplemented")
}

func (mapbc *MultipleArmPosesButtonCamera) Images(ctx context.Context, filterSourceNames []string, extra map[string]any) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	return nil, resource.ResponseMetadata{}, errors.New("Images unimplemented")
}

func (mapbc *MultipleArmPosesButtonCamera) NextPointCloud(ctx context.Context, extra map[string]any) (pointcloud.PointCloud, error) {
	mapbc.mu.Lock()
	defer mapbc.mu.Unlock()
	if mapbc.pc == nil || mapbc.pc.Size() == 0 {
		return nil, errors.New("no pointcloud yet")
	}
	return mapbc.pc, nil
}
func (mapbc *MultipleArmPosesButtonCamera) Geometries(context.Context, map[string]any) ([]spatialmath.Geometry, error) {
	return nil, nil
}

func (mapbc *MultipleArmPosesButtonCamera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{SupportsPCD: true}, nil
}

func (mapbc *MultipleArmPosesButtonCamera) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	return nil, nil
}

func (mapbc *MultipleArmPosesButtonCamera) Close(ctx context.Context) error {
	multipleArmPosesButtonCameraModelRegistryMu.Lock()
	defer multipleArmPosesButtonCameraModelRegistryMu.Unlock()
	delete(multipleArmPosesButtonCameraModelRegistry, mapbc.name.String())
	return nil
}
