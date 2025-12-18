package touch

import (
	"context"
	"fmt"
	"image"
	"image/color"
	"math"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"github.com/lucasb-eyer/go-colorful"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/spatialmath"

	"github.com/erh/vmodutils"
)

var LookAtCameraModel = vmodutils.NamespaceFamily.WithModel("pc-look-at-crop-camera")

func init() {
	resource.RegisterComponent(
		camera.API,
		LookAtCameraModel,
		resource.Registration[camera.Camera, *LookAtCameraConfig]{
			Constructor: newLookAtCamera,
		})
}

type LookAtCameraConfig struct {
	Src      string
	UseColor bool `json:"use_color"`
}

func (ccc *LookAtCameraConfig) Validate(path string) ([]string, []string, error) {
	if ccc.Src == "" {
		return nil, nil, fmt.Errorf("need a src camera")
	}
	return []string{ccc.Src}, nil, nil
}

func newLookAtCamera(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (camera.Camera, error) {
	newConf, err := resource.NativeConfig[*LookAtCameraConfig](config)
	if err != nil {
		return nil, err
	}

	cc := &lookAtCamera{
		name:   config.ResourceName(),
		cfg:    newConf,
		logger: logger,
	}

	cc.src, err = camera.FromProvider(deps, newConf.Src)
	if err != nil {
		return nil, err
	}

	cc.srcProperties, err = cc.src.Properties(ctx)
	if err != nil {
		return nil, err
	}

	return cc, nil
}

type lookAtCamera struct {
	resource.AlwaysRebuild

	name   resource.Name
	cfg    *LookAtCameraConfig
	logger logging.Logger

	src           camera.Camera
	srcProperties camera.Properties

	lock               sync.Mutex
	active             bool
	lastPointCloud     pointcloud.PointCloud
	lastPointCloudTime time.Time
	lastPointCloudErr  error
}

func (cc *lookAtCamera) Name() resource.Name {
	return cc.name
}

func (cc *lookAtCamera) Image(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
	pc, err := cc.NextPointCloud(ctx, extra)
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

func (cc *lookAtCamera) Images(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
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
	return []camera.NamedImage{ni}, resource.ResponseMetadata{time.Now()}, nil
}

func (cc *lookAtCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (cc *lookAtCamera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {

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

func (cc *lookAtCamera) waitForPointCloudAfter(ctx context.Context, when time.Time) (pointcloud.PointCloud, error) {
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

func (cc *lookAtCamera) doNextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	pc, err := cc.src.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, err
	}

	if cc.cfg.UseColor {
		imgs, _, err := cc.src.Images(ctx, nil, nil)
		if err != nil {
			return nil, err
		}

		img, err := imgs[0].Image(ctx)
		if err != nil {
			return nil, err
		}

		box, colorChecker, err := getBoundingBoxBasedOnCenterHSV(img)
		if err != nil {
			return nil, err
		}

		pc, err = PCLimitToImageBoxes(pc, []*image.Rectangle{box}, colorChecker, cc.srcProperties)
		if err != nil {
			return nil, err
		}

	}

	start := time.Now()
	defer func() {
		cc.logger.Infof("PCLookAtSegment took %v", time.Since(start))
	}()

	return PCLookAtSegment(pc)
}

func (cc *lookAtCamera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: true,
	}, nil
}

func (cc *lookAtCamera) Close(ctx context.Context) error {
	return nil
}

func (cc *lookAtCamera) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}

func PCLookAtSegment(pc pointcloud.PointCloud) (pointcloud.PointCloud, error) {

	type data struct {
		p r3.Vector
		d pointcloud.Data
	}

	bucketSize := 5.0
	buckets := map[string]pointcloud.PointCloud{}

	hash := func(p r3.Vector) string {
		return fmt.Sprintf("%d-%d-%d",
			int(p.X/bucketSize),
			int(p.Y/bucketSize),
			int(p.Z/bucketSize),
		)
	}

	var best r3.Vector
	distanceFromCenter := 100000.0
	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if p.Z < 20 {
			return true
		}
		myD := math.Pow((p.X*p.X)+(p.Y*p.Y), .5)
		if myD < distanceFromCenter {
			distanceFromCenter = myD
			best = p
		}

		bucket := hash(p)

		x, ok := buckets[bucket]
		if !ok {
			x = pointcloud.NewBasicEmpty()
			buckets[bucket] = x
		}
		x.Set(p, d)

		return true
	})

	good := pointcloud.NewBasicEmpty()
	goodBucket, ok := buckets[hash(best)]
	if !ok {
		panic(1)
	}
	goodBucket.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		good.Set(p, d)
		return true
	})

	for {
		added := false

		for hash, b := range buckets {
			if b.Size() == 0 {
				continue
			}
			d := minEstimateDisance(good, b)
			if d > bucketSize*2 {
				continue
			}

			x := pointcloud.NewBasicEmpty()

			b.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
				if isWithin(p, good, bucketSize) {
					added = true
					good.Set(p, d)
				} else {
					x.Set(p, d)
				}
				return true
			})

			buckets[hash] = x
		}

		if !added {
			break
		}

	}

	return good, nil
}

func isWithin(look r3.Vector, pc pointcloud.PointCloud, distance float64) bool {
	md := pc.MetaData()

	dx := math.Max(0, math.Max(md.MinX-look.X, look.X-md.MaxX))
	dy := math.Max(0, math.Max(md.MinY-look.Y, look.Y-md.MaxY))
	dz := math.Max(0, math.Max(md.MinZ-look.Z, look.Z-md.MaxZ))

	return math.Sqrt(dx*dx+dy*dy+dz*dz) < distance
}

// takes 2 pointclouds, and figures out the the closest possible distance by just looking at the edges as defined by max and min
func minEstimateDisance(a, b pointcloud.PointCloud) float64 {
	if a.Size() == 0 || b.Size() == 0 {
		return math.Inf(1)
	}

	amd := a.MetaData()
	bmd := b.MetaData()

	// Calculate the minimum distance between two axis-aligned bounding boxes
	dx := math.Max(0, math.Max(amd.MinX-bmd.MaxX, bmd.MinX-amd.MaxX))
	dy := math.Max(0, math.Max(amd.MinY-bmd.MaxY, bmd.MinY-amd.MaxY))
	dz := math.Max(0, math.Max(amd.MinZ-bmd.MaxZ, bmd.MinZ-amd.MaxZ))

	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

type ColorCheck func(c color.Color) (bool, error)

func getBoundingBoxBasedOnCenterHSV(img image.Image) (*image.Rectangle, ColorCheck, error) {
	bounds := img.Bounds()

	centerX := bounds.Min.X + bounds.Dx()/2
	centerY := bounds.Min.Y + bounds.Dy()/2

	cc, ok := colorful.MakeColor(img.At(centerX, centerY))
	if !ok {
		return nil, nil, fmt.Errorf("bad color %v", img.At(centerX, centerY))
	}

	good, _, _ := cc.Hsv()
	good = math.Floor(good / 100)

	// Track visited pixels
	visited := make([][]bool, bounds.Dy())
	for i := range visited {
		visited[i] = make([]bool, bounds.Dx())
	}

	goodRectange := image.Rect(centerX, centerY, centerX, centerY)

	type point struct{ x, y int }
	queue := []point{{centerX, centerY}}

	colorChecker := func(c color.Color) (bool, error) {
		cc, ok := colorful.MakeColor(c)
		if !ok {
			return false, fmt.Errorf("bad color %v", c)
		}
		h, _, _ := cc.Hsv()
		bucket := math.Floor(h / 100)
		return bucket == good, nil
	}

	for len(queue) > 0 { // BFS from center
		p := queue[0]
		queue = queue[1:]

		// Bounds check
		if p.x < bounds.Min.X || p.x >= bounds.Max.X || p.y < bounds.Min.Y || p.y >= bounds.Max.Y {
			continue
		}

		// Already visited check
		vy, vx := p.y-bounds.Min.Y, p.x-bounds.Min.X
		if visited[vy][vx] {
			continue
		}
		visited[vy][vx] = true

		good, err := colorChecker(img.At(p.x, p.y))
		if err != nil {
			return nil, nil, err
		}
		if !good {
			continue
		}

		goodRectange.Min.X = min(goodRectange.Min.X, p.x)
		goodRectange.Min.Y = min(goodRectange.Min.Y, p.y)
		goodRectange.Max.X = max(goodRectange.Max.X, p.x)
		goodRectange.Max.Y = max(goodRectange.Max.Y, p.y)

		// Add neighbors
		queue = append(queue, point{p.x - 1, p.y})
		queue = append(queue, point{p.x + 1, p.y})
		queue = append(queue, point{p.x, p.y - 1})
		queue = append(queue, point{p.x, p.y + 1})
	}

	return &goodRectange, colorChecker, nil
}
