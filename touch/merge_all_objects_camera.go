package touch

import (
	"context"
	"encoding/json"
	"fmt"
	"sync"
	"time"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	viz "go.viam.com/rdk/vision"

	"github.com/erh/vmodutils"
	"github.com/erh/vmodutils/pcclean"
)

var MergeAllObjectsModel = vmodutils.NamespaceFamily.WithModel("merge-all-objects-pointclouds")

func init() {
	resource.RegisterComponent(
		camera.API,
		MergeAllObjectsModel,
		resource.Registration[camera.Camera, *MergeAllObjectsConfig]{
			Constructor: newMergeAllObjects,
		})
}

// VisionServiceSource is one GetObjectPointClouds dependency for the merge camera.
// MinObjects is the minimum number of label-matching, non-empty objects required
// from this service on each NextPointCloud call. 0 means the source is optional
// (errors / empty results are skipped, matching legacy string-list behavior).
type VisionServiceSource struct {
	Name       string `json:"name"`
	MinObjects int    `json:"min_objects"`
}

// VisionServiceSourceList unmarshals either a legacy string list
// (["left","right"] → MinObjects 0) or a list of source objects
// ([{"name":"left","min_objects":1}, ...]).
type VisionServiceSourceList []VisionServiceSource

func (l *VisionServiceSourceList) UnmarshalJSON(data []byte) error {
	if len(data) == 0 || string(data) == "null" {
		*l = nil
		return nil
	}

	var names []string
	if err := json.Unmarshal(data, &names); err == nil {
		out := make(VisionServiceSourceList, len(names))
		for i, n := range names {
			out[i] = VisionServiceSource{Name: n, MinObjects: 0}
		}
		*l = out
		return nil
	}

	type rawSource struct {
		Name       string `json:"name"`
		MinObjects *int   `json:"min_objects"`
	}
	var raw []rawSource
	if err := json.Unmarshal(data, &raw); err != nil {
		return fmt.Errorf("vision_services must be a list of strings or objects with name/min_objects: %w", err)
	}
	out := make(VisionServiceSourceList, len(raw))
	for i, r := range raw {
		if r.Name == "" {
			return fmt.Errorf("vision_services[%d]: name is required", i)
		}
		minObjects := 0
		if r.MinObjects != nil {
			minObjects = *r.MinObjects
		}
		if minObjects < 0 {
			return fmt.Errorf("vision_services[%d]: min_objects must be >= 0", i)
		}
		out[i] = VisionServiceSource{Name: r.Name, MinObjects: minObjects}
	}
	*l = out
	return nil
}

func (l VisionServiceSourceList) Names() []string {
	names := make([]string, len(l))
	for i, s := range l {
		names[i] = s.Name
	}
	return names
}

type MergeAllObjectsConfig struct {
	VisionServices VisionServiceSourceList `json:"vision_services"`
	Label          string                  `json:"label"`

	pcclean.Config
}

func (c *MergeAllObjectsConfig) Validate(path string) ([]string, []string, error) {
	if len(c.VisionServices) == 0 {
		return nil, nil, fmt.Errorf("need at least one vision service")
	}
	for i, s := range c.VisionServices {
		if s.Name == "" {
			return nil, nil, fmt.Errorf("vision_services[%d]: name is required", i)
		}
		if s.MinObjects < 0 {
			return nil, nil, fmt.Errorf("vision_services[%d]: min_objects must be >= 0", i)
		}
	}
	return c.VisionServices.Names(), nil, nil
}

func newMergeAllObjects(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (camera.Camera, error) {
	newConf, err := resource.NativeConfig[*MergeAllObjectsConfig](config)
	if err != nil {
		return nil, err
	}
	pcclean.FillDefaults(&newConf.Config)

	cc := &MergeAllObjectsCamera{
		name:    config.ResourceName(),
		cfg:     newConf,
		sources: make([]mergeAllObjectsSource, 0, len(newConf.VisionServices)),
		logger:  logger,
	}

	for _, src := range newConf.VisionServices {
		s, err := vision.FromProvider(deps, src.Name)
		if err != nil {
			return nil, err
		}
		cc.sources = append(cc.sources, mergeAllObjectsSource{
			svc:        s,
			minObjects: src.MinObjects,
		})
	}

	return cc, nil
}

type mergeAllObjectsSource struct {
	svc        vision.Service
	minObjects int
}

type MergeAllObjectsCamera struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name    resource.Name
	cfg     *MergeAllObjectsConfig
	logger  logging.Logger
	sources []mergeAllObjectsSource
}

func (opc *MergeAllObjectsCamera) Name() resource.Name {
	return opc.name
}

func (opc *MergeAllObjectsCamera) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (opc *MergeAllObjectsCamera) Images(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
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

func (opc *MergeAllObjectsCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

type sourceFetchResult struct {
	objects []*viz.Object
	err     error
}

func (opc *MergeAllObjectsCamera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	results := make([]sourceFetchResult, len(opc.sources))
	var wg sync.WaitGroup
	for i, src := range opc.sources {
		wg.Add(1)
		go func(i int, src mergeAllObjectsSource) {
			defer wg.Done()
			objects, err := src.svc.GetObjectPointClouds(ctx, "", extra)
			results[i] = sourceFetchResult{objects: objects, err: err}
		}(i, src)
	}
	wg.Wait()

	inputs := []pointcloud.PointCloud{}
	totalSize := 0

	for i, src := range opc.sources {
		res := results[i]
		if res.err != nil {
			if src.minObjects > 0 {
				return nil, fmt.Errorf("required vision service %s failed GetObjectPointClouds: %w", src.svc.Name(), res.err)
			}
			opc.logger.Warnf("error getting object point clouds from %s: %v", src.svc.Name(), res.err)
			continue
		}

		count := 0
		for _, obj := range res.objects {
			if opc.cfg.Label != "" && obj.Geometry != nil {
				if obj.Geometry.Label() != opc.cfg.Label {
					continue
				}
			}
			if obj.PointCloud == nil || obj.Size() == 0 {
				continue
			}
			count++
			totalSize += obj.Size()
			inputs = append(inputs, obj.PointCloud)
		}

		if count < src.minObjects {
			return nil, fmt.Errorf(
				"vision service %s: got %d matching object(s), need at least %d",
				src.svc.Name(), count, src.minObjects,
			)
		}
	}

	big := pointcloud.NewBasicPointCloud(totalSize)
	for _, pc := range inputs {
		if err := pointcloud.ApplyOffset(pc, nil, big); err != nil {
			return nil, err
		}
	}

	cleaned, err := pcclean.Clean(big, &opc.cfg.Config)
	if err != nil {
		return nil, err
	}
	return cleaned, nil
}

func (opc *MergeAllObjectsCamera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: true,
	}, nil
}

func (opc *MergeAllObjectsCamera) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}
