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

type VisionServiceSourceList []VisionServiceSource

func (l VisionServiceSourceList) Names() []string {
	names := make([]string, len(l))
	for i, s := range l {
		names[i] = s.Name
	}
	return names
}

// MergeAllObjectsConfig attributes. VisionServices is []any so RDK's mapstructure
// attribute decoder accepts both legacy string lists and {name,min_objects} objects.
// encoding/json UnmarshalJSON alone is not enough — TransformAttributeMap does not use it.
type MergeAllObjectsConfig struct {
	VisionServices []any  `json:"vision_services"`
	Label          string `json:"label"`

	pcclean.Config
}

// ParseVisionServices normalizes vision_services entries that arrive as either
// strings ("left") or maps ({"name":"left","min_objects":1}).
func ParseVisionServices(raw []any) (VisionServiceSourceList, error) {
	if len(raw) == 0 {
		return nil, nil
	}
	out := make(VisionServiceSourceList, 0, len(raw))
	for i, entry := range raw {
		src, err := parseVisionServiceEntry(i, entry)
		if err != nil {
			return nil, err
		}
		out = append(out, src)
	}
	return out, nil
}

func parseVisionServiceEntry(i int, entry any) (VisionServiceSource, error) {
	switch v := entry.(type) {
	case string:
		if v == "" {
			return VisionServiceSource{}, fmt.Errorf("vision_services[%d]: name is required", i)
		}
		return VisionServiceSource{Name: v, MinObjects: 0}, nil
	case map[string]any:
		name, _ := v["name"].(string)
		if name == "" {
			return VisionServiceSource{}, fmt.Errorf("vision_services[%d]: name is required", i)
		}
		minObjects, err := optionalIntAttr(v, "min_objects")
		if err != nil {
			return VisionServiceSource{}, fmt.Errorf("vision_services[%d]: %w", i, err)
		}
		if minObjects < 0 {
			return VisionServiceSource{}, fmt.Errorf("vision_services[%d]: min_objects must be >= 0", i)
		}
		return VisionServiceSource{Name: name, MinObjects: minObjects}, nil
	default:
		// After JSON round-trip through some paths, maps may be map[string]interface{}
		// already handled above; also accept typed structs from tests.
		b, err := json.Marshal(entry)
		if err != nil {
			return VisionServiceSource{}, fmt.Errorf("vision_services[%d]: must be a string or object with name/min_objects", i)
		}
		var asString string
		if err := json.Unmarshal(b, &asString); err == nil {
			return parseVisionServiceEntry(i, asString)
		}
		var asMap map[string]any
		if err := json.Unmarshal(b, &asMap); err == nil {
			return parseVisionServiceEntry(i, asMap)
		}
		return VisionServiceSource{}, fmt.Errorf("vision_services[%d]: must be a string or object with name/min_objects", i)
	}
}

func optionalIntAttr(m map[string]any, key string) (int, error) {
	raw, ok := m[key]
	if !ok || raw == nil {
		return 0, nil
	}
	switch n := raw.(type) {
	case int:
		return n, nil
	case int32:
		return int(n), nil
	case int64:
		return int(n), nil
	case float64:
		return int(n), nil
	case json.Number:
		i, err := n.Int64()
		return int(i), err
	default:
		return 0, fmt.Errorf("%s must be an integer", key)
	}
}

func (c *MergeAllObjectsConfig) Validate(path string) ([]string, []string, error) {
	sources, err := ParseVisionServices(c.VisionServices)
	if err != nil {
		return nil, nil, err
	}
	if len(sources) == 0 {
		return nil, nil, fmt.Errorf("need at least one vision service")
	}
	return sources.Names(), nil, nil
}

func newMergeAllObjects(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (camera.Camera, error) {
	newConf, err := resource.NativeConfig[*MergeAllObjectsConfig](config)
	if err != nil {
		return nil, err
	}
	pcclean.FillDefaults(&newConf.Config)

	sources, err := ParseVisionServices(newConf.VisionServices)
	if err != nil {
		return nil, err
	}

	cc := &MergeAllObjectsCamera{
		name:    config.ResourceName(),
		cfg:     newConf,
		sources: make([]mergeAllObjectsSource, 0, len(sources)),
		logger:  logger,
	}

	for _, src := range sources {
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
