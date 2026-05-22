// Package pcclean cleans a merged point cloud through an opt-out pipeline of
// statistical outlier removal, largest-connected-component selection, and a
// centroid radius crop. It is consumed by camera components that union many
// noisy point clouds and want to drop the wide ground halo and stray flyers
// before publishing.
package pcclean

import (
	"math"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
)

const (
	DefaultOutlierMeanK               = 50
	DefaultOutlierStdDevThresh        = 2.0
	DefaultClusterMaxDistance         = 10.0
	DefaultClusterMinPointsPerSegment = 5
	DefaultClusterMinPointsPerCluster = 50
	DefaultMaxRadiusFromCenter        = 250.0
)

// Config holds the knobs for the cleaning pipeline. Each stage is skipped
// when its primary knob is <= 0. Zero values get the documented defaults
// applied by FillDefaults so existing configs pick up cleaning automatically;
// set a knob to a negative value to explicitly disable a stage. JSON tags are
// flat so this struct can be anonymously embedded in a component config and
// its keys promoted to the outer JSON namespace.
type Config struct {
	OutlierMeanK        int     `json:"outlier_mean_k,omitempty"`
	OutlierStdDevThresh float64 `json:"outlier_std_dev_thresh,omitempty"`

	ClusterMaxDistance         float64 `json:"cluster_max_distance,omitempty"`
	ClusterMinPointsPerSegment int     `json:"cluster_min_points_per_segment,omitempty"`
	ClusterMinPointsPerCluster int     `json:"cluster_min_points_per_cluster,omitempty"`

	MaxRadiusFromCenter float64 `json:"max_radius_from_center,omitempty"`

	Disable bool `json:"disable_cleaning,omitempty"`
}

// FillDefaults rewrites zero-valued cleaning knobs to the conservative
// defaults. A negative value is preserved so a user can explicitly disable a
// stage without it being silently re-enabled by the default-fill.
func FillDefaults(c *Config) {
	if c.OutlierMeanK == 0 {
		c.OutlierMeanK = DefaultOutlierMeanK
	}
	if c.OutlierStdDevThresh == 0 {
		c.OutlierStdDevThresh = DefaultOutlierStdDevThresh
	}
	if c.ClusterMaxDistance == 0 {
		c.ClusterMaxDistance = DefaultClusterMaxDistance
	}
	if c.ClusterMinPointsPerSegment == 0 {
		c.ClusterMinPointsPerSegment = DefaultClusterMinPointsPerSegment
	}
	if c.ClusterMinPointsPerCluster == 0 {
		c.ClusterMinPointsPerCluster = DefaultClusterMinPointsPerCluster
	}
	if c.MaxRadiusFromCenter == 0 {
		c.MaxRadiusFromCenter = DefaultMaxRadiusFromCenter
	}
}

// applyOutlier runs RDK's StatisticalOutlierFilter and returns the cleaned
// cloud. A non-positive meanK or an input smaller than meanK+1 points makes
// this a no-op (returns the input unchanged) since the kNN math is not
// meaningful in those cases.
func applyOutlier(in pointcloud.PointCloud, meanK int, stdDev float64) (pointcloud.PointCloud, error) {
	if meanK <= 0 || stdDev <= 0 {
		return in, nil
	}
	if in.Size() <= meanK {
		return in, nil
	}
	filter, err := pointcloud.StatisticalOutlierFilter(meanK, stdDev)
	if err != nil {
		return nil, err
	}
	out := pointcloud.NewBasicEmpty()
	if err := filter(in, out); err != nil {
		return nil, err
	}
	return out, nil
}

// keepLargestCluster keeps only the largest dense connected component of
// `in`. It bypasses the general-purpose touch.Cluster() because that function
// is O(S² × |a|×|b|) in the merge phase (pairwise within-distance check
// between every pair of grid buckets), which is prohibitively slow on the
// noisy clouds this camera sees. Since we only need the single largest
// component — not all clusters — a grid + union-find pass is enough and
// runs in linear time. If clustering is disabled (maxDistance <= 0) or no
// component meets the size thresholds, the input is returned unchanged.
func keepLargestCluster(in pointcloud.PointCloud, maxDistance float64, minPointsPerSegment, minPointsPerCluster int) (pointcloud.PointCloud, error) {
	if maxDistance <= 0 || in.Size() == 0 {
		return in, nil
	}
	out, ok, err := findLargestComponent(in, maxDistance, minPointsPerSegment, minPointsPerCluster)
	if err != nil {
		return nil, err
	}
	if !ok {
		return in, nil
	}
	return out, nil
}

// gridKey identifies a 3D voxel cell.
type gridKey struct{ x, y, z int }

func gridKeyFor(p r3.Vector, size float64) gridKey {
	return gridKey{
		x: int(math.Floor(p.X / size)),
		y: int(math.Floor(p.Y / size)),
		z: int(math.Floor(p.Z / size)),
	}
}

// gridCell holds the points that landed in one voxel.
type gridCell struct {
	points []r3.Vector
	data   []pointcloud.Data
}

// findLargestComponent buckets points into voxels of side `cellSize`, drops
// voxels with fewer than `minPointsPerCell` points, then unions every
// remaining voxel with its 26 grid-neighbors. The voxel set forming the
// largest connected component (by point count) is reassembled into a fresh
// PointCloud. The bool return is false when the input is empty or when the
// largest component fails the `minPointsPerComponent` threshold; in those
// cases the caller should fall through to the input rather than yielding an
// empty cloud.
func findLargestComponent(
	in pointcloud.PointCloud,
	cellSize float64,
	minPointsPerCell, minPointsPerComponent int,
) (pointcloud.PointCloud, bool, error) {
	if cellSize <= 0 || in.Size() == 0 {
		return in, false, nil
	}

	cells := make(map[gridKey]*gridCell)
	in.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		k := gridKeyFor(p, cellSize)
		c, ok := cells[k]
		if !ok {
			c = &gridCell{}
			cells[k] = c
		}
		c.points = append(c.points, p)
		c.data = append(c.data, d)
		return true
	})

	if minPointsPerCell > 0 {
		for k, c := range cells {
			if len(c.points) < minPointsPerCell {
				delete(cells, k)
			}
		}
	}
	if len(cells) == 0 {
		return in, false, nil
	}

	parent := make(map[gridKey]gridKey, len(cells))
	for k := range cells {
		parent[k] = k
	}
	find := func(k gridKey) gridKey {
		for parent[k] != k {
			parent[k] = parent[parent[k]] // path halving
			k = parent[k]
		}
		return k
	}
	union := func(a, b gridKey) {
		ra, rb := find(a), find(b)
		if ra != rb {
			parent[ra] = rb
		}
	}

	for k := range cells {
		for dx := -1; dx <= 1; dx++ {
			for dy := -1; dy <= 1; dy++ {
				for dz := -1; dz <= 1; dz++ {
					if dx == 0 && dy == 0 && dz == 0 {
						continue
					}
					nk := gridKey{k.x + dx, k.y + dy, k.z + dz}
					if _, ok := cells[nk]; ok {
						union(k, nk)
					}
				}
			}
		}
	}

	rootCount := make(map[gridKey]int)
	rootCells := make(map[gridKey][]gridKey)
	for k := range cells {
		r := find(k)
		rootCount[r] += len(cells[k].points)
		rootCells[r] = append(rootCells[r], k)
	}

	var bestRoot gridKey
	bestCount := 0
	for r, c := range rootCount {
		if c > bestCount {
			bestCount = c
			bestRoot = r
		}
	}
	if bestCount <= minPointsPerComponent {
		return in, false, nil
	}

	out := pointcloud.NewBasicPointCloud(bestCount)
	for _, k := range rootCells[bestRoot] {
		c := cells[k]
		for i, p := range c.points {
			if err := out.Set(p, c.data[i]); err != nil {
				return nil, false, err
			}
		}
	}
	return out, true, nil
}

// cropToRadius keeps only points within `radius` of the cloud's centroid
// (mean of all points). A non-positive radius makes this a no-op.
func cropToRadius(in pointcloud.PointCloud, radius float64) (pointcloud.PointCloud, error) {
	if radius <= 0 || in.Size() == 0 {
		return in, nil
	}
	center := pointcloud.CloudCentroid(in)
	out := pointcloud.NewBasicEmpty()
	var iterErr error
	in.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if p.Distance(center) > radius {
			return true
		}
		if err := out.Set(p, d); err != nil {
			iterErr = err
			return false
		}
		return true
	})
	if iterErr != nil {
		return nil, iterErr
	}
	return out, nil
}

// Clean runs the full cleaning pipeline on a merged cloud: outlier removal,
// then largest-cluster keep, then radius crop. Each stage is individually
// bypassable via its config knob.
func Clean(pc pointcloud.PointCloud, cfg *Config) (pointcloud.PointCloud, error) {
	if cfg.Disable || pc == nil || pc.Size() == 0 {
		return pc, nil
	}

	out, err := applyOutlier(pc, cfg.OutlierMeanK, cfg.OutlierStdDevThresh)
	if err != nil {
		return nil, err
	}

	out, err = keepLargestCluster(out, cfg.ClusterMaxDistance, cfg.ClusterMinPointsPerSegment, cfg.ClusterMinPointsPerCluster)
	if err != nil {
		return nil, err
	}

	out, err = cropToRadius(out, cfg.MaxRadiusFromCenter)
	if err != nil {
		return nil, err
	}

	return out, nil
}
