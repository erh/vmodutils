package pcclean

import (
	"math"
	"math/rand"
	"testing"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/test"
)

// makeSphereCluster generates `n` points uniformly distributed inside a
// sphere of `radius` mm centered at `center`. Deterministic for a given seed.
func makeSphereCluster(seed int64, n int, center r3.Vector, radius float64) pointcloud.PointCloud {
	r := rand.New(rand.NewSource(seed))
	pc := pointcloud.NewBasicEmpty()
	for i := 0; i < n; i++ {
		// Rejection-sample a unit cube down to the unit sphere so density is uniform.
		var x, y, z float64
		for {
			x = r.Float64()*2 - 1
			y = r.Float64()*2 - 1
			z = r.Float64()*2 - 1
			if x*x+y*y+z*z <= 1 {
				break
			}
		}
		p := r3.Vector{X: center.X + x*radius, Y: center.Y + y*radius, Z: center.Z + z*radius}
		_ = pc.Set(p, pointcloud.NewBasicData())
	}
	return pc
}

// addSlab scatters `n` points across an axis-aligned XY slab of width/depth
// `size` mm centered at `center`, with vertical thickness ±`zJitter` mm.
// Density is intentionally low — this is the "ground halo" the cleaning
// pipeline should reject.
func addSlab(into pointcloud.PointCloud, seed int64, n int, center r3.Vector, size, zJitter float64) {
	r := rand.New(rand.NewSource(seed))
	for i := 0; i < n; i++ {
		p := r3.Vector{
			X: center.X + (r.Float64()*2-1)*size/2,
			Y: center.Y + (r.Float64()*2-1)*size/2,
			Z: center.Z + (r.Float64()*2-1)*zJitter,
		}
		_ = into.Set(p, pointcloud.NewBasicData())
	}
}

// addBlob places a small dense blob far from the origin to exercise outlier
// and cluster filters.
func addBlob(into pointcloud.PointCloud, seed int64, n int, center r3.Vector, radius float64) {
	r := rand.New(rand.NewSource(seed))
	for i := 0; i < n; i++ {
		p := r3.Vector{
			X: center.X + (r.Float64()*2-1)*radius,
			Y: center.Y + (r.Float64()*2-1)*radius,
			Z: center.Z + (r.Float64()*2-1)*radius,
		}
		_ = into.Set(p, pointcloud.NewBasicData())
	}
}

// mergeAll copies every input point into a fresh BasicEmpty cloud.
func mergeAll(in ...pointcloud.PointCloud) pointcloud.PointCloud {
	out := pointcloud.NewBasicEmpty()
	for _, pc := range in {
		pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
			_ = out.Set(p, d)
			return true
		})
	}
	return out
}

// nonEmptyTotal counts points that survive into `pc`.
func nonEmptyTotal(pc pointcloud.PointCloud) int {
	if pc == nil {
		return 0
	}
	return pc.Size()
}

func TestFillDefaults(t *testing.T) {
	t.Run("zero values get defaults", func(t *testing.T) {
		c := &Config{}
		FillDefaults(c)
		test.That(t, c.OutlierMeanK, test.ShouldEqual, DefaultOutlierMeanK)
		test.That(t, c.OutlierStdDevThresh, test.ShouldEqual, DefaultOutlierStdDevThresh)
		test.That(t, c.ClusterMaxDistance, test.ShouldEqual, DefaultClusterMaxDistance)
		test.That(t, c.ClusterMinPointsPerSegment, test.ShouldEqual, DefaultClusterMinPointsPerSegment)
		test.That(t, c.ClusterMinPointsPerCluster, test.ShouldEqual, DefaultClusterMinPointsPerCluster)
		test.That(t, c.MaxRadiusFromCenter, test.ShouldEqual, DefaultMaxRadiusFromCenter)
	})

	t.Run("negative values are preserved", func(t *testing.T) {
		c := &Config{
			OutlierMeanK:        -1,
			ClusterMaxDistance:  -1,
			MaxRadiusFromCenter: -1,
		}
		FillDefaults(c)
		test.That(t, c.OutlierMeanK, test.ShouldEqual, -1)
		test.That(t, c.ClusterMaxDistance, test.ShouldEqual, -1.0)
		test.That(t, c.MaxRadiusFromCenter, test.ShouldEqual, -1.0)
		// Untouched zero fields still get their defaults.
		test.That(t, c.OutlierStdDevThresh, test.ShouldEqual, DefaultOutlierStdDevThresh)
	})
}

func TestApplyOutlier(t *testing.T) {
	t.Run("drops far isolated noise", func(t *testing.T) {
		// 500 dense points + a handful of far flyers. The flyers will have
		// huge avg-kNN distance and should be filtered out.
		pc := makeSphereCluster(1, 500, r3.Vector{}, 30)
		addBlob(pc, 2, 5, r3.Vector{X: 5000, Y: 5000, Z: 5000}, 5)
		before := pc.Size()

		out, err := applyOutlier(pc, 50, 1.0)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, out.Size(), test.ShouldBeLessThan, before)
		// Far flyers must be gone.
		out.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
			test.That(t, p.Norm(), test.ShouldBeLessThan, 1000.0)
			return true
		})
	})

	t.Run("noop when meanK <= 0", func(t *testing.T) {
		pc := makeSphereCluster(1, 100, r3.Vector{}, 10)
		out, err := applyOutlier(pc, 0, 1.0)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, out.Size(), test.ShouldEqual, pc.Size())
	})

	t.Run("noop when input smaller than meanK", func(t *testing.T) {
		pc := makeSphereCluster(1, 10, r3.Vector{}, 10)
		out, err := applyOutlier(pc, 50, 1.0)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, out.Size(), test.ShouldEqual, pc.Size())
	})
}

func TestKeepLargestCluster(t *testing.T) {
	t.Run("keeps dense object, drops disconnected halo", func(t *testing.T) {
		object := makeSphereCluster(1, 600, r3.Vector{}, 25)
		halo := pointcloud.NewBasicEmpty()
		// Spread the slab far enough from the object that grid-based cluster
		// merging won't chain it to the dense cluster, but keep it sparse so
		// individual grid buckets fall below min-points-per-segment.
		addSlab(halo, 2, 80, r3.Vector{X: 400, Y: 0, Z: 0}, 600, 1)
		merged := mergeAll(object, halo)

		out, err := keepLargestCluster(merged, 10, 5, 50)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, out.Size(), test.ShouldBeGreaterThan, 0)
		test.That(t, out.Size(), test.ShouldBeLessThan, merged.Size())
		// Whatever survives should be near the object centroid.
		out.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
			test.That(t, p.Norm(), test.ShouldBeLessThan, 100.0)
			return true
		})
	})

	t.Run("noop when maxDistance <= 0", func(t *testing.T) {
		pc := makeSphereCluster(1, 50, r3.Vector{}, 10)
		out, err := keepLargestCluster(pc, 0, 5, 5)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, out.Size(), test.ShouldEqual, pc.Size())
	})

	t.Run("falls through when no clusters meet thresholds", func(t *testing.T) {
		// Sparse points with a min-points threshold nothing meets — should
		// pass through rather than collapse to empty.
		pc := makeSphereCluster(1, 20, r3.Vector{}, 10)
		out, err := keepLargestCluster(pc, 1, 1000, 1000)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, out.Size(), test.ShouldEqual, pc.Size())
	})
}

func TestCropToRadius(t *testing.T) {
	t.Run("trims points outside radius", func(t *testing.T) {
		pc := makeSphereCluster(1, 400, r3.Vector{}, 30)
		addBlob(pc, 2, 50, r3.Vector{X: 500, Y: 0, Z: 0}, 5)

		out, err := cropToRadius(pc, 100)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, out.Size(), test.ShouldBeLessThan, pc.Size())
		// Centroid is pulled toward the off-center blob, but everything kept
		// must be within radius of that centroid by construction.
		center := pointcloud.CloudCentroid(pc)
		out.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
			test.That(t, p.Distance(center), test.ShouldBeLessThanOrEqualTo, 100.0)
			return true
		})
	})

	t.Run("noop when radius <= 0", func(t *testing.T) {
		pc := makeSphereCluster(1, 50, r3.Vector{}, 10)
		out, err := cropToRadius(pc, 0)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, out.Size(), test.ShouldEqual, pc.Size())
	})
}

func TestCleanFullPipeline(t *testing.T) {
	// Build the scene from the real picture: a dense object cluster, a wide
	// horizontal table-plane halo, and a small far-away blob.
	object := makeSphereCluster(1, 600, r3.Vector{X: 0, Y: 0, Z: 0}, 25)
	addSlab(object, 2, 200, r3.Vector{X: 600, Y: 0, Z: 0}, 800, 1)
	addBlob(object, 3, 10, r3.Vector{X: 4000, Y: 4000, Z: 0}, 5)

	cfg := &Config{}
	FillDefaults(cfg)

	out, err := Clean(object, cfg)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out.Size(), test.ShouldBeGreaterThan, 0)
	test.That(t, out.Size(), test.ShouldBeLessThan, object.Size())

	// Anything that survives should be near the original dense object center.
	out.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		test.That(t, math.Sqrt(p.X*p.X+p.Y*p.Y+p.Z*p.Z), test.ShouldBeLessThan, 200.0)
		return true
	})
}

func TestCleanDisable(t *testing.T) {
	pc := makeSphereCluster(1, 100, r3.Vector{}, 10)
	addBlob(pc, 2, 20, r3.Vector{X: 5000}, 5)

	cfg := &Config{Disable: true}
	FillDefaults(cfg)

	out, err := Clean(pc, cfg)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, nonEmptyTotal(out), test.ShouldEqual, pc.Size())
}

func TestCleanEmpty(t *testing.T) {
	cfg := &Config{}
	FillDefaults(cfg)
	empty := pointcloud.NewBasicEmpty()
	out, err := Clean(empty, cfg)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out.Size(), test.ShouldEqual, 0)
}

// noisySAM2Like builds a synthetic cloud roughly the shape of what
// sam2-merged-segments returns: a dense object cluster (~6k points), a wide
// ground halo of scattered noise points across a meter-scale slab, and a
// handful of isolated far blobs.
func noisySAM2Like() pointcloud.PointCloud {
	pc := makeSphereCluster(1, 6000, r3.Vector{}, 30)
	addSlab(pc, 2, 4000, r3.Vector{X: 200, Y: 200, Z: 0}, 1200, 5)
	addBlob(pc, 3, 50, r3.Vector{X: 3000, Y: -2000, Z: 100}, 30)
	addBlob(pc, 4, 40, r3.Vector{X: -2500, Y: 1500, Z: -50}, 25)
	return pc
}

func BenchmarkKeepLargestCluster_Fast(b *testing.B) {
	pc := noisySAM2Like()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, err := keepLargestCluster(pc, DefaultClusterMaxDistance, DefaultClusterMinPointsPerSegment, DefaultClusterMinPointsPerCluster)
		if err != nil {
			b.Fatal(err)
		}
	}
}
