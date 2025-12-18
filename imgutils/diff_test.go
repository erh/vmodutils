package imgutils

import (
	"math"
	"testing"

	"go.viam.com/rdk/rimage"
	"go.viam.com/test"
)

func TestDiff1(t *testing.T) {
	img1, err := rimage.ReadImageFromFile("data/diff1a.jpg")
	test.That(t, err, test.ShouldBeNil)

	img2, err := rimage.ReadImageFromFile("data/diff1b.jpg")
	test.That(t, err, test.ShouldBeNil)

	a := ComputeGrayscaleAverage(img1)
	b := ComputeGrayscaleAverage(img2)

	diff := math.Abs(a - b)
	test.That(t, diff, test.ShouldBeGreaterThan, .1)
	test.That(t, diff, test.ShouldBeLessThan, 1)
}
