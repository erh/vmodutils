package touch

import (
	"image"
	"image/png"
	"os"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/rimage"
	"go.viam.com/test"
)

func TestSanding2(t *testing.T) {
	logger := logging.NewTestLogger(t)

	imgIn, err := rimage.ReadImageFromFile("data/sanding2.jpg")
	test.That(t, err, test.ShouldBeNil)

	box, _, err := getBoundingBoxBasedOnCenterHSV(imgIn)
	test.That(t, err, test.ShouldBeNil)
	logger.Infof("box: %v", box)

	in, err := pointcloud.NewFromFile("data/sanding2.pcd", "")
	test.That(t, err, test.ShouldBeNil)

	look, err := PCLimitToImageBoxes(in, []*image.Rectangle{box}, nil, RealSenseProperties)
	test.That(t, err, test.ShouldBeNil)

	logger.Infof("size: %d", look.Size())

	test.That(t, look.Size(), test.ShouldBeLessThanOrEqualTo, 538967)

	img := PCToImage(look)

	file, err := os.Create("sanding2.png")
	test.That(t, err, test.ShouldBeNil)
	defer file.Close()

	err = png.Encode(file, img)
	test.That(t, err, test.ShouldBeNil)
}
