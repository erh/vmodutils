package touch

import (
	"image"
	"image/png"
	"os"
	"testing"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/vision/objectdetection"
	"go.viam.com/test"
)

func TestPCDetectCrop1(t *testing.T) {
	in, err := pointcloud.NewFromFile("data/glass-top1.pcd", "")
	test.That(t, err, test.ShouldBeNil)

	out, err := PCDetectCrop(in, []objectdetection.Detection{
		objectdetection.NewDetectionWithoutImgBounds(image.Rect(525, 200, 699, 421), .8, "glass"),
	}, RealSenseProperties)

	test.That(t, err, test.ShouldBeNil)

	test.That(t, out.Size(), test.ShouldBeGreaterThan, 0)
	test.That(t, out.Size(), test.ShouldBeLessThan, in.Size())

	img := PCToImage(out)

	file, err := os.Create("glass-project1.png")
	test.That(t, err, test.ShouldBeNil)
	defer file.Close()

	err = png.Encode(file, img)
	test.That(t, err, test.ShouldBeNil)

}
