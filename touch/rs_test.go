package touch

import (
	"image"
	"image/color"
	_ "image/png"
	"os"
	"testing"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/test"
)

func TestRSPixelToPoint(t *testing.T) {

	x, y, z := 592.0, 499.0, 459.601175

	xx, yy, zz := PixelToPoint(RealSenseProperties, x, y, z)
	test.That(t, xx, test.ShouldAlmostEqual, -27.87, .01)
	test.That(t, yy, test.ShouldAlmostEqual, 63.23, .01)
	test.That(t, zz, test.ShouldAlmostEqual, z)

	xxx, yyy, zzz := RSPixelToPoint(RealSenseProperties, x, y, z)
	test.That(t, zzz, test.ShouldAlmostEqual, z)

	// TODO - i think this is wrong??

	test.That(t, xxx, test.ShouldAlmostEqual, -27.87, .01)
	test.That(t, yyy, test.ShouldAlmostEqual, 63.23, .01)

	//test.That(t, xxx, test.ShouldAlmostEqual, -37.042980)
	//test.That(t, yyy, test.ShouldAlmostEqual, 62.105444)

}

func TestRS1(t *testing.T) {
	logger := logging.NewTestLogger(t)

	p := camera.Properties{
		IntrinsicParams:  &transform.PinholeCameraIntrinsics{640, 480, 609.566162109375, 609.5594482421875, 319.1065368652344, 253.16976928710938},
		DistortionParams: &transform.BrownConrady{RadialK1: 0, RadialK2: 0, RadialK3: 0, TangentialP1: 0, TangentialP2: 0},
	}

	bishopPixelX := 137.0
	bishopPixelY := 266.0

	df, err := os.Open("data/rs1/rs-depth.png")
	test.That(t, err, test.ShouldBeNil)
	defer df.Close()

	di, _, err := image.Decode(df)
	test.That(t, err, test.ShouldBeNil)

	did := di.At(int(bishopPixelX), int(bishopPixelY))
	fromDepth := float64(did.(color.Gray16).Y)

	guessX, guessY, _ := PixelToPoint(p, bishopPixelX, bishopPixelY, fromDepth)

	logger.Infof("guessX: %0.2f, guessY: %0.2f", guessX, guessY)

	pc, err := pointcloud.NewFromFile("data/rs1/rs.pcd", "")
	test.That(t, err, test.ShouldBeNil)

	peak := PCFindLowestInRegion(pc, image.Rectangle{
		Min: image.Point{int(guessX - 50), int(guessY - 50)},
		Max: image.Point{int(guessX + 50), int(guessY + 50)},
	})

	test.That(t, peak.Z, test.ShouldAlmostEqual, fromDepth, 1.5)
	logger.Infof("peak: %v", peak)

	newX, newY, _ := PixelToPoint(p, bishopPixelX, bishopPixelY, peak.Z)
	logger.Infof("newX(i): %0.2f, newY: %0.2f", newX, newY)

	newXB, newYB, _ := RSPixelToPoint(p, bishopPixelX, bishopPixelY, peak.Z)
	logger.Infof("newX(b): %0.2f, newY: %0.2f", newX, newY)

	test.That(t, newX, test.ShouldAlmostEqual, newXB)
	test.That(t, newY, test.ShouldAlmostEqual, newYB)

	ppX, ppY := p.IntrinsicParams.PointToPixel(peak.X, peak.Y, peak.Z)
	test.That(t, ppX, test.ShouldAlmostEqual, bishopPixelX, 2.5)
	test.That(t, ppY, test.ShouldAlmostEqual, bishopPixelY, 2.5)

	/*
		for d := 10.0; d < 500; d++ {
			x, y, z := PixelToPoint(p, bishopPixelX, bishopPixelY, d)
			logger.Infof("d: %v x: %0.2f, y: %0.2f, z: %0.2f", d, x, y, z)
		}
	*/

}
