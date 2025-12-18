package touch

import (
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/rimage/transform"
)

var RealSenseProperties = camera.Properties{
	IntrinsicParams:  &transform.PinholeCameraIntrinsics{Width: 1280, Height: 720, Fx: 906.0663452148438, Fy: 905.1234741210938, Ppx: 646.94970703125, Ppy: 374.4667663574219},
	DistortionParams: &transform.BrownConrady{RadialK1: 0, RadialK2: 0, RadialK3: 0, TangentialP1: 0, TangentialP2: 0},
}

func PixelToPoint(properties camera.Properties, pixelX, pixelY, depth float64) (float64, float64, float64) {
	x, y, z := properties.IntrinsicParams.PixelToPoint(pixelX, pixelY, depth)

	if properties.DistortionParams != nil {
		x, y = properties.DistortionParams.Transform(x, y)
	}

	return x, y, z
}

func RSPixelToPoint(properties camera.Properties, pixelX, pixelY, depth float64) (float64, float64, float64) {
	intrin := properties.IntrinsicParams

	x := (pixelX - intrin.Ppx) / intrin.Fx
	y := (pixelY - intrin.Ppy) / intrin.Fy

	xo := x
	yo := y

	brown, ok := properties.DistortionParams.(*transform.BrownConrady)
	if ok {
		for i := 0; i < 10; i++ {
			r2 := x*x + y*y
			icdist := 1 / (1 + ((brown.TangentialP2*r2+brown.RadialK2)*r2+brown.RadialK1)*r2)
			delta_x := 2*brown.RadialK3*x*y + brown.TangentialP1*(r2+2*x*x)
			delta_y := 2*brown.TangentialP1*x*y + brown.RadialK3*(r2+2*y*y)
			x = (xo - delta_x) * icdist
			y = (yo - delta_y) * icdist
		}
	} else if properties.DistortionParams != nil {
		x, y = properties.DistortionParams.Transform(x, y)
	}

	return depth * x, depth * y, depth
}
