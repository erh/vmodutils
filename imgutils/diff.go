package imgutils

import (
	"image"
	"image/color"
)

func ComputeGrayscaleAverage(img image.Image) float64 {
	bounds := img.Bounds()

	totalValue := 0.0
	numPixels := 0.0

	for y := bounds.Min.Y; y < bounds.Max.Y; y++ {
		for x := bounds.Min.X; x < bounds.Max.X; x++ {
			grayColor := color.GrayModel.Convert(img.At(x, y)).(color.Gray)
			totalValue += float64(grayColor.Y)
			numPixels++
		}
	}

	return totalValue / numPixels
}
