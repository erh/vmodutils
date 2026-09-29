package touch

import (
	"context"
	"testing"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
)

func TestDebugImages(t *testing.T) {
	ctx := context.Background()

	fakeCamera := func(sources ...string) (*inject.Camera, *[][]string) {
		var calls [][]string
		cam := inject.NewCamera("cam")
		cam.ImagesFunc = func(_ context.Context, filter []string, _ map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
			calls = append(calls, filter)
			var out []camera.NamedImage
			for _, s := range sources {
				if len(filter) == 0 || filter[0] == s {
					out = append(out, camera.NamedImage{SourceName: s})
				}
			}
			return out, resource.ResponseMetadata{}, nil
		}
		return cam, &calls
	}

	t.Run("camera with a color source is asked for color only", func(t *testing.T) {
		cam, calls := fakeCamera("color", "depth")
		images, _, err := debugImages(ctx, cam)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(images), test.ShouldEqual, 1)
		test.That(t, images[0].SourceName, test.ShouldEqual, "color")
		test.That(t, *calls, test.ShouldResemble, [][]string{{"color"}})
	})

	t.Run("camera without a color source falls back to all sources", func(t *testing.T) {
		cam, calls := fakeCamera("rgb")
		images, _, err := debugImages(ctx, cam)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(images), test.ShouldEqual, 1)
		test.That(t, images[0].SourceName, test.ShouldEqual, "rgb")
		test.That(t, *calls, test.ShouldResemble, [][]string{{"color"}, nil})
	})
}
