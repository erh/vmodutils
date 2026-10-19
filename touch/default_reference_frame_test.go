package touch

import (
	"context"
	"testing"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
)

func cameraWithFrame(name, frame string) camera.Camera {
	c := inject.NewCamera(name)
	c.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{DefaultReferenceFrame: frame}, nil
	}
	return c
}

func TestMergeCameraDefaultReferenceFrame(t *testing.T) {
	ctx := context.Background()

	agree := &MergeCamera{cameras: []camera.Camera{cameraWithFrame("a", "world"), cameraWithFrame("b", "world")}}
	props, err := agree.Properties(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, props.DefaultReferenceFrame, test.ShouldEqual, "world")

	disagree := &MergeCamera{cameras: []camera.Camera{cameraWithFrame("a", "world"), cameraWithFrame("b", "b")}}
	props, err = disagree.Properties(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, props.DefaultReferenceFrame, test.ShouldEqual, "")
}

func TestCropCameraDefaultReferenceFrame(t *testing.T) {
	ctx := context.Background()
	src := cameraWithFrame("src", "src")

	for _, tc := range []struct {
		name string
		cfg  CropCameraConfig
		want string
	}{
		{"world", CropCameraConfig{Src: "src"}, "world"},
		{"transform back", CropCameraConfig{Src: "src", TransformBackToSourceFrame: true}, "src"},
		{"local with src_frame", CropCameraConfig{Src: "src", SrcFrame: "src-frame", CropInLocalFrame: true}, "src-frame"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			cc := &cropCamera{cfg: &tc.cfg, src: src}
			props, err := cc.Properties(ctx)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, props.DefaultReferenceFrame, test.ShouldEqual, tc.want)
		})
	}
}
