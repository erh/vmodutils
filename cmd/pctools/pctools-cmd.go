package main

import (
	"context"
	"flag"
	"fmt"
	"image/color"
	"image/png"
	"os"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/services/vision"

	"github.com/erh/vmodutils"
	"github.com/erh/vmodutils/touch"
)

func main() {
	err := realMain()
	if err != nil {
		panic(err)
	}
}

func realMain() error {
	logger := logging.NewLogger("cmd-wave")
	ctx := context.Background()

	host := flag.String("host", "", "hostname")
	cmd := flag.String("cmd", "", "command")
	cameraName := flag.String("camera", "", "camera to use")
	visionName := flag.String("vision", "", "vision service")
	out := flag.String("out", "", "output file")
	in := flag.String("in", "", "input file")
	colorDistance := flag.Float64("color-distance", 100, "")
	colorRed := flag.Int("color-red", 255, "")
	colorGreen := flag.Int("color-green", 255, "")
	colorBlue := flag.Int("color-blue", 255, "")

	maxDistance := flag.Float64("max-distance", 30, "")
	minPointsPerSegment := flag.Int("min-points-per-segment", 20, "")
	minPointsPerCluster := flag.Int("min-points-per-cluster", 100, "")

	flag.Parse()

	if *cmd == "" {
		return fmt.Errorf("need a cmd")
	}

	if *cmd == "download" {
		if *out == "" {
			return fmt.Errorf("need an 'out'")
		}

		machine, err := vmodutils.ConnectToHostFromCLIToken(ctx, *host, logger)
		if err != nil {
			return err
		}
		defer machine.Close(ctx)

		myCamera, err := camera.FromRobot(machine, *cameraName)
		if err != nil {
			return err
		}

		pc, err := myCamera.NextPointCloud(ctx, nil)
		if err != nil {
			return err
		}

		return writePCToFile(*out, pc)
	}

	if *cmd == "realsense-all" {
		machine, err := vmodutils.ConnectToHostFromCLIToken(ctx, *host, logger)
		if err != nil {
			return err
		}
		defer machine.Close(ctx)

		myCamera, err := camera.FromRobot(machine, *cameraName)
		if err != nil {
			return err
		}

		pc, err := myCamera.NextPointCloud(ctx, nil)
		if err != nil {
			return err
		}

		f, err := os.OpenFile("rs.pcd", os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0o644)
		if err != nil {
			return err
		}
		defer f.Close()

		err = pointcloud.ToPCD(pc, f, pointcloud.PCDBinary)
		if err != nil {
			return err
		}

		imgs, _, err := myCamera.Images(ctx, nil, nil)
		if err != nil {
			return err
		}

		for _, i := range imgs {
			fn := fmt.Sprintf("rs-%s.png", i.SourceName)

			f, err := os.OpenFile(fn, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0o644)
			if err != nil {
				return err
			}
			defer f.Close()

			theImage, err := i.Image(ctx)
			if err != nil {
				return err
			}

			err = png.Encode(f, theImage)
			if err != nil {
				return fmt.Errorf("cannot write (%s): %w", fn, err)
			}
		}

		props, err := myCamera.Properties(ctx)
		if err != nil {
			return err
		}
		logger.Infof("props - IntrinsicParams %T %v", props.IntrinsicParams, props.IntrinsicParams)
		if props.ExtrinsicParams != nil {
			logger.Infof("props - ExtrinsicParams Translation %v", props.ExtrinsicParams.Translation)
			logger.Infof("props - ExtrinsicParams Orientation %#v", props.ExtrinsicParams.Orientation)
		}
		logger.Infof("props - DistortionParams %T %v", props.DistortionParams, props.DistortionParams)

		return nil
	}

	if *cmd == "size" {
		in, err := pointcloud.NewFromFile(*in, "")
		if err != nil {
			return err
		}
		logger.Infof("size: %d", in.Size())
		return nil
	}

	if *cmd == "image" {
		in, err := pointcloud.NewFromFile(*in, "")
		if err != nil {
			return err
		}
		img := touch.PCToImage(in)
		if *out == "" {
			return fmt.Errorf("need an out")
		}

		return rimage.WriteImageToFile(*out, img)
	}

	if *cmd == "color-filter" {
		in, err := pointcloud.NewFromFile(*in, "")
		if err != nil {
			return err
		}
		min := r3.Vector{-50000, -50000, -50000}
		max := r3.Vector{min.X * -1, min.Y * -1, min.Z * -1}

		filtered := touch.PCCropWithColor(in, min, max, []touch.ColorFilter{
			{color.RGBA{uint8(*colorRed), uint8(*colorGreen), uint8(*colorBlue), 0}, *colorDistance},
		})

		if *out == "" {
			return fmt.Errorf("need an out")
		}

		return writePCToFile(*out, filtered)
	}

	if *cmd == "cluster" {
		in, err := pointcloud.NewFromFile(*in, "")
		if err != nil {
			return err
		}

		clusters, err := touch.Cluster(in, *maxDistance, *minPointsPerSegment, *minPointsPerCluster)
		if err != nil {
			return err
		}

		logger.Infof("got %d clusters", len(clusters))

		for idx, o := range clusters {
			fn := fmt.Sprintf("cluster-%d.pcd", idx)
			err := writePCToFile(fn, o)
			if err != nil {
				return err
			}
			md := o.MetaData()
			logger.Infof("wrote %s size: %d center: %v", fn, o.Size(), md.Center())
		}

		return nil
	}

	if *cmd == "objects" {
		machine, err := vmodutils.ConnectToHostFromCLIToken(ctx, *host, logger)
		if err != nil {
			return err
		}
		defer machine.Close(ctx)

		myVision, err := vision.FromRobot(machine, *visionName)
		if err != nil {
			return err
		}

		objs, err := myVision.GetObjectPointClouds(ctx, "", nil)
		if err != nil {
			return err
		}

		for idx, o := range objs {
			fn := fmt.Sprintf("obj-%s-%d.pcd", o.Geometry.Label(), idx)
			err := writePCToFile(fn, o)
			if err != nil {
				return err
			}
			logger.Infof("wrote %s", fn)
		}

		return nil
	}

	if *cmd == "detections" {
		machine, err := vmodutils.ConnectToHostFromCLIToken(ctx, *host, logger)
		if err != nil {
			return err
		}
		defer machine.Close(ctx)

		myVision, err := vision.FromRobot(machine, *visionName)
		if err != nil {
			return err
		}

		detections, err := myVision.DetectionsFromCamera(ctx, "", nil)
		if err != nil {
			return err
		}

		for idx, d := range detections {
			logger.Infof("detection %d: %v", idx, d)
		}
		return nil
	}

	if *cmd == "object-crop" {
		machine, err := vmodutils.ConnectToHostFromCLIToken(ctx, *host, logger)
		if err != nil {
			return err
		}
		defer machine.Close(ctx)

		myVision, err := vision.FromRobot(machine, *visionName)
		if err != nil {
			return err
		}

		myCamera, err := camera.FromRobot(machine, *cameraName)
		if err != nil {
			return err
		}

		props, err := myCamera.Properties(ctx)
		if err != nil {
			return fmt.Errorf("Properties call failed: %w", err)
		}

		detections, err := myVision.DetectionsFromCamera(ctx, "", nil)
		if err != nil {
			return err
		}

		for idx, d := range detections {
			logger.Infof("detection %d: %v", idx, d)
		}

		pc, err := myCamera.NextPointCloud(ctx, nil)
		if err != nil {
			return err
		}

		sizeBefore := pc.Size()
		pc, err = touch.PCDetectCrop(pc, detections, props)
		if err != nil {
			return err
		}
		logger.Infof("size went from %d to %d", sizeBefore, pc.Size())

		fn := "out.pcd"
		f, err := os.OpenFile(fn, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0o644)
		if err != nil {
			return err
		}
		defer f.Close()

		err = pointcloud.ToPCD(pc, f, pointcloud.PCDBinary)
		if err != nil {
			return err
		}

		logger.Infof("wrote result to %s", fn)

		return nil
	}

	return fmt.Errorf("invalid command [%s]", *cmd)

}

func writePCToFile(fn string, pc pointcloud.PointCloud) error {
	f, err := os.OpenFile(fn, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0o644)
	if err != nil {
		return err
	}
	defer f.Close()

	return pointcloud.ToPCD(pc, f, pointcloud.PCDBinary)
}
