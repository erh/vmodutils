package file_utils

import (
	"bytes"
	"encoding/json"
	"fmt"
	"image"
	"image/jpeg"
	"os"
	"path/filepath"
	"time"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/services/datamanager/builtin/shared"
	rutils "go.viam.com/rdk/utils"
)

func GetPathInCaptureDir(subDirName string) string {
	return filepath.Join(shared.ViamCaptureDotDir, subDirName)
}

// EnsureDirExists creates the target directory path if it does not exist
// using secure default permissions. It is safe to call multiple times.
func EnsureDirExists(dirPath string) error {
	if dirPath == "" {
		return nil
	}

	if err := os.MkdirAll(dirPath, 0o750); err != nil {
		return fmt.Errorf("failed to create directory %q: %w", dirPath, err)
	}

	return nil
}

// EnsureDir checks if a directory exists and creates it if it doesn't.
// This function provides additional validation compared to EnsureDirExists.
func EnsureDir(path string) error {
	info, err := os.Stat(path)
	if os.IsNotExist(err) {
		// directory doesn't exist, create it
		return EnsureDirExists(path)
	}
	if err != nil {
		// some other error accessing the path
		return fmt.Errorf("error checking directory: %w", err)
	}
	if !info.IsDir() {
		// exists but is not a directory
		return fmt.Errorf("%s exists but is not a directory", path)
	}
	// the directory in question already exists
	return nil
}

func SaveFile(b []byte, dirPath, filename string, t time.Time) error {
	// create destination directory if it doesn't exist
	if err := EnsureDir(dirPath); err != nil {
		return fmt.Errorf("failed to create destination directory: %w", err)
	}

	formattedTimestamp := t.Format("January_02_2006_15_04_05")
	dst := filepath.Join(dirPath, fmt.Sprintf("%s_%s", formattedTimestamp, filename))

	if err := os.WriteFile(dst, b, 0o600); err != nil {
		return err
	}

	return nil
}

func SaveJsonFile(data any, dirPath, filename string, t time.Time) error {
	bytes, err := json.Marshal(data)
	if err != nil {
		return err
	}
	return SaveFile(bytes, dirPath, filename, t)
}

func SavePointCloudFile(data pointcloud.PointCloud, dirPath, filename string, t time.Time) error {
	bytes, err := pointcloud.ToBytes(data)
	if err != nil {
		return err
	}
	return SaveFile(bytes, dirPath, filename, t)
}

func SaveImageFile(rawImage image.Image, dirPath, filenameWithoutExtension string, t time.Time) error {
	var imageData []byte
	ext := ".jpeg"

	// Try to get raw data from LazyEncodedImage first (most efficient)
	li, ok := rawImage.(*rimage.LazyEncodedImage)
	if ok {
		if li.MIMEType() != rutils.MimeTypeJPEG {
			ext = ".raw"
		}
		imageData = li.RawData()
	} else {
		// For non-lazy images, encode to JPEG
		var buf bytes.Buffer
		if err := jpeg.Encode(&buf, rawImage, &jpeg.Options{Quality: 90}); err != nil {
			return err
		}
		imageData = buf.Bytes()
	}

	return SaveFile(imageData, dirPath, filenameWithoutExtension+ext, t)
}
