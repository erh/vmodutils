# vmodutils Module

<p align="center">
  <a href="https://pkg.go.dev/github.com/erh/vmodutils"><img src="https://pkg.go.dev/badge/github.com/erh/vmodutils" alt="PkgGoDev"></a>
</p>

The `erh:vmodutils` module bundles a few utility models for arm-based automation, point-cloud processing, and data-capture orchestration:

1. **`erh:vmodutils:pc-crop-camera`** — A camera that crops a source point cloud to an axis-aligned world-frame box, with optional RGB color filtering.
2. **`erh:vmodutils:pc-detect-crop-camera`** — A camera that crops a point cloud to the 2D bounding boxes returned by a vision service's detections.
3. **`erh:vmodutils:pc-merge`** — A camera that fetches point clouds from a list of source cameras and merges them into one.
4. **`erh:vmodutils:pc-look-at-crop-camera`** — A camera that isolates the cluster of points closest to the camera centerline (with optional center-color matching).
5. **`erh:vmodutils:pc-multiple-arm-poses`** — A camera that drives a list of arm-position switches through their saved poses, captures a point cloud at each, and merges them in the world frame.
6. **`erh:vmodutils:pc-cluster`** — A vision service that segments a camera's point cloud into spatial clusters and returns them as `ObjectPointClouds`.
7. **`erh:vmodutils:arm-position-saver`** — A switch that records the current arm pose into module config and later moves the arm back to it.
8. **`erh:vmodutils:multi-arm-position-switch`** — A switch that moves an arm between a configured list of joint positions, one per switch index.
9. **`erh:vmodutils:obstacle`** — A gripper that doesn't grip — it just publishes a set of static `Geometries` so the motion service can avoid them.
10. **`erh:vmodutils:obstacle-open-box`** — A gripper that publishes the five faces of an open-top box as obstacle geometries, and can drive an arm to the box opening.
11. **`erh:vmodutils:calibration-checker`** — A sensor that compares world-frame positions of a shared AprilTag across two or more pose trackers to detect arm/camera drift.
12. **`erh:vmodutils:session-capture`** — A sensor wired into the data manager's `capture_control_sensor` that toggles capture on/off for a list of components and tags clips with a session id.

---

## Model: `erh:vmodutils:pc-crop-camera`

**API:** `rdk:component:camera`

Wraps a source camera and crops its point cloud to an axis-aligned bounding box specified in the **world frame**. Optionally filters points by RGB color similarity.

### Configuration

```json
{
  "src": "<string>",
  "src_frame": "<string>",
  "min": { "X": 0, "Y": 0, "Z": 0 },
  "max": { "X": 100, "Y": 100, "Z": 100 },
  "good_colors": [
    { "Color": { "R": 255, "G": 0, "B": 0, "A": 255 }, "Distance": 50 }
  ],
  "transform_back_to_source_frame": false,
  "forward_source_images": false
}
```

| Name                             | Type   | Required | Description                                                                                                                                                                                                            |
| -------------------------------- | ------ | -------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `src`                            | string | Yes      | Name of the source camera.                                                                                                                                                                                             |
| `src_frame`                      | string | No       | Frame the source point cloud is in. Defaults to the source camera's name. Points are transformed from this to `world`.                                                                                                 |
| `min`                            | vector | No       | Lower-bound `(X, Y, Z)` of the crop box in the world frame.                                                                                                                                                            |
| `max`                            | vector | No       | Upper-bound `(X, Y, Z)` of the crop box in the world frame.                                                                                                                                                            |
| `good_colors`                    | array  | No       | RGB color filters. A point is kept only if its color is within `Distance` (Euclidean RGB) of every listed `Color`.                                                                                                     |
| `transform_back_to_source_frame` | bool   | No       | If `true`, after cropping in world coordinates the point cloud is transformed back into `src_frame`, and `Properties` forwards the source camera's `IntrinsicParams` / `DistortionParams`. Defaults to `false`.        |
| `forward_source_images`          | bool   | No       | If `true`, `Images` appends the source camera's `NamedImage`s after the `cropped` image so downstream consumers expecting `color` / `depth` still get them. Defaults to `false`.                                       |

The cropped point cloud is exposed via `NextPointCloud`. `Images` returns the cropped 2D PNG as the first `NamedImage` (named `cropped`); when `forward_source_images` is `true` it is followed by whatever the source camera's `Images` call returns.

---

## Model: `erh:vmodutils:pc-detect-crop-camera`

**API:** `rdk:component:camera`

Runs a vision service's detector against the source camera, then crops the source point cloud to the union of the detection bounding boxes by projecting points back into the image plane. The source camera **must** publish `IntrinsicParams`.

### Configuration

```json
{
  "src": "<string>",
  "service": "<string>"
}
```

| Name      | Type   | Required | Description                                               |
| --------- | ------ | -------- | --------------------------------------------------------- |
| `src`     | string | Yes      | Name of the source camera. Must expose camera intrinsics. |
| `service` | string | Yes      | Name of the vision service used for detections.           |

---

## Model: `erh:vmodutils:object-pc-merge`

**API:** `rdk:component:camera`

Calls `GetObjectPointClouds` on a list of vision services, optionally filters by label, merges the resulting per-object point clouds into one, and runs an opt-out cleaning pipeline (statistical outlier removal → largest connected component → radius crop) to drop ground-plane halo and stray noise. Exposes the cleaned cloud via `NextPointCloud` and a 2D projection via `Images`.

### Configuration

```json
{
  "vision_services": ["<vision service 1>", "<vision service 2>"],
  "label": "<optional label filter>",

  "outlier_mean_k": 50,
  "outlier_std_dev_thresh": 2.0,
  "cluster_max_distance": 10,
  "cluster_min_points_per_segment": 5,
  "cluster_min_points_per_cluster": 50,
  "max_radius_from_center": 250,
  "disable_cleaning": false
}
```

| Name | Type | Required | Default | Description |
| ---- | ---- | -------- | ------- | ----------- |
| `vision_services` | string list | Yes | — | Source vision services. Each must implement `GetObjectPointClouds`. |
| `label` | string | No | "" | If set, only objects whose `Geometry.Label()` equals this string are merged. |
| `outlier_mean_k` | int | No | 50 | `meanK` for the statistical outlier filter. Set `<= 0` to disable this stage. |
| `outlier_std_dev_thresh` | float | No | 2.0 | StdDev multiplier for the outlier filter — points whose mean kNN distance exceeds `mean + this * stddev` are dropped. |
| `cluster_max_distance` | float (mm) | No | 10 | Voxel cell size for the largest-connected-component step. Two voxels with `>= cluster_min_points_per_segment` points are connected if they are 26-grid-neighbors. Set `<= 0` to disable this stage. |
| `cluster_min_points_per_segment` | int | No | 5 | Voxels with fewer points are dropped before connectivity, so sparse noise can't bridge clusters. |
| `cluster_min_points_per_cluster` | int | No | 50 | Minimum size of the largest component. If no component meets this, the input is passed through unchanged (avoids returning an empty cloud for sparse-but-valid scenes). |
| `max_radius_from_center` | float (mm) | No | 250 | Final radius crop around the centroid of the surviving cloud. Set `<= 0` to disable this stage. |
| `disable_cleaning` | bool | No | false | If `true`, bypass all three cleaning stages and return the raw merged cloud. |

Defaults are tuned conservatively for cup/bottle-sized objects on a tabletop. Set any numeric knob to a negative value (e.g. `-1`) to explicitly disable that stage; setting it to `0` re-applies the default.

---

## Model: `erh:vmodutils:pc-merge`

**API:** `rdk:component:camera`

Wraps a list of source cameras, calls `NextPointCloud` on each, and returns the union as a single point cloud. No cleaning or transformation is applied — points are emitted in whatever frame each source camera publishes them in. Exposes the merged cloud via `NextPointCloud` and a 2D projection via `Images`.

### Configuration

```json
{
  "cameras": ["<camera1>", "<camera2>"]
}
```

| Name      | Type        | Required | Description                                                 |
| --------- | ----------- | -------- | ----------------------------------------------------------- |
| `cameras` | string list | Yes      | One or more source camera names. Must contain at least one. |

---

## Model: `erh:vmodutils:pc-look-at-crop-camera`

**API:** `rdk:component:camera`

Finds the point in the source point cloud closest to the camera's centerline (smallest `(X² + Y²)` with `Z > 20`), then grows a connected cluster of nearby points around it using a coarse spatial bucketing. Useful for isolating "the thing in front of the camera" without configuring an explicit crop box.

### Configuration

```json
{
  "src": "<string>",
  "use_color": false
}
```

| Name        | Type   | Required | Description                                                                                                                                                 |
| ----------- | ------ | -------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `src`       | string | Yes      | Name of the source camera. Must expose camera intrinsics when `use_color` is `true`.                                                                        |
| `use_color` | bool   | No       | If `true`, also flood-fills a region in the 2D image around the center pixel by HSV bucket and restricts the point cloud to those pixels before clustering. |

---

## Model: `erh:vmodutils:pc-multiple-arm-poses`

**API:** `rdk:component:camera`

Iterates over a list of switch components (typically `arm-position-saver` or `multi-arm-position-switch`), drives each to its "go to" position, captures a point cloud from the source camera at each pose, transforms each into the world frame, and merges them.

For every switch in `positions`, the model calls `SetPosition(2, ...)` (the "go to" position on `arm-position-saver`) — so any switch wired in must accept index `2` as a "move to saved pose" command.

### Configuration

```json
{
  "src": "<string>",
  "sleep_seconds": 1.0,
  "positions": ["<switch1>", "<switch2>"]
}
```

| Name            | Type     | Required | Description                                                                                                   |
| --------------- | -------- | -------- | ------------------------------------------------------------------------------------------------------------- |
| `src`           | string   | Yes      | Source camera. The point cloud captured here will be transformed into the world frame using the frame system. |
| `sleep_seconds` | float    | No       | Seconds to wait after each move before capturing, to let vibrations settle. Defaults to `1`.                  |
| `positions`     | string[] | Yes      | Names of switch components to drive. At least one is required.                                                |

`Images` is **not** supported; only `NextPointCloud`.

---

## Model: `erh:vmodutils:pc-cluster`

**API:** `rdk:service:vision`

Vision service that segments a camera's point cloud into spatial clusters: points are bucketed into a 3D grid of `max-distance` cells, each non-empty bucket becomes a segment, and segments whose closest pair of points are within `max-distance` are iteratively merged. Only `GetObjectPointClouds` is implemented — the other vision methods return errors.

### Configuration

```json
{
  "camera": "<string>",
  "max-distance": 20.0,
  "min-points-per-segment": 5,
  "min-points-per-cluster": 50
}
```

| Name                     | Type   | Required | Description                                                                     |
| ------------------------ | ------ | -------- | ------------------------------------------------------------------------------- |
| `camera`                 | string | Yes      | Source camera providing the point cloud.                                        |
| `max-distance`           | float  | Yes      | Both the bucket size and the merge threshold (must be `> 0`).                   |
| `min-points-per-segment` | int    | Yes      | Minimum points a bucket must contain to be considered a candidate segment.      |
| `min-points-per-cluster` | int    | Yes      | Minimum points a final merged cluster must contain to be returned as an object. |

---

## Model: `erh:vmodutils:arm-position-saver`

**API:** `rdk:component:switch`

A 3-position switch that records and replays a single arm pose. The "saved" pose is stored directly in the component's cloud config — so it persists across reboots — and is replayed via the motion service or directly via the arm.

The switch has three positions:

| Index | Name            | Effect                                                                                                                                            |
| ----- | --------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0     | `idle`          | No-op.                                                                                                                                            |
| 1     | `update config` | Reads the current arm state and writes it back to this component's cloud config (`joints` if `motion` is unset, otherwise `point`+`orientation`). |
| 2     | `go to`         | Moves the arm to the saved pose. Returns the switch to `idle` on completion.                                                                      |

Replay strategy:
- If `joints` is set and `motion` is configured → `motion.Move` with joint goals.
- If `joints` is set and `motion` is unset → `arm.MoveToJointPositions`.
- If `point`/`orientation` are set and `motion` is configured → cartesian `motion.Move`.

### Configuration

```json
{
  "arm": "<string>",
  "motion": "<string>",
  "joints": [0, 0, 0, 0, 0, 0],
  "point": { "X": 0, "Y": 0, "Z": 500 },
  "orientation": { "OX": 0, "OY": 0, "OZ": 1, "Theta": 0 },
  "vision_services": ["<string>"],
  "extra": { },
  "constraints": { }
}
```

| Name              | Type     | Required | Description                                                                                                      |
| ----------------- | -------- | -------- | ---------------------------------------------------------------------------------------------------------------- |
| `arm`             | string   | Yes      | Name of the arm to record and move.                                                                              |
| `motion`          | string   | No       | Motion service name (typically `"builtin"`). When unset, the switch uses `arm.MoveToJointPositions` directly.    |
| `joints`          | float[]  | No       | Saved joint positions (radians). Populated automatically by the "update config" position when `motion` is unset. |
| `point`           | vector   | No       | Saved cartesian point (mm). Populated automatically by "update config" when `motion` is set.                     |
| `orientation`     | object   | No       | Saved orientation as an `OrientationVectorDegrees` (`OX`, `OY`, `OZ`, `Theta`).                                  |
| `vision_services` | string[] | No       | Vision services whose `GetObjectPointClouds` results are added to the world state passed to the motion service.  |
| `extra`           | object   | No       | Arbitrary `extra` map forwarded to `motion.Move` / `arm.MoveToJointPositions`. May not contain `goal_state`.     |
| `constraints`     | object   | No       | Motion constraints forwarded to `motion.Move` (only used when `motion` is set).                                  |

### DoCommand

**`cfg`** — Return the saved configuration.

```json
{ "cfg": true }
```

Returns:

```json
{
  "joints": [0, 0, 0, 0, 0, 0],
  "point": { "X": 0, "Y": 0, "Z": 500 },
  "orientation": { "OX": 0, "OY": 0, "OZ": 1, "Theta": 0 },
  "as_json": "<full config as JSON string>"
}
```

---

## Model: `erh:vmodutils:multi-arm-position-switch`

**API:** `rdk:component:switch`

A switch with one position per pre-configured joint goal. `SetPosition(i)` moves the arm to `joints_list[i]`. Only one move can be in flight at a time — concurrent calls return an error.

When `write_files_to_capture_directory` is enabled, every move writes the config, goal, and final joint positions to the data-manager capture directory. If a `traceID` is propagated through the request context, files are placed under a `tag=<traceID>` subdirectory.

### Configuration

```json
{
  "arm": "<string>",
  "motion": "<string>",
  "joints_list": [[0, 0, 0, 0, 0, 0], [1.57, 0, 0, 0, 0, 0]],
  "vision_services": ["<string>"],
  "extra": { },
  "constraints": { },
  "write_files_to_capture_directory": false
}
```

| Name                               | Type      | Required | Description                                                                                                              |
| ---------------------------------- | --------- | -------- | ------------------------------------------------------------------------------------------------------------------------ |
| `arm`                              | string    | Yes      | Name of the arm to move.                                                                                                 |
| `joints_list`                      | float[][] | Yes      | Ordered list of joint goals. The switch exposes `len(joints_list)` positions named `"go to 0"`, `"go to 1"`, …           |
| `motion`                           | string    | No       | Motion service name (typically `"builtin"`). When unset, uses `arm.MoveToJointPositions`.                                |
| `vision_services`                  | string[]  | No       | Vision services whose `GetObjectPointClouds` results are added to the world state passed to the motion service.          |
| `extra`                            | object    | No       | Arbitrary `extra` map forwarded to motion. May not contain `goal_state`.                                                 |
| `constraints`                      | object    | No       | Motion constraints forwarded to `motion.Move` (only used when `motion` is set).                                          |
| `write_files_to_capture_directory` | bool      | No       | When `true`, persists config, goal, and actual joint values to the capture directory on every move. Defaults to `false`. |

`DoCommand` is not implemented.

---

## Model: `erh:vmodutils:obstacle`

**API:** `rdk:component:gripper`

Publishes a configurable set of static `spatialmath.Geometry` objects so the motion service treats them as obstacles. Configure this component with a `frame` to position the obstacles in the world. `Grab` and `Open` always return errors — this gripper does not move.

### Configuration

```json
{
  "geometries": [
    { "type": "box", "x": 100, "y": 100, "z": 100 },
    { "type": "sphere", "r": 100 }
  ]
}
```

| Name         | Type  | Required | Description                                                                                  |
| ------------ | ----- | -------- | -------------------------------------------------------------------------------------------- |
| `geometries` | array | Yes      | One or more `spatialmath.GeometryConfig` entries. See the RDK `spatialmath` docs for fields. |

---

## Model: `erh:vmodutils:obstacle-open-box`

**API:** `rdk:component:gripper`

A specialised obstacle gripper that publishes the **five faces of an open-top box** (floor, front, back, left, right) as obstacle geometries. Configure with a `frame` to place the box in the world. Optionally drives a target component (e.g. a real gripper or item) to the box opening via `Grab`.

### Configuration

```json
{
  "length": 200,
  "width": 200,
  "height": 150,
  "thickness": 1,
  "to_move": "<string>",
  "motion": "<string>",
  "offset": 50
}
```

| Name        | Type   | Required | Description                                                                                                                         |
| ----------- | ------ | -------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| `length`    | float  | Yes      | Box length along the X axis (mm).                                                                                                   |
| `width`     | float  | Yes      | Box width along the Y axis (mm).                                                                                                    |
| `height`    | float  | Yes      | Box height along the Z axis (mm).                                                                                                   |
| `thickness` | float  | No       | Wall thickness (mm). Defaults to `1`.                                                                                               |
| `to_move`   | string | No       | Name of the component to drive into the box opening when `Grab` is called.                                                          |
| `motion`    | string | No       | Motion service used by `Grab`. Required if `to_move` is set; defaults to `"builtin"`.                                               |
| `offset`    | float  | No       | Vertical offset (mm) added to the box's world-frame origin when computing the destination of `to_move` in `Grab`. Defaults to `50`. |

`Grab` moves `to_move` to a pose `offset` mm above the box origin, with `OZ = -1` and `Theta` copied from `to_move`'s current orientation, under a 180° orientation constraint. It always returns `false` for "did grab" (the box doesn't actually pick anything up).

---

## Model: `erh:vmodutils:calibration-checker`

**API:** `rdk:component:sensor`

Asks two or more `posetracker` components for the same AprilTag, transforms each tracker's reported tag pose into the world frame using the frame system, and computes the maximum pairwise distance between the resulting points. If the distance exceeds `tolerance_mm`, the reading flags a calibration failure.

Useful for detecting arm or camera drift: if multiple cameras observe the same fiducial, all transforms should agree on its world-frame position.

### Configuration

```json
{
  "pose_trackers": ["<tracker1>", "<tracker2>"],
  "tag_id": "0",
  "tolerance_mm": 10
}
```

| Name            | Type     | Required | Description                                                                                           |
| --------------- | -------- | -------- | ----------------------------------------------------------------------------------------------------- |
| `pose_trackers` | string[] | Yes      | At least **two** pose-tracker component names that should all see the same tag.                       |
| `tag_id`        | string   | No       | Tag identifier to look up in each tracker's `Poses` result. Defaults to `"0"`.                        |
| `tolerance_mm`  | float    | No       | Maximum allowed pairwise distance (mm) between trackers' world-frame tag positions. Defaults to `10`. |

### Readings

`Readings` (and `DoCommand` with any payload) returns a flat map containing, for each tracker:

- `<tracker>_visible`: bool — whether the tag was reported.
- `<tracker>_frame`: string — parent frame of the reported tag pose.
- `<tracker>_x` / `_y` / `_z`: float — world-frame coordinates (mm).
- `<tracker>_error` / `<tracker>_transform_error`: string — error message if the tracker call or frame-system transform failed.

Plus the aggregate fields:

```json
{
  "max_distance_mm": 7.3,
  "tolerance_mm": 10,
  "calibration_ok": true
}
```

When `calibration_ok` is `false`, an `error` field is included describing which two trackers diverged the most. When fewer than two trackers see the tag, `calibration_ok` is `true` and a `reason` field explains the skip.

---

## Model: `erh:vmodutils:session-capture`

**API:** `rdk:component:sensor`

A "session control" sensor designed to be wired into the data manager's `capture_control_sensor`. When inactive, `Readings` returns just `{"active": false}` and the data manager applies no overrides — capture is effectively off. When active, `Readings` publishes an `overrides` array (one entry per configured component) telling the data manager to capture each at its configured frequency, tagged with the current session id.

Session ids are generated from the local clock at start time as `<tag_prefix><YYYYMMDD_HHMMSS.mmm>`.

### Configuration

```json
{
  "components": [
    { "resource_name": "my-cam", "method": "ReadImage", "capture_frequency_hz": 5 },
    { "resource_name": "my-arm", "method": "EndPosition" }
  ],
  "tag_prefix": "session_"
}
```

| Name         | Type   | Required | Description                                                                                                          |
| ------------ | ------ | -------- | -------------------------------------------------------------------------------------------------------------------- |
| `components` | array  | Yes      | One or more capture targets. Each entry needs `resource_name` and `method`; `capture_frequency_hz` defaults to `10`. |
| `tag_prefix` | string | No       | Prefix prepended to the timestamped session id. Defaults to empty.                                                   |

Each entry in `components`:

| Name                   | Type   | Required | Description                                                               |
| ---------------------- | ------ | -------- | ------------------------------------------------------------------------- |
| `resource_name`        | string | Yes      | Name of the component to capture from.                                    |
| `method`               | string | Yes      | Capture method (e.g. `"ReadImage"`, `"NextPointCloud"`, `"EndPosition"`). |
| `capture_frequency_hz` | float  | No       | Capture frequency override. Defaults to `10`.                             |

### Readings

When inactive:

```json
{ "active": false }
```

When active:

```json
{
  "active": true,
  "overrides": [
    {
      "resource_name": "my-cam",
      "method": "ReadImage",
      "capture_frequency_hz": 5,
      "tags": ["session_20260501_120000.000"]
    }
  ]
}
```

### DoCommand

**`start`** — Begin a new capture session. Stamps a fresh `<tag_prefix><timestamp>` tag and flips `active` to `true`.

```json
{ "start": true }
```

Returns:

```json
{ "status": "capturing", "tags": ["session_20260501_120000.000"] }
```

**`stop`** — End the current session. Flips `active` to `false` and clears the tag list, so subsequent `Readings` calls return only `{"active": false}`.

```json
{ "stop": true }
```

Returns:

```json
{ "status": "stopped" }
```
