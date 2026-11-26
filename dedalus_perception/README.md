# dedalus_perception

Perception package of DEDALUS (Python, ROS 2 Jazzy). It gives the other teams:

- **the lane** → `/perception/lane` (`dedalus_msgs/Lane`: `centre_line`, `e_y`, `e_psi`, `kappa`, `confidence`)
- **the obstacles** → `/perception/objects` (`vision_msgs/Detection3DArray`: 3D box + class)

Team: Jamileth (lead, obstacles / YOLO, U-Net), Alessandro (lane baseline), Angelica (ground removal, Patchwork++),
Giulia (ground removal baseline, clustering).

## Nodes

_None yet._ Planned:

| Node | Subscribes | Publishes |
|---|---|---|
| `lane_node` | `/sensors/camera_front/image_raw`, `/sensors/camera_front/camera_info` | `/perception/lane` |
| `objects_node` | `/sensors/camera_front/image_raw`, `/sensors/lidar` | `/perception/objects` |

`/ground_truth/odom` is used only offline, to measure our errors.

## Current state: offline tools (`tools/`)

The algorithms are developed offline on recorded CARLA data first, then moved into the nodes.

| Folder | What | Who |
|---|---|---|
| `tools/camera/` | Recording scripts for CARLA (RGB, depth, segmentation, 3D LiDAR + ground truth), shared camera model `camera.py` | Jamileth |
| `tools/lane/baseline/` | Classical lane pipeline: BEV → colour filters + Sobel → sliding window → RANSAC → `e_y`, `e_psi`, `kappa` | Alessandro |
| `tools/lane/unet/` | U-Net (Kaggle notebook) replacing the colour-filter step | Jamileth |
| `tools/obstacles/ground_removal/` | Ground removal on the 3D LiDAR: Patchwork++ (main), RANSAC plane (baseline) | Angelica, Giulia |
| `tools/obstacles/clustering/` | Non-ground points → 3D objects | Giulia |
| `tools/obstacles/yolo/` | YOLO detection + distance; later fusion with the LiDAR and tracking | Jamileth |
| `docs/` | Documents and design decisions | all |

## Sensor setup in CARLA (agreed with the coordinator)

| | Value |
|---|---|
| Ego vehicle | `vehicle.tesla.model3`, CARLA 0.9.16, Town10HD |
| Camera (RGB, depth, segmentation) | 960×540, FOV 90°, mounted 1.5 m ahead of the vehicle centre, 1.5 m high, pitch −8° |
| LiDAR | 3D, 32 channels, 50 m, +10° / −30°, roof, 2.0 m high above the vehicle centre |

Conventions follow the repository README: SI units, **`base_link` at the centre of the rear axle**
(x forward, y left, z up), positive to the left for `e_y`, yaw and curvature, `e_psi` = vehicle heading − path heading.
Mounts are given from the vehicle centre (as in CARLA); `tools/camera/camera.py` converts them to `base_link`
using the rear-axle position that the recorder measures and saves in `camera_info.json`.

## Data

Recorded sessions, bags and model weights are **not** in git. They are on Google Drive: _add the link here_.

```
data/<session>/
  camera_info.json          camera + vehicle measurements (rear axle, wheelbase, size)
  info.csv                  per frame: time, speed, pose, true e_y / e_psi at base_link, lane width
  rgb/000000.png
  profondita/000000.npy     depth in metres
  segmentazione/000000.png  CARLA class id (RoadLine = 24)
  lidar/000000.npy          N×4: x, y, z, intensity
```

## Workflow

- Branch names: `perception/<topic>` (e.g. `perception/lane-baseline`). Never commit to `main`.
- Work only inside `dedalus_perception/`. Changes to `dedalus_msgs`, `dedalus_bringup` or topic names: pull request reviewed by Oguz.
- Open a pull request when the work runs, and tell the team.
