# aea_dedalus

Code of DEDALUS, the autonomous driving project of AEA (Politecnico di Milano).

We build the driving stack ourselves: state estimation, perception, planning and control. The work starts in the CARLA simulator and later moves to a small Ackermann robot (Yahboom ROSMASTER R2). We do not start from a ready-made stack, because the goal is to learn how each part works.

This repository holds only the code. The Docker environment (CARLA + ROS 2) is in a separate repository: [dedalus-devenv](https://github.com/AEA-PoliMi/dedalus-devenv).

## Stack

- Ubuntu 24.04
- ROS 2 Jazzy
- CARLA 0.9.16, map Town10HD

## Repository layout

Each sub-team owns one package. Shared packages are used by everyone.

| Package | Owner | Language | Purpose |
|---|---|---|---|
| `dedalus_msgs` | shared | | Custom messages (`Lane`, `StopFlag`) |
| `dedalus_bringup` | shared | | Launch files and shared configuration |
| `dedalus_estimation` | State Estimation | C++ | Vehicle pose and velocity estimate |
| `dedalus_perception` | Perception | Python | Lane and object detection |
| `dedalus_planning` | Control/Planning | Python | Trajectory generation |
| `dedalus_control` | Control/Planning | C++ | Steering and speed command |

Inside a package:

```
dedalus_perception/
├── dedalus_perception/   nodes (C++ packages use src/ and include/)
├── launch/
├── config/               parameters (.yaml)
├── tools/                offline scripts that run on bags
├── test/
└── README.md             what the nodes do and how to run them
```

## Getting started

1. Set up the environment by following the README of `dedalus-devenv`.

2. Create a workspace and clone this repository into its `src/` folder:

```bash
mkdir -p ~/aea_dedalus_ws/src
cd ~/aea_dedalus_ws/src
git clone git@github.com:AEA-PoliMi/aea_dedalus.git
```

3. In `dedalus-devenv/.env`, set `WS_PATH` to the workspace folder (`~/aea_dedalus_ws`). The container sees it as `/ws`.

4. Build from the workspace root (`/ws` inside the container), then source it:

```bash
colcon build --symlink-install
source install/setup.bash
```

`build/`, `install/` and `log/` are created next to `src/`. They are generated on every machine and are not part of the repository.

## How we work

- Work only inside your team's package.
- Never commit directly to `main`. Create a branch named `<team>/<topic>`, for example `perception/lane-detection`.
- When the work builds and runs, open a pull request to `main`.
- A change to `dedalus_msgs`, `dedalus_bringup` or a topic name affects every team. Open a pull request and ask Oguz to review it.
- Keep your package README up to date: which nodes exist, what they subscribe to and publish, how to run them.

## Interfaces

Teams talk to each other only through ROS 2 topics. Do not rename a topic or change a message on your own.

Inputs, published by the CARLA bridge:

| Topic | Type |
|---|---|
| `/sensors/imu` | `sensor_msgs/Imu` |
| `/sensors/wheel_odom` | `nav_msgs/Odometry` |
| `/sensors/lidar` | `sensor_msgs/PointCloud2` |
| `/sensors/camera_<front\|back\|left\|right>/image_raw` | `sensor_msgs/Image` |
| `/sensors/camera_<front\|back\|left\|right>/camera_info` | `sensor_msgs/CameraInfo` |
| `/ground_truth/odom` | `nav_msgs/Odometry` |

`/ground_truth/odom` is the true pose from the simulator. Use it to measure the error of your result, not as an input of your algorithm. It does not exist on the real robot.

Outputs of the teams:

| Topic | Type | Published by |
|---|---|---|
| `/estimation/odom` | `nav_msgs/Odometry` | State Estimation |
| `/perception/lane` | `dedalus_msgs/Lane` | Perception |
| `/perception/objects` | `vision_msgs/Detection3DArray` | Perception |
| `/planning/trajectory` | message not defined yet | Control/Planning |
| `/control/cmd` | `ackermann_msgs/AckermannDriveStamped` | Control/Planning |

## Conventions

- SI units: metres, radians, seconds.
- `base_link` is at the centre of the rear axle: x forward, y left, z up.
- Positive to the left: steering angle, yaw, curvature and lateral error `e_y`.
- Heading error: `e_psi` = vehicle heading − path heading.
- Frames: `map` → `odom` → `base_link`.

## What does not go in git

- Bags and recorded data (`bags/`, `data/`)
- Datasets and trained model weights
- `build/`, `install/`, `log/`

These are large or generated files. Share them outside git and write in your package README where to find them.

## Contact

Oguz Isik
