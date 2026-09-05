# Stage 1: Falcon handheld mapping on the Jetson

This directory is the real-machine-only entry point for sensor checks,
handheld data capture, offline replay, and map handoff. It never starts a
flight controller and it never publishes motion commands.

## One-command live mapping view

With the aircraft disarmed and propellers removed, open a NoMachine terminal
and run:

```bash
cd /home/nv/SL_WS/PRE_MAP_VLN_real_fly
./real_fly/stage1_exploration/scripts/start_handheld_mapping_rviz.sh
```

The command starts or reuses the local ROS master, starts MAVROS telemetry,
the aircraft MID-360S, D435 RGB-D, the borrowed tuned FAST-LIO profile, and
RViz configured for `world` and `/cloud_registered`. Closing RViz stops only
the child processes created by this invocation. Use `--no-camera` for a
LiDAR-only diagnostic run, or `--no-rviz` when no desktop display is present.

Before FAST-LIO starts, the launcher requests the same 200 Hz PX4
`HIGHRES_IMU` stream used by the borrowed aircraft's `fly.sh`. It does not add
a timestamp-stability waiting period; FAST-LIO still records any detected IMU
timestamp rollback in the run's `fastlio.log`.

For a formal handheld pass, record the raw sensors, RGB-D, FAST-LIO pose and
map output while watching RViz:

```bash
./real_fly/stage1_exploration/scripts/start_handheld_mapping_rviz.sh \
  --record room_01
```

Recording at the same time as mapping preserves the synchronized evidence
needed for offline FAST-LIO replay and Boxer processing. Closing RViz cleanly
stops the bag before the launch processes are stopped. Never defer recording
until after the mapping walk has finished.

## Current machine findings

- The host is the Jetson Orin NX described in `real_fly/machine_profile.md`.
- D435i is visible through `uvcvideo` as `/dev/video0` through `/dev/video5`.
  Short UVC frame reads already succeeded for `/dev/video2` and `/dev/video4`.
- The ROS RealSense packages are not installed yet. The exact arm64 apt
  candidates are `ros-noetic-realsense2-camera` and
  `ros-noetic-realsense2-description`; `v4l-utils` is a diagnostic helper.
  The explicitly authorized no-upgrade install was attempted, but APT stopped
  before unpacking because the existing `clash-verge` package has unresolved
  dependencies (including an unavailable `libwebkit2gtk-4.1-0`). Do not run
  `apt --fix-broken` as part of this Stage-1 setup.
- Existing Livox driver2 and FAST-LIO source/build artifacts are available in
  `/home/nv/dls_ws`, but they are treated as read-only underlay. The existing
  FAST-LIO configuration uses `/mavros/imu/data` and is not used here.
- LAN1 is currently `eth0` with no carrier because the MID-360 is not powered.
  Do not configure or probe it until the sensor and cable are ready.

## Isolation rules

Use a separate ROS master, normally a free localhost port such as `11312`,
for sensor-only and offline work. The capture script rejects the borrowed
master on port `11311`, because that master currently contains unrelated GCS
and flight registrations. The environment helper only changes the child
shell that sources it; it does not edit shell startup files or ROS system
configuration.

FAST-LIO is built as `stage1_fast_lio` in `ros_ws/`. Its compile-time output
root is a new per-run directory below `data/<run_id>/mapping/`, so its
log/PCD writes do not go to the borrowed `/home/nv/dls_ws` source tree or
overwrite an earlier run.

The mapping IMU is the MID-360's own `/livox/imu` stream. The D435i is captured
for RGB/depth coverage and later colorization; its IMU is not silently mixed
into FAST-LIO. The LiDAR-to-IMU rotation/translation in the handheld config
starts as identity/zero and must be measured or validated before a map is
accepted for Stage 2.

## Safe sequence

1. Install the missing RealSense packages only if the operator approves the
   exact apt command shown below. Do not run `apt upgrade`.
2. Start a sensor-only ROS master and the D435i launch. After the MID-360 is
   powered and LAN1 has carrier, identify its actual IP and host IP, create a
   run-local JSON with `scripts/make_mid360_config.sh`, then start the Livox
   sensor-only launch.
3. Run `scripts/check_ros_topics.sh` against a run-local `topics.env`. Confirm
   message types, TF, and short rate checks before recording.
   `scripts/check_rgb_coverage.py` later measures RGB/depth temporal overlap;
   it deliberately does not claim geometric point coloring without calibration.
4. When the operator explicitly says to start handheld acquisition, invoke
   `scripts/record_handheld_bag.sh --confirm-handheld ...`. It records only
   the listed sensor/TF topics and never uses `rosbag record -a`.
5. Stop the bag while the device is stationary. Run `scripts/check_bag.py`.
6. Replay the bag with `scripts/run_mapping_from_bag.sh`. This starts only a
   local roscore, FAST-LIO, rosbag record/play, and no hardware/control node.
7. Run `scripts/export_stage1_report.py` to produce
   `outputs/<run_id>/` with the trajectory CSV, start pose, frame description,
   PCD/header manifest, calibration status, target-annotation template,
   temporal RGB coverage report, and Stage 2 handoff metadata.

`scripts/monitor_resources.sh` can be run in a second shell during the static
test, capture, or replay. It records only resource observations and never
changes Jetson performance settings.

## Exact package command and current blocker

```bash
sudo apt-get install --no-install-recommends \
  ros-noetic-realsense2-camera \
  ros-noetic-realsense2-description \
  v4l-utils
```

This exact command was attempted after operator authorization. It did not
install anything because APT refused to resolve the pre-existing broken
`clash-verge` dependency set before unpacking. It does not modify the network,
ROS master, flight controller, or shell startup files; do not replace it with
`apt --fix-broken` without a separate system-environment review.

## Important limitations

The RGB check initially proves temporal availability and CameraInfo/depth
availability. It is not a geometric “every map point has a valid RGB pixel”
claim until the camera-to-LiDAR extrinsic and synchronized projection are
measured. The generated report labels this distinction explicitly.
