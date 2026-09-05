# MID-360S to D435 extrinsic calibration

This directory prepares the official HKU-MARS FAST-Calib workflow for the
aircraft's Livox driver2 MID-360S and Intel RealSense D435. It calibrates
`T_cam_lidar`; it does not change the borrowed LiDAR-to-FCU-IMU calibration.

The pinned upstream revision is `1018ecfdf9deda51b91a8a11bd11972a0b159008`.
The project patch changes only the ROS message dependency from the old
`livox_ros_driver` custom message to the installed `livox_ros_driver2` type.

## Fixed calibration inputs

- D435 serial observed on this aircraft: `112222070870`.
- Color profile: 640x480.
- Intrinsics from the recorded `/camera/color/camera_info`:
  `fx=605.2134399`, `fy=605.4168091`, `cx=330.0566101`,
  `cy=245.5398865`, zero reported distortion.
- Official 1400x1000 mm target: 200 mm ArUco markers with IDs 1, 2, 3, 4;
  four 240 mm diameter holes with 500x400 mm center spacing.

Do not continue if the physical target dimensions differ or the camera mount
moves after capture.

## Build (already completed on this NX)

```bash
cd /home/nv/SL_WS/PRE_MAP_VLN_real_fly
./real_fly/calibration/scripts/setup_fast_calib.sh
```

The setup is project-local and does not install system packages.

## Capture three static scenes

Requirements: propellers removed, aircraft disarmed, MID-360 Ethernet and D435
USB connected, and the sensor-to-camera mount rigid. The flight controller is
not needed. Put the complete target about 2.5 m from the LiDAR and avoid a wall
or another large plane at nearly the same range.

Use one shared set ID for all three captures. Keep the board upright. Capture
one view with the rig centered, then move the whole sensor rig laterally for a
right-side and a left-side view while aiming it at the board. Both sensor rig
and board must be motionless during each 15-second recording.

```bash
cd /home/nv/SL_WS/PRE_MAP_VLN_real_fly

./real_fly/calibration/scripts/capture_fast_calib_scene.sh \
  --set-id calib_01 --scene front --confirm-static-calibration

./real_fly/calibration/scripts/capture_fast_calib_scene.sh \
  --set-id calib_01 --scene right --confirm-static-calibration

./real_fly/calibration/scripts/capture_fast_calib_scene.sh \
  --set-id calib_01 --scene left --confirm-static-calibration
```

Each invocation starts only a private ROS master, D435 color stream and
MID-360S, verifies all four ArUco IDs, then records only `/livox/lidar`. It does
not start MAVROS, FAST-LIO, a planner, controller or motion publisher.

## Compute single-scene and joint results

The default LiDAR crop assumes the target center is approximately 2.5 m in
front of the sensor. Run each scene separately:

```bash
./real_fly/calibration/scripts/run_single_calib.sh --set-id calib_01 --scene front
./real_fly/calibration/scripts/run_single_calib.sh --set-id calib_01 --scene right
./real_fly/calibration/scripts/run_single_calib.sh --set-id calib_01 --scene left
./real_fly/calibration/scripts/run_multi_calib.sh --set-id calib_01
```

If a single scene cannot isolate the board, preserve its log and rerun under a
new set ID or move the failed result directory aside, supplying tighter crop
arguments such as `--x-min 2.0 --x-max 3.0 --y-min -1.0 --y-max 1.0`.

The final matrix is written to:

```text
real_fly/calibration/data/calib_01/results/multi/multi_calib_result.txt
```

It is `T_cam_lidar` and must be visually checked with the generated colored
point cloud before conversion or insertion into the Stage 2 projection config.
Data, images, bags, logs, upstream source and build products are Git-ignored.
