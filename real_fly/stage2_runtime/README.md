# Stage 2 real-machine runtime

`stage2_runtime` contains the lightweight online localization entry point for
real flight. It is separate from Stage 1 mapping and from the GPU-heavy Stage
2 offline Boxer pipeline.

## Localization-only mode

This launcher is a lightweight **local-origin diagnostic**. It is useful for
latency tests, but it does not perform global relocalization against an approved
map and does not publish `/ekf_quat/ekf_odom`; therefore it must not be used as
the localization source for a Stage-2 mission or PX4Ctrl.

With the flight controller powered, aircraft disarmed, and MID-360 connected:

```bash
cd /home/nv/SL_WS/PRE_MAP_VLN_real_fly
./real_fly/stage2_runtime/scripts/start_localization_only.sh
```

The launcher starts or reuses the local ROS master, starts MAVROS telemetry,
the MID-360S, lightweight FAST-LIO, and a read-only terminal health display.
It never starts D435, RViz, PX4Ctrl, a planner/controller, or any setpoint and
never arms the aircraft.

The online interface is:

- `/Odometry`: FAST-LIO state at the LiDAR rate (target: about 20 Hz).
- TF `world -> body`: the corresponding pose transform.
- `/livox/lidar` and `/mavros/imu/data`: localization sensor inputs.

FAST-LIO still maintains an internal incremental point map because scan-to-map
matching requires it. The lightweight profile disables point-cloud, global-map,
path, planner-facing LiDAR-bin and PCD publication/saving, which removes the
heavy Stage 1 visualization work while preserving odometry.

The monitor marks odometry delay below 100 ms as healthy and warns when the
rolling maximum exceeds 150 ms. Before using this output downstream, perform
a stationary/handheld test lasting at least five minutes and confirm that the
delay remains bounded instead of increasing over time.

Because the monitor only subscribes, it can also be run separately against an
already-running localization stack:

```bash
bash -c 'source real_fly/stage1_exploration/scripts/env.sh; \
  export ROS_MASTER_URI=http://127.0.0.1:11311; \
  exec real_fly/stage2_runtime/scripts/monitor_localization.py'
```

Current limitation: the files can be checked without attached sensors, but
frequency and end-to-end latency must be validated after the MID-360 is
connected. A real-flight planner/controller must not be enabled merely because
the launcher starts successfully.

## Persistent global localization for Stage 2

Before PX4Ctrl or SUPER, start the senior's complete global-localization chain
in its own terminal with the exact approved map:

```bash
./real_fly/stage2_runtime/scripts/start_global_localization.sh \
  /absolute/path/to/handheld_map_RUN_ID_complete.pcd
```

This runs MAVROS, MID-360, global FAST-LIO relocalization and the downstream EKF.
It must remain alive from before entering PX4Ctrl hover mode until the aircraft
has landed and is disarmed. Its `/ekf_quat/ekf_odom` is consumed continuously by
both PX4Ctrl and SUPER. If it becomes stale, PX4Ctrl's state machine leaves
AUTO_HOVER/CMD_CTRL and restores the pre-Offboard flight mode.

The senior stack does not currently feed this odometry into PX4's own EKF via
`/mavros/vision_pose/pose` or `/mavros/odometry/in`. Consequently, native PX4
position modes must not be assumed to have LiDAR localization. Position hold in
this workflow is PX4Ctrl `AUTO_HOVER` operating in PX4 `OFFBOARD` mode.

Run the read-only single-aircraft terminal monitor in another terminal:

```bash
./real_fly/stage2_runtime/scripts/monitor_single_uav.sh
```

It combines the senior GCS health semantics into one local display: FCU link,
arming and PX4 mode; battery; RC channels 5/6; EKF pose, frequency, delay and
rolling position jump; IMU; PX4Ctrl FSM state; planner command flow and attitude
setpoint flow. It publishes no topic and calls no ROS service. `PASS` is only a
software readiness result and does not replace the QGC pre-arm report or a
physical aircraft inspection.

## Guarded Stage-2 to SUPER adapter

The task layer does not publish `PositionCommand`. It converts an offline
validated `mission_plan.json` into an immutable execution bundle and sends only
individual 3-D `PoseStamped` goals to the senior SUPER planner. SUPER remains
responsible for the live occupancy map, trajectory continuity, dynamics and
`/planning/pos_cmd`; PX4Ctrl remains the only aircraft controller.

SUPER receives the approved PCD as **occupied-only** prior constraints and the
registered live LiDAR cloud for ray-integrated free space and current obstacles.
The prior map is never used to manufacture free space, and unknown space remains
blocked by the indoor profile.

Prepare a bundle on the workstation:

```bash
./real_fly/stage2_runtime/scripts/prepare_real_execution.sh \
  RUN_ID /absolute/path/to/mission_plan.json
```

The runtime adapter defaults to `preview`. In preview mode it publishes only
`/pre_map_vln/approved_route`, `/pre_map_vln/approved_goals`, and status; the
process does not even create a `/planning/click_goal` publisher.

Start the planner (still without sending a goal) with the exact approved map:

```bash
./real_fly/stage2_runtime/scripts/start_super_indoor_planner.sh \
  /absolute/path/to/handheld_map_RUN_ID_complete.pcd
```

Real execution is deliberately not wrapped in a one-command launcher. It
requires the exact approved map SHA256, live `world`-frame EKF odometry, a
connected and armed FCU, a SUPER subscriber, start-pose agreement, low initial
speed, per-goal arrival checks and bounded timeouts. It never arms, takes off,
lands, or publishes `PositionCommand`. Validate preview and a restrained test
with the aircraft secured before enabling free flight.
