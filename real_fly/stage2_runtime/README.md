# Stage 2 real-machine runtime

`stage2_runtime` contains the lightweight online localization entry point for
real flight. It is separate from Stage 1 mapping and from the GPU-heavy Stage
2 offline Boxer pipeline.

## Localization-only mode

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
