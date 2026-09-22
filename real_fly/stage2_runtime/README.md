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

The monitor is configured for the aircraft's 4S pack and uses the existing
3.5 V/cell lower bound (14.0 V total). For different hardware, pass the cell
count explicitly, for example `monitor_single_uav.sh --battery-cells 6`; it is
intentionally never inferred from pack voltage.

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

Intermediate `transit` goals are pass-through route constraints: the adapter
switches to the next goal inside a 0.40 m radius without requiring low speed or
a dwell, allowing SUPER to preserve trajectory continuity. Only final
`observation` goals require the configured 0.20 m arrival tolerance, low speed,
and 4.0 s dwell. The pass-through radius can be changed explicitly with
`--transit-switch-radius`, but must remain in `[0.10, 1.0]` m.

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

Start the planner (still without sending a goal) with the exact approved map
and execution bundle:

```bash
./real_fly/stage2_runtime/scripts/start_super_indoor_planner.sh \
  /absolute/path/to/handheld_map_RUN_ID_complete.pcd \
  --bundle real_fly/stage2_runtime/missions/RUN_ID/TASK_ID/execution_bundle.json
```

The launcher keeps that original PCD unchanged for localization and identity
checks. For ROG's occupied-only prior it creates and reuses an XYZ-only 0.15 m
voxel cache under `~/.cache/pre_map_vln/super_collision/`. Every live LiDAR
point is retained, matching the senior real-flight profile, while headless
planner/map visualization is disabled. Collision inflation, unknown-space
handling, map resolution, and flight limits are not relaxed. Set
`PRE_MAP_VLN_SUPER_STATIC_VOXEL` only for an explicit benchmark, not on a
per-map basis.

The task-specific collision cache removes only static voxels inside a 0.60 m
sphere around the explicitly approved launch hover pose, with a hard maximum
of 100 removed voxels. Execution also requires the aircraft to start within
0.15 m of that approved pose. Together these bounds retain at least 0.45 m of
static clearance at the actual start. The original localization PCD and its
SHA256 remain unchanged. Before the first goal is released, the adapter
verifies the cache identity and requires the current registered LiDAR cloud to
show at least 0.40 m clearance around the aircraft. A dense static structure
or a live obstacle therefore causes a refusal instead of being silently carved
away.

Real execution is deliberately not wrapped in a one-command launcher. It
requires the exact approved map SHA256, live `world`-frame EKF odometry, a
connected and armed FCU, a SUPER subscriber, start-pose agreement, low initial
speed, per-goal arrival checks and bounded timeouts. It never arms, takes off,
lands, or publishes `PositionCommand`. Validate preview and a restrained test
with the aircraft secured before enabling free flight.

## Manual-flight exploration recording

`scripts/record_manual_exploration_bag.sh` is an observation-only recorder for
a manually flown mapping pass. Start it in a separate terminal after
`/ekf_quat/ekf_odom` and `/cloud_registered` are healthy, but before arming:

```bash
./real_fly/stage2_runtime/scripts/record_manual_exploration_bag.sh \
  dorm_manual_explore_01
```

It refuses to start unless MAVROS reports connected and disarmed. The bag
contains a 2 Hz registered point cloud and 5 Hz JPEG-compressed RGB stream,
plus odometry, TF, FCU, controller and diagnostics topics. A full-rate H.264
RGB MP4 is stored beside the bag. These defaults deliberately avoid recording
raw 30 Hz RGB or full-rate clouds during flight. The script starts no mapping,
PX4Ctrl, takeoff, mode-change, goal, or setpoint publisher. Stop it with
Ctrl-C only after landing and disarming so the split bag and MP4 are sealed.

Replay one completed manual-exploration session as a progressively growing
5 cm voxel map, actual trajectory and RGB first-person panel:

```bash
./real_fly/stage2_runtime/scripts/replay_manual_exploration_rviz.sh \
  real_fly/stage2_runtime/runtime/manual_exploration_bags/<run>/<session> \
  --rate 0.5 --max-z 2.0
```

The replay runs in a network-isolated container. Points above world Z=2 m are
discarded before accumulation. It starts no sensor, MAVROS, PX4Ctrl, planner,
setpoint or flight node. It also auto-detects the earlier Stage-1 profile with
`/cloud_registered`, raw `/camera/color/image_raw`, and fused odometry, so the
existing `dorm_room_v3/raw/sensors.bag` can be demonstrated immediately.
Older Stage-3 flight bags that omitted point clouds and kept RGB only as an
MP4 sidecar cannot provide this synchronized progressive view.

Add `--record-video outputs/dorm_exploration_demo.mp4` to capture only the
RViz window as an H.264 presentation video. The capture includes the growing
map, trajectory, current pose and embedded RGB panel. The launcher delays bag
playback until the RViz window and screen recorder are ready and refuses to
overwrite an existing MP4.

## Direct full-smooth execution

For static tasks, `run_real_stage2_task.sh` generates final MINCO on the host.
The saved coefficients, timing and yaw in `final_minco.txt` are the authority
for both the host preview and NX execution. NX verifies the hashed artifact
set and loads it without replanning. See [SAVED_MINCO.md](SAVED_MINCO.md).
Automatic landing remains disabled; live clearance monitoring is shadow-only.

After copying the task directory to the NX, start the direct executor with:

```bash
./real_fly/stage2_runtime/scripts/start_full_smooth_mission.sh \
  "/home/nv/dls_ws/${RUN_ID}.pcd" \
  --bundle \
  "real_fly/stage2_runtime/missions/${RUN_ID}/${TASK_ID}/execution_bundle.json"
```

Start PX4Ctrl separately. After takeoff and stable hover, switching PX4Ctrl to
command-control mode emits `/traj_start_trigger`; `full_smooth_mission` then
checks the current odometry and starts the continuous route. Landing remains a
manual pilot action.

## NX planning from the localized takeoff point

After host-side room and semantic approval, export the compact per-run planning
package and copy it to the NX. The exporter verifies the approved PCD SHA256 and
refuses to overwrite an existing NX package:

```bash
./real_fly/stage2_offline/scripts/export_real_planning_package.sh \
  --run-id "$RUN_ID" --sync-nx
```

With persistent global localization running, and the FCU connected and
disarmed, parse the natural-language instruction with Qwen and plan directly on
the NX:

```bash
./real_fly/stage2_runtime/scripts/plan_from_current_pose.sh \
  "$RUN_ID" "$TASK_ID" --instruction '先观察灭火器，再观察电视柜。'
```

The API key is intentionally not included in the map package. It defaults to
`~/.config/pre_map_vln/dashscope_api_key` on the NX and must be provisioned
separately with mode `600`. Natural-language Qwen parsing is the only task-input
path; there is no fixed-target shortcut.

The start capture is subscriber-only. It samples stable
`/ekf_quat/ekf_odom`, reads the senior PX4Ctrl profile's relative
`takeoff_height`, and plans from `(ground x, ground y, ground z + height,
ground yaw)`. It refuses to run while the FCU is armed. Qwen only produces the
task graph; the geometry modules still choose observation poses and routes. The
command publishes no ROS topic, service call, setpoint, or flight command.

On an NX desktop terminal, inspect the result and then close RViz:

```bash
./real_fly/stage2_runtime/scripts/view_runtime_plan_rviz.sh \
  "$RUN_ID" "$TASK_ID"
```

The preview starts only a static PCD publisher and visualization markers. It
does not start SUPER or PX4Ctrl and does not create a goal publisher. Close it
before flight to release NX graphics and memory resources. The same local
`execution_bundle.json` can then be used by the guarded adapter; no round trip
to the host is required.
