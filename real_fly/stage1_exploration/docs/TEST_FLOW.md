# Stage-1 reproducible test flow

The commands below are operator-run checkpoints. They are deliberately not
executed during repository setup.

## A. No hardware/control launch

```bash
cd /home/nv/SL_WS/PRE_MAP_VLN_real_fly
source real_fly/stage1_exploration/scripts/env.sh
export ROS_MASTER_URI=http://127.0.0.1:11312
roscore -p 11312
```

In a second shell, launch only the D435i and, after LAN1 has carrier, the
Livox file with an explicit run-local JSON. Do not source or launch the
borrowed Falcon flight launch files.

The prepared sensor-only wrapper can be used after the isolated master is
running. Select each sensor explicitly; it refuses the borrowed master port
and never starts roscore or a flight node:

```bash
real_fly/stage1_exploration/scripts/start_sensor_only.sh \
  --run-id room_YYYYMMDD_HHMMSS \
  --master-port 11312 \
  --start-d435i \
  --start-mid360 \
  --mid360-config /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/data/<run_id>/MID360_config.json
```

## B. Static sensor checks

```bash
real_fly/stage1_exploration/scripts/check_realsense_uvc.sh
real_fly/stage1_exploration/scripts/check_mid360_lan.sh --interface eth0
real_fly/stage1_exploration/scripts/check_ros_topics.sh \
  --config real_fly/stage1_exploration/data/<run_id>/topics.env \
  --require-rgb
```

The LAN check returns a nonzero status while the radar has no carrier. That
is an expected stop condition, not a reason to assign a guessed address.

## C. Handheld capture checkpoint

First verify the device is physically safe to carry and that only sensor
topics are in the manifest. Then the operator explicitly starts:

```bash
real_fly/stage1_exploration/scripts/record_handheld_bag.sh \
  --confirm-handheld \
  --run-id room_YYYYMMDD_HHMMSS \
  --topics-config real_fly/stage1_exploration/data/<run_id>/topics.env
```

Walk slowly, keep the sensor orientation consistent, revisit the start, and
stop while stationary. Keep the first pass short (about 30–60 seconds) before
attempting a larger room.

## D. Offline map and handoff

```bash
real_fly/stage1_exploration/scripts/check_bag.py \
  real_fly/stage1_exploration/data/<run_id>/raw/sensors.bag \
  --topics-config real_fly/stage1_exploration/data/<run_id>/topics.env \
  --require-rgb \
  --output real_fly/stage1_exploration/data/<run_id>/raw/bag_report.json

real_fly/stage1_exploration/scripts/check_rgb_coverage.py \
  real_fly/stage1_exploration/data/<run_id>/raw/sensors.bag \
  --topics-config real_fly/stage1_exploration/data/<run_id>/topics.env \
  --output real_fly/stage1_exploration/data/<run_id>/raw/rgb_coverage.json

real_fly/stage1_exploration/scripts/build_stage1_fast_lio.sh \
  --runtime-root /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/data/<run_id>/mapping/fast_lio_runtime
real_fly/stage1_exploration/scripts/run_mapping_from_bag.sh \
  --bag real_fly/stage1_exploration/data/<run_id>/raw/sensors.bag \
  --run-id <run_id> \
  --runtime-root /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/data/<run_id>/mapping/fast_lio_runtime

real_fly/stage1_exploration/scripts/export_stage1_report.py \
  --run-dir real_fly/stage1_exploration/data/<run_id> \
  --raw-bag real_fly/stage1_exploration/data/<run_id>/raw/sensors.bag \
  --mapping-bag real_fly/stage1_exploration/data/<run_id>/mapping/mapping_outputs.bag \
  --pcd /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/data/<run_id>/mapping/fast_lio_runtime/PCD/handheld_map_<run_id>.pcd \
  --topics-config real_fly/stage1_exploration/data/<run_id>/topics.env \
  --raw-report real_fly/stage1_exploration/data/<run_id>/raw/bag_report.json \
  --output /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/outputs/<run_id>/report.json
```

The final report is a Stage-1 artifact. It does not authorize Stage-2 live
flight; conversion to occupancy/ESDF and any execution adapter require a
separate review.

During capture or replay, resource sampling is also available. It only reads
procfs, thermal files, disk statistics, and optional `tegrastats`; it does not
change Jetson performance settings:

```bash
real_fly/stage1_exploration/scripts/monitor_resources.sh \
  --output /home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage1_exploration/data/<run_id>/raw/resources.tsv \
  --duration-sec 120 --interval-sec 5
```
