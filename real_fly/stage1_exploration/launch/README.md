# Sensor-only launch files

These launch files are intentionally not run during repository setup. Invoke
them only under a separate localhost ROS master after the corresponding
packages and topic checks are ready.

- `realsense_d435i.launch` starts only the D435i camera nodelet.
- `livox_mid360.launch` includes only the existing Livox driver2 node and
  requires an explicit run-local JSON; it has no default IP and no internal
  rosbag recorder.
- `mapping_from_bag.launch` is not provided because replay orchestration is
  handled by `scripts/run_mapping_from_bag.sh`, which keeps all output paths
  run-local and never starts a hardware or control node.
