# Independent Stage-1 ROS overlay

The overlay is intentionally inside `real_fly/stage1_exploration/ros_ws`.
It uses the system ROS Noetic underlay and read-only source/build artifacts
from `/home/nv/dls_ws` only for Livox message definitions and FAST-LIO source.
Build products and runtime output stay in this directory or its sibling
`runtime/` directory.

`src/stage1_fast_lio` is a small local package that recompiles the existing
FAST-LIO mapping sources with a Stage-1 `ROOT_DIR`. This prevents the original
binary from truncating files in `/home/nv/dls_ws/src/localization/FAST_LIO/Log`
or writing a PCD into its source tree.
