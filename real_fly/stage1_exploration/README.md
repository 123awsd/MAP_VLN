# Stage 1: Falcon exploration

This directory is for real-flight collection notes, launch wrappers, and
experiment-specific configuration.  Do not modify the borrowed Falcon
workspace directly.

Recommended data layout:

```text
data/<run_id>/
├── raw/            # rosbag and original sensor recordings
├── video/          # presentation video
├── map/            # manually built map and map metadata
├── calibration/    # camera calibration and frame transforms
└── metadata/       # time, machine, command, and commit information
```

The video is for presentation.  Keep the rosbag and pose/TF information for
replay and reproducibility.
