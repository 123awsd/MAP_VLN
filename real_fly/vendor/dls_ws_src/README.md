# Vendored real-flight DLS sources

This directory is the source snapshot needed to rebuild the real-flight
SUPER/full-smooth chain without depending on the current NX disk.

Source origin:

- Remote workspace: `/home/nv/dls_ws`
- Repository: `git@github.com:XXLiu-HNU/dls.git`
- Base commit on the NX: `f2e83d7`
- Snapshot date: 2026-09-19

Included source packages:

- `control`
- `drivers`
- `localization`
- `mission_planner`
- `rog_map`
- `super_planner`

The NX working tree also contained these real-flight modifications, which are
included in this snapshot:

- `mission_planner/Apps/ros1_full_smooth_mission.cpp`
- `super_planner/src/super_core/ciri.cpp`
- `super_planner/src/super_core/corridor_generator.cpp`

The CIRI fix keeps the seed endpoints inside the local boundary with a small
numeric margin and records the seed-line diagnostics. It does not change A*,
`robot_r`, observation poses, or MINCO parameters.

Intentionally excluded from this source snapshot:

- build/devel/install products;
- SUPER logs and binary replay logs;
- FAST-LIO `PCD/*.scdb` generated databases;
- bags, maps, flight logs, and runtime mission outputs.

To restore the source into a fresh DLS workspace, copy the contents of this
directory under `DLS_WS/src/`, then install the ROS/system dependencies and
run the normal Release catkin build. The source snapshot is independent of
the ignored runtime data under `real_fly/stage2_runtime/`.
