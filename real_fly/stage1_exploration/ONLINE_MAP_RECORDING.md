# Online dense map recording

Use the same Stage 1 command on NX:

```bash
cd /home/nv/SL_WS/PRE_MAP_VLN_real_fly
RUN_ID=unique_run_name
./real_fly/stage1_exploration/scripts/start_handheld_mapping_rviz.sh --no-rviz --record "$RUN_ID"
```

After Ctrl-C, wait for `Finalizing online dense PCD` to finish. A valid run has
`data/$RUN_ID/online/completion.json` with `status: complete`. Do not power off
while `dense_map.pcd.partial` is present. The recorder does not start flight
control or send motion commands.

`raw/sensors.bag` holds RGB, depth, camera calibration, FAST-LIO odometry, and
EKF odometry. `online/dense_map.pcd` holds all finite points from FAST-LIO's
world-frame registered scans. `online/mapping_outputs.bag` holds FAST-LIO
odometry and a registered scan about every 0.5 seconds for offline free-space
ray tracing. It is not a raw LiDAR recording. `online/ekf_reference.json`,
`fastlio.yaml`, and `lidar_camera.json` preserve the frame parameters used in
semantic projection.

On the host, copy the complete run directory with `rsync` and use the existing
`run_real_scene_semantic.sh --run-id "$RUN_ID" --boxer-mode complete` command.
The pipeline detects the online archive, imports its PCD, and projects RGB-D
with the recorded EKF pose. Existing old-format runs with raw LiDAR Bags still
use the offline FAST-LIO replay path.

The PCD is dense and can be large. Its temporary write requires about twice
the PCD size in free disk space during finalization; the temporary payload is
removed after the final PCD is safely closed. Always check the completion
marker before transferring or processing a run.
