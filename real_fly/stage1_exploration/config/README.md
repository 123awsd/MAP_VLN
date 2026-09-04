# Stage 1 configuration

Copy a template to a run-local file below `data/<run_id>/` and fill it only
after `rostopic list`/`rostopic type` has confirmed the actual driver output.
Templates contain no simulation topics and no control topics.

- `topics.env.example`: topic names and expected ROS message types.
  It supports either the D435i fused IMU topic or the driver's separate gyro
  and accel topics. A D435 without a motion module may leave those fields empty
  and set `CAMERA_IMU_REQUIRED=0`; FAST-LIO uses the Livox IMU separately.
- `MID360_config.example.json` / `MID360s_config.example.json`: model-specific
  driver JSON shapes; use
  `scripts/make_mid360_config.sh` to create a run-local JSON after the actual
  sensor and host IPs are known. Pass `--model mid360s` when the device reports
  Livox SDK device type 35.
- `fast_lio_mid360_handheld.yaml`: handheld mapping parameters. Its identity
  extrinsic is a provisional bring-up value, not a final calibration.
