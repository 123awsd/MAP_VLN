# Real-flight experiment

This directory contains the real-flight experiment entry points and notes for
the `real-fly` branch.  The existing `stage1/` and `stage2/` Python packages
remain shared algorithm code; real-flight-only scripts and configurations
belong under the two directories below.

```text
real_fly/
├── machine_profile.md
├── stage1_exploration/
│   └── data/                 # rosbag, video, map, calibration; not in Git
└── stage2_offline/
    └── data/                 # inputs, execution logs, reports; not in Git
```

## 真机验证状态

已在线下完成真机验证：跨房间飞行、在线目标识别、条件分支执行、局部视点重试和跨房间恢复搜索。
离线地图处理、静态轨迹回放和定位诊断各有独立入口；其局部功能范围不代表完整系统的实验范围。

## Data policy

Keep large or generated files under the corresponding `data/` directory, for
example `stage1_exploration/data/20260903_v1/`.  These files are intentionally
ignored by Git.  Track only small metadata, configuration, and result summaries
needed to reproduce an experiment.

Do not put the Falcon workspace, ROS installation, passwords, SSH private keys,
or subscription URLs in this repository.

## Environment policy

The borrowed Falcon environment is treated as the underlay and should not be
modified directly.  Use a separate overlay workspace and keep the offline
Stage 2 runner independent from the live flight-control topics.
