# Stage 2：真机离线数据准备与规划验证

本目录把 `Drone_room` 的真机手持建图结果转换为阶段二可消费的数据。全流程
只读原始 bag，不启动 ROS、MAVROS、PX4、飞控或运动控制节点，也不向无人机
发布任何消息。

## 一键运行

环境和权重已经准备在项目内。以后重启 NX 后只需：

```bash
cd /home/nv/SL_WS/PRE_MAP_VLN_real_fly
./real_fly/stage2_offline/scripts/prepare_drone_room_stage2.sh
```

默认命令会复用现有 Boxer 结果做快速 pipeline smoke。脚本会复用已经完成的
RGB-D 导出、体素快照和 Boxer 结果，再生成场景图、
选择真实可达的识别目标、执行三维 A* 离线规划并做碰撞审计。它不会覆盖
`stage1_exploration/data/Drone_room/raw/sensors.bag`。当前结果是主动停止在
37/49 帧的部分、未融合结果，因此验收会明确标记为 smoke，不冒充完整语义图。
需要完整重跑时才使用：

```bash
./real_fly/stage2_offline/scripts/prepare_drone_room_stage2.sh --complete-boxer
```

当前 NX 的完整 CPU 推理约需 40 分钟；更推荐把导出的约 37 MB
`scene9002_00` 放到配置好 CUDA/PyTorch 的 RTX 5060 主机运行 Boxer，再把 CSV
结果复制回来。

## 当前输入和产物

- 原始 bag：`stage1_exploration/data/Drone_room/raw/sensors.bag`
- FAST-LIO 点云：`stage1_exploration/runtime/live_fast_lio/PCD/handheld_20260905_003121.pcd`
- Boxer/ScanNet 序列：`stage2_offline/data/Drone_room/scene9002_00/`
- 无靶标外参：`scene9002_00/calibration/camera_extrinsic.json`
- 三态体素图：`stage2_offline/data/Drone_room/voxel_snapshot/`
- Boxer 融合检测：`stage2_offline/data/Drone_room/boxer_final_v2/scene9002_00/`
- 场景图和规划：`stage2_offline/data/Drone_room/scene_graph.json`、`planning/`
- 最终验收：`stage2_offline/data/Drone_room/reports/stage2_validation.json`

所有 `data/`、模型权重、第三方源码和虚拟环境都被 Git 忽略；Git 只记录脚本、
配置和说明。外参是无靶标、多帧 RGB-D 到同时刻 LiDAR 扫描的鲁棒拟合结果，
只在相机与雷达刚性安装关系不变时有效。正式飞行前仍应另做静态悬停与低速
安全验证，本离线 PASS 不代表已经授权或验证真实飞行。

## 环境说明

- Python 环境：`.envs/boxer-jp5`，Python 3.8。
- Boxer 固定源码提交：`1f86542dc342a4b1d474c87c97c5d1d6566d9148`。
- Python 3.8 所需的延迟类型注解兼容补丁保存在
  `patches/boxer-python38-future-annotations.patch`，模型文件校验值保存在
  `config/boxer_assets.lock.json`。
- 当前系统只有 Jetson CUDA 驱动接口而没有完整 CUDA Toolkit 动态库，NVIDIA
  PyTorch wheel 无法加载 `libcufft.so.10`；因此使用 aarch64 CPU PyTorch
  2.4.1。没有为此修改 JetPack、内核或 CUDA 系统组件。
- 本次阶段二唯一新增 APT 包是 `python3.8-venv`。安装模拟为 0 升级、1 新装、
  0 删除、0 降级；没有重启网络服务，也没有影响 SSH、Wi-Fi、NoMachine、
  Clash/Mihomo 或 ROS Noetic。

规划配置把 UNKNOWN 当作不可通行，并以 0.35 m 水平半径、0.20 m 垂直半径和
0.15 m 安全余量生成 0.50 m 最小 ESDF 安全距离。这里的输出只能用于离线
阶段二输入与算法验证，不能直接作为真机控制命令。
