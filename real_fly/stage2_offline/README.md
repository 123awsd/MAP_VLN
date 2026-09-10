# Stage 2：真机离线数据准备与规划验证

本目录把 `Drone_room` 的真机手持建图结果转换为阶段二可消费的数据。全流程
只读原始 bag，不启动 ROS、MAVROS、PX4、飞控或运动控制节点，也不向无人机
发布任何消息。

## 一键运行

### 每张真机地图的房间分割与人工确认

完成 `run_real_scene_semantic.sh` 并用 RViz 选定适合当前地图的 Box 最小出现次数后，
只需再运行一条命令。例如本图选择 3 次：

```bash
./real_fly/stage2_offline/scripts/review_and_approve_real_scene.sh \
  --run-id "$RUN_ID" \
  --min-observations 3 \
  --room-max-z 2.0 \
  --wall-min-count 100 \
  --min-room-area 5.0
```

脚本直接从该 RUN 的最终 FAST-LIO PCD 提取墙体几何，再用 OccuSG 给出房间边界草案，
最后打开人工编辑器。D435 RGB-D 只用于物体语义，不参与房间几何。单击选择房间，Shift+单击
多选；可新建、修改、删除或合并多边形，填写房间名、允许的观察飞行 z 范围和相邻房间。若要拆分，
删除原区域后绘制两个新区域即可；`Auto Adj` 可按门口距离重建相邻关系。按
`Save Draft` 后会检查房间重叠、邻接连通、碰撞安全体素以及从
`planning_start.json` 的可达性。

校验通过后，先查看 `room_review_pcd/validation_preview.png`，再在终端输入完全一致的
`APPROVE`。正式产物是每个 RUN 独立的 `approved_scene_graph.json`；未确认时只保留
草稿，不会覆盖原 `scene_graph.json`。该流程不调用 Qwen 判断房间语义，Box 次数也
不会写成全局固定参数。后续 `run_real_stage2_task.sh` 会自动优先使用已确认场景图。
`--room-max-z` 和 `--wall-min-count` 同样是每张地图独立的审核参数；后者也可设为
`auto`，使用当前 PCD 非空 XY 网格点数的第 95 百分位。

只生成 OccuSG 草案而暂不打开编辑器时，可追加 `--prepare-only`。房间语义确认仍是
离线地图审核，不代表授权解锁、起飞或执行轨迹。

### 真机阶段二：自然语言任务与安全预览

完成语义地图并人工验收后，在主机离线生成任务图、观察位姿、三维路径、碰撞检查和
只读执行包：

```bash
./real_fly/stage2_offline/scripts/run_real_stage2_task.sh \
  --run-id RUN_ID \
  --task-id TASK_ID \
  --instruction '检查房间里的白板是否还在。'
```

若已确定真机起飞后稳定悬停点在地图 `world` 坐标中的位姿，应显式传入：

```bash
./real_fly/stage2_offline/scripts/run_real_stage2_task.sh \
  --run-id RUN_ID --task-id TASK_ID \
  --instruction '检查房间里的白板是否还在。' \
  --start X Y Z YAW
```

RViz 只读预览：

```bash
./real_fly/stage2_offline/scripts/view_real_scene_rviz.sh RUN_ID 1 TASK_ID
```

预览会从当前 RUN 的 `voxel_snapshot` 同时显示两种规划空间：绿色半透明体素是经过
安全膨胀后仍可通行的已观测空间，橙红色体素是原本为空闲、但因净空不足而被安全
膨胀排除的空间。未知空间仍按不可通行处理但默认不绘制；两层均可在 RViz 左侧单独
开关。它们只用于离线解释规划结果，不会发布飞控命令。

未传 `--start` 时会复用自动选择的 `planning_start.json`，仅可预览，运行时适配器
拒绝正式发送目标。当前执行包也只覆盖运动测试；真机在线目标检测、条件分支和
Qwen 主动恢复尚未接通时，不会被标记为完整自主语义执行。

### 任意真机 Bag 的通用语义地图入口

新 Bag 放在 `stage1_exploration/data/<run_id>/raw/sensors.bag` 后，使用通用入口：

```bash
./real_fly/stage2_offline/scripts/run_real_scene_semantic.sh \
  --run-id Drone_room_03_20260906
```

该脚本为每个 run 单独创建 `stage2_offline/data/<run_id>/`，默认完成
原生 FAST-LIO 离线重放并保存完整 `scans.pcd`、原始深度离线对齐、体素快照、完整 Boxer 3D 融合和
`scene_graph.json` 生成，不覆盖其他场景结果。若只想快速 smoke：

```bash
./real_fly/stage2_offline/scripts/run_real_scene_semantic.sh \
  --run-id Drone_room_03_20260906 --boxer-mode smoke
```

如需继续生成离线任务规划，在命令末尾增加 `--with-planning`。脚本默认使用
`real_fly/calibration/config/calib_01_lidar_camera.json` 和
`fast_lio_mid360_handheld.yaml`；若相机与 LiDAR 安装关系发生变化，必须先更换
对应标定文件，不能直接复用 `calib_01`。

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
- FAST-LIO 最终地图：`stage2_offline/data/<run_id>/fastlio_complete/handheld_map_<run_id>_complete.pcd`
- FAST-LIO 原生地图来源：离线运行时由 FAST-LIO 原生 `pcd_save` 生成的 `PCD/scans.pcd`；同目录下的
  `handheld_map_<run_id>_complete_kdtree_local.pcd` 仅作局部 iKD-Tree 诊断，不作为最终地图。
- FAST-LIO 重放输出：`stage2_offline/data/<run_id>/fastlio_complete/mapping_outputs.bag`
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
