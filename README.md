# PRE_MAP_VLN

Habitat 中的无人机预探索与全局 3D 语义地图构建工程。

第一阶段目标：使用 FALCON 完成自主探索，使用 Boxer 构建全局 3D 物体框，使用 OccuSG 划分房间区域，最终输出供后续 VLN 使用的统一语义地图与场景图。

项目状态和对话迁移入口见 [`记忆/README.md`](记忆/README.md)。

## 当前状态

第一阶段工程闭环已在公开 HM3D example `00861-GLAQ4DNUx5U` 上通过：Habitat 与 FALCON 闭环探索、Boxer 全局 3D 框、OccuSG 房间分割以及语义场景图均有实测输出。详细结果见 [`docs/第一阶段设计.md`](docs/第一阶段设计.md)。

已有探索 episode 的后处理命令：

```bash
cd /home/uav/map_VLN/PRE_MAP_VLN
./scripts/run_stage1_postprocess.sh hm3d_stage1
.envs/habitat/bin/python scripts/validate_stage1.py
```

大型数据、环境、权重、第三方仓库和实验输出均在本目录内但不进入 Git。OccuSG 构建必须使用 `./scripts/build_occusg.sh`，它会检查可用内存并将 C++ 编译限制为最多两个任务。

## RViz 与 rosbag

默认同步包：`outputs/bags/hm3d_stage1_complete_v3_final.bag`。它不是定时截断，而是在 FALCON 明确进入 `FINISH` 后自动停止：248.303 秒、2,479 帧 Habitat RGB、43,379 条消息，末帧 active Frontier 为 0、occupied map 为 234,056 点。包内包含 RGB-D、FALCON 轨迹/frontier、地图点云、实际相机姿态，以及 179 个 Boxer 3D boxes 的 118 次渐进事件。

```bash
cd /home/uav/map_VLN/PRE_MAP_VLN
./scripts/replay_bag_rviz.sh
```

脚本会自动复制当前桌面的 X11 授权到项目内的忽略文件，不修改系统级 X11 配置。

回放会同时打开一个参考 FUEL `office3.gif` 的“掀顶式”3D/RGB 窗口和一个俯视 2D 边界/frontier 窗口。白底地图使用按 Z 高度在青—蓝—洋红之间变化的细点配色，而不是大块实心 voxel 或完整彩虹；地板高度由无人机高度在线推断，天花板按每个 XY 栅格柱的局部最高表面剥离，因此斜顶也能随坡度被去掉。深蓝细线为 Habitat 实际已执行轨迹，琥珀色为 FALCON 当前 B-spline，Frontier 保留 FALCON 的簇颜色但仅用 1 px 半透明点显示；红色细视锥直接跟随实际 `camera_optical` 姿态，与实时 RGB 朝向一致。

循环播放：

```bash
./scripts/replay_bag_rviz.sh hm3d_stage1_complete_v3_final true
```

bag 播放完成后 RViz 会保持打开。第三个参数可调整回放倍速，第四个参数控制 2D 窗口，例如 `./scripts/replay_bag_rviz.sh hm3d_stage1_complete_v3_final false 2.0 false`。

第五至七个参数依次调整离地净空、开始识别顶棚的最小室内高度和沿局部顶面的剥离厚度。例如使用默认的 0.25/1.65/0.60 m：

```bash
./scripts/replay_bag_rviz.sh hm3d_stage1_complete_v3_final false 1.0 true 0.25 1.65 0.60
```

重新录制一个同回合渐进包：

```bash
./scripts/record_progressive_stage1.sh <episode_name> 600 10 25
```

第二个参数是最大安全时长，而不是正常结束时间；只有 FALCON FSM 进入 `FINISH` 才生成 complete 包，超时只保留 raw bag。最后一个参数是静止期相机扫描角速度（度/秒）。UAV 模式默认精确跟踪 PositionCommand；地面 VLN 可显式使用 `run_habitat_falcon.py --navmesh-constrained`。

实时 FALCON 运行时，点云过滤、RGB、实际轨迹、当前规划和 Frontier 都在线发布；默认 bag 中的 Boxer 框按同回合首次观测时间渐进回放。当前 Boxer 推理仍是录制后的批处理，并非探索过程中在线运行。
