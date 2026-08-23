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

最终同步包：`outputs/bags/hm3d_stage1_progressive_final.bag`。它包含同一次探索的 Habitat RGB/Depth、FALCON 轨迹与 frontier、地图点云、机体位姿，以及按首次稳定观测时间渐进出现的 Boxer 3D boxes。

```bash
cd /home/uav/map_VLN/PRE_MAP_VLN
./scripts/replay_bag_rviz.sh
```

脚本会自动复制当前桌面的 X11 授权到项目内的忽略文件，不修改系统级 X11 配置。

循环播放：

```bash
./scripts/replay_bag_rviz.sh hm3d_stage1_progressive_final true
```

bag 播放完成后 RViz 会保持打开。第三个参数可调整回放倍速，例如 `./scripts/replay_bag_rviz.sh hm3d_stage1_progressive_final false 2.0`。

重新录制一个同回合渐进包：

```bash
./scripts/record_progressive_stage1.sh <episode_name> 40 10
```
