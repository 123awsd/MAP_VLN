# PRE_MAP_VLN

Habitat 中的无人机预探索、全局 3D 语义地图与长时程多任务 VLN 工程。

第一阶段使用 FALCON 完成自主探索，以 Boxer 构建全局 3D 物体框、OccuSG 划分房间区域。第二阶段使用 Qwen VLM 生成 task graph，在真实占据地图上联合优化任务顺序和观察位姿；目标在旧位置未找到时，Qwen 生成可审计的房间—功能区域先验，3D地图实例化任务相关搜索区域，几何模块联合到达、区域覆盖和观察代价滚动重规划。

项目状态和对话迁移入口见 [`记忆/README.md`](记忆/README.md)。

## 新用户入口

第一次安装、U盘迁移目录、依赖/权重/场景清单以及最短运行流程见 [`docs/安装与使用.md`](docs/安装与使用.md)。大型场景、模型、环境和运行产物不进入 Git，密钥不得随项目或U盘资产目录传播。

## 当前状态

语义区域恢复闭环已实现：支持支撑面、地面邻域、家具周围、低视角遮挡区和固定实例；负观测按新增覆盖和观察质量更新 belief，语义区域耗尽后才启用房间任务 Frontier。成功恢复与预算耗尽两类 Habitat 回归、74 项全仓库测试通过。恢复候选只允许引用真实场景图 ID，Qwen 不生成坐标、不做逐帧检测，同一目标一次恢复最多调用一次。完整说明见 [`docs/第六阶段语义恢复搜索.md`](docs/第六阶段语义恢复搜索.md)。跨任务物品位置规律学习已冻结，不属于当前课题。

10 场景 × 10 条长指令的 `PRE-MAP-VLN-Bench-v1` 已具备独立于 Codex 的可恢复流水线、自然指令生成器、候选/几何规划严格校验、100 任务闭环执行、分类指标、三类代表性 bag 和最终 acceptance gate。协议与数据边界见 [`docs/PRE-MAP-VLN-Bench-v1.md`](docs/PRE-MAP-VLN-Bench-v1.md)；只有共享盘中的 `reports/acceptance.json` 为 `passed` 时才代表该 benchmark 真正完成。

房间表示已规范化为 6 个语义房间与 4 个 corridor；单层高度带与 HM3D `stairs/stairs railing` 语义禁行区防止楼梯和下层混入 2D 规划。2 个家具遮挡源碎片只保留为来源证据。官方 navmesh、探索占据图、楼梯禁行区和房间分割的同坐标对照见 [`docs/房间分割与真值对比.md`](docs/房间分割与真值对比.md)。

两个阶段的工程闭环均已在公开 HM3D example `00861-GLAQ4DNUx5U` 上通过。第二阶段现以本地 GPU OWLv2 为默认开放词表检测器，在线演示完成 4 个实际访问任务、4 次动态重规划和 74 帧连续 RGB；找到电视后正确跳过条件柜子任务。联合初始路径比固定顺序短 32.09%。详细结果见 [`docs/第一阶段设计.md`](docs/第一阶段设计.md) 与 [`docs/第二阶段设计.md`](docs/第二阶段设计.md)。

论文级规划评测已增加 4 类 task graph、6 个基线/消融方法以及 3 scenes × 3 seeds × 5 task cases 的 270 次实验。实验设计、指标和结论边界见 [`docs/第三阶段实验评测.md`](docs/第三阶段实验评测.md)。

开放词表感知已升级为异步 OWLv2、RGB-D 多帧三维融合、未知物体两视角确认和最多 3 个观察位姿的失败恢复。三地图 144 帧 pilot 的平均/p95 延迟为 35.78/36.89 ms；带 Semantic GT 的开发图上 image-level micro precision/recall/F1 为 0.658/0.481/0.556。完整设计和结论边界见 [`docs/第四阶段开放词表感知.md`](docs/第四阶段开放词表感知.md)。

开源方法对比已接入 Open-Nav、VLN-Zero 和 Spatial-X 的固定源码版本，建立了共享的 Habitat 0.1.7/Python 3.8 隔离环境、Open-Nav 官方 R2R-CE 100 episode、10 个所需 MP3D 场景、航点/视觉权重以及统一 NE/OSR/SR/SPL/nDTW 指标。Open-Nav + Qwen + 本地 OWLv2 的真实单 episode 闭环已通过；接入状态和结论边界见 [`docs/第五阶段开源方法对比.md`](docs/第五阶段开源方法对比.md)。

第二阶段一键运行、三地图几何回归和 RViz 回放：

```bash
cd /home/uav/map_VLN/PRE_MAP_VLN
./scripts/run_stage2_demo.sh
./scripts/run_stage2_open_vocab_demo.sh
./scripts/run_stage2_multiscene.sh
./scripts/run_stage2_experiments.sh
.envs/habitat/bin/python scripts/evaluate_open_vocab_multiscene.py --views-per-scene 48
./scripts/validate_stage2_perception.py
./scripts/replay_stage2_rviz.sh hm3d_stage2_perception_v2_complete false 1.0
.envs/habitat/bin/python scripts/check_external_baselines.py
.envs/habitat/bin/python -m unittest tests.test_external_baselines
```

最终 6 demo 可完整重建并验收：

```bash
cd /home/uav/map_VLN/PRE_MAP_VLN
./scripts/run_final_demos.sh
.envs/habitat/bin/python scripts/validate_final_demos.py

# 条件任务与恢复搜索回放（loop=true，跑完不会退出）
./scripts/replay_stage2_rviz.sh final_demos/conditional_01_branch true 1.0
./scripts/replay_stage2_rviz.sh final_demos/recovery_01_tv true 1.0
```

Qwen 和 Matterport 凭据通过 `scripts/configure_secrets.py` 写入 Git 忽略的 `.secrets/`，不得写入命令记录、文档或提交。Qwen 只负责 task graph 和可选低置信度复核，不用于连续目标检测；预算由 `PRE_MAP_VLN_QWEN_BUDGET_CNY` 配置并写入审计 ledger。

已有探索 episode 的后处理命令：

```bash
cd /home/uav/map_VLN/PRE_MAP_VLN
./scripts/run_stage1_postprocess.sh hm3d_stage1
.envs/habitat/bin/python scripts/validate_stage1.py
```

大型数据、环境、权重、第三方仓库和实验输出均在本目录内但不进入 Git。OccuSG 构建必须使用 `./scripts/build_occusg.sh`，它会检查可用内存并将 C++ 编译限制为最多两个任务。

## RViz 与 rosbag

默认同步包：`outputs/bags/hm3d_stage1_complete_v3_indoor_v1_final.bag`。它复用 FALCON 明确进入 `FINISH` 的完整探索回合：248.303 秒、2,479 帧 Habitat RGB、43,332 条消息。第一阶段在探索后对 497 个 RGB-D/Pose 关键帧用 54 类稳定室内词表离线检测；OWLv2 阈值 0.20、融合阈值 0.45、至少 4 帧支持，得到 89 个多视角 3D boxes，并按 71 个首次观测事件渐进注入。旧 10 类包 `hm3d_stage1_complete_v3_final.bag` 保留作对照。

```bash
cd /home/uav/map_VLN/PRE_MAP_VLN
./scripts/replay_bag_rviz.sh
```

脚本会自动复制当前桌面的 X11 授权到项目内的忽略文件，不修改系统级 X11 配置。

回放默认只打开参考 FUEL `office3.gif` 的“掀顶式”3D/RGB 主窗口，不再打开 2D 俯视窗口。白底地图使用按 Z 高度在青—蓝—洋红之间变化的细点配色，而不是大块实心 voxel 或完整彩虹；地板高度由无人机高度在线推断，天花板按每个 XY 栅格柱的局部最高表面剥离，因此斜顶也能随坡度被去掉。深蓝细线为 Habitat 实际已执行轨迹，琥珀色为 FALCON 当前 B-spline，Frontier 保留 FALCON 的簇颜色但仅用 1 px 半透明点显示；红色细视锥直接跟随实际 `camera_optical` 姿态，与实时 RGB 朝向一致。渐进 Box 在显示层改为 0.020 m 的略粗深色线框，不改变包内原始数据和出现时序。

循环播放：

```bash
./scripts/replay_bag_rviz.sh hm3d_stage1_complete_v3_indoor_v1_final true
```

bag 播放完成后 RViz 会保持打开。第三个参数是回放倍速：`0.5` 为半速，`2.0` 为 2 倍速，`4.0` 为 4 倍速；例如：

```bash
./scripts/replay_bag_rviz.sh hm3d_stage1_complete_v3_indoor_v1_final false 2.0
```

倍速会同步作用于 RGB、点云、轨迹和 Box。机器负载较高时建议使用 2–4 倍速，过高可能使 RViz 来不及渲染每一帧。第四个参数仅用于按需重新启用 2D 俯视窗口，默认是 `false`。

第五至七个参数依次调整离地净空、开始识别顶棚的最小室内高度和沿局部顶面的剥离厚度。例如使用默认的 0.25/1.65/0.60 m：

```bash
./scripts/replay_bag_rviz.sh hm3d_stage1_complete_v3_indoor_v1_final false 1.0 false 0.25 1.65 0.60
```

重新录制一个同回合渐进包：

```bash
./scripts/record_progressive_stage1.sh <episode_name> 600 10 25
```

第二个参数是最大安全时长，而不是正常结束时间；只有 FALCON FSM 进入 `FINISH` 才生成 complete 包，超时只保留 raw bag。最后一个参数是静止期相机扫描角速度（度/秒）。UAV 模式默认精确跟踪 PositionCommand；地面 VLN 可显式使用 `run_habitat_falcon.py --navmesh-constrained`。

实时 FALCON 运行时，点云过滤、RGB、实际轨迹、当前规划和 Frontier 都在线发布；默认 bag 中的 Boxer 框按同回合首次观测时间渐进回放。当前 Boxer 推理仍是录制后的批处理，并非探索过程中在线运行。
