# PRE-MAP-VLN-Bench-v1

## 评测范围

本 benchmark 固定为 10 个 Matterport3D 场景、每场景 10 条复杂多目标指令，共 100 条。所有指令包含 4 个主要目标和显式先后约束；其中：

- 40 条为顺序 + 空间观察位姿约束；
- 30 条在上述长指令中增加条件分支，15 条受控进入分支、15 条受控跳过；
- 30 条在上述长指令中增加 1 个历史位置失效目标，其中 20 条应通过语义区域主动搜索重新发现，10 条应搜索耗尽。

条件和目标丢失是长指令上的附加属性，不是与普通多目标导航割裂的短任务。每条任务同时保存细粒度 `category` 和 `tags`，后续论文可以将前两类合并统计而不需要重跑数据。

## 数据边界

规划器只读取机器人预探索生成的 occupancy grid、OccuSG 房间图和 Boxer 3D 实例，不读取官方语义真值。Matterport3D 官方语义对象仅作为 held-out 质量审计，统计机器人地图中可与真值类别和位置对应的实例数。

第一阶段离线使用 214 类室内词表；第二阶段在线 OWLv2 只激活当前 task graph 中出现的少量类别。Qwen 只用于结构物体过滤和目标丢失后的房间—功能区域语义先验，不承担逐帧目标检测。目标丢失搜索仍由 A* 到达代价、语义区域覆盖代价、观察质量与负观测 belief 更新共同决策。

## 可恢复目录

重数据根目录默认为 `/shared/PRE_MAP_VLN_benchmark_v1`：

- `episodes/`：第一阶段 RGB-D/pose 关键帧；
- `maps/`：Boxer、导航/结构栅格、OccuSG、scene graph；
- `tasks/`：冻结的 100 条自然指令及任务图；
- `runs/prepared/`：精确目标实例、候选观察位姿、几何最优与固定顺序参考路径；
- `runs/executions/`：100 条 Habitat 闭环轨迹和检测证据；
- `bags/representative/`：三类代表性 RViz bag；
- `reports/`：第一阶段质量、逐任务指标、最终验收；
- `recovery_snapshots/`：系统中断时被完整归档的半成品，不覆盖或伪装成成功结果。

场景和任务均以状态文件为恢复粒度。入口会跳过已经通过的场景和哈希一致的任务执行结果。为了不让长时间 GPU 作业依赖 Codex 会话，推荐分两段在本机后台运行；每段都会写 PID、独占锁、持续日志和最终退出码。

第一段只完成 10 个场景的预探索、语义建图和质量门：

```bash
PRE_MAP_VLN_BENCHMARK_ROOT=/shared/PRE_MAP_VLN_benchmark_v1 \
PRE_MAP_VLN_QWEN_BUDGET_CNY=20 \
./scripts/start_benchmark_phase.sh maps

./scripts/benchmark_status.sh maps
```

状态变为 `state=passed` 后，第二段生成并执行 100 条任务、汇总指标、构建三条代表性 bag 并做最终验收：

```bash
PRE_MAP_VLN_BENCHMARK_ROOT=/shared/PRE_MAP_VLN_benchmark_v1 \
PRE_MAP_VLN_QWEN_BUDGET_CNY=20 \
./scripts/start_benchmark_phase.sh tasks

./scripts/benchmark_status.sh tasks
```

第二段在启动时会重新严格检查十张地图，地图不合格就直接拒绝执行，避免生成无效 benchmark。系统中断后重复执行同一段即可续跑；旧的半成品会保留到恢复快照。调试时可前台运行 `run_benchmark_maps_v1.sh` 或 `run_benchmark_tasks_v1.sh`，原来的 `run_full_benchmark_v1.sh` 仍保留为两段串行入口。

## 指标

`scripts/evaluate_benchmark_v1.py` 自动汇总：

- 任务预期结果成功率；
- 顺序约束、条件分支和恢复结果满足率；
- 实际路径长度、冻结地图上的初始几何最优路径及路径效率；
- 仿真墙钟时间、飞行帧数和本地开放检测延迟；
- 按场景与按任务类别的独立汇总。

机器可读结果为 `reports/benchmark_report.json` 和 `reports/episodes.csv`，可直接放入论文记录的表格为 `reports/summary.md`；第一阶段逐场景覆盖、房间、实例和 Frontier 末态审计另存为 `reports/stage1_quality.json/.csv`。

受控 stale/found ID 与恢复任务的 `rediscovered/exhausted` 结果始终写入 episode 和 execution trace，用于确定性测试条件逻辑与恢复闭环，不冒充自然检测精度。普通目标仍由本地开放检测真实判定；报告另列不受控制的终端检测成功率和所有终端的原始 OWLv2 成功率。

## 可视化

任意完成的 Stage-1 场景都可直接从共享盘回放掀顶点云、RGB、轨迹、Frontier、相机视锥和渐进语义 Box，无需重新探索或重新录包：

```bash
./scripts/replay_benchmark_stage1_rviz.sh Z6MFQCViBuw false 2.0
./scripts/replay_benchmark_stage1_rviz.sh QUCTc6BB5sX false 2.0
./scripts/replay_benchmark_stage1_rviz.sh TbHJrupSAjP false 2.0
```

建议一次只打开一个场景，避免多个 1–3 GB bag 同时回放抢占磁盘和 RViz 渲染资源。

最终验收生成三条代表性 bag：普通顺序/位姿、条件分支、语义恢复。每条都包含完整 RGB、掀顶点云、橙色累计真实轨迹、动态全局/局部规划、房间边界与标签、任务/上下文 box、当前任务状态、目标五角星、UAV 和真实相机视锥。恢复任务还会在激活后显示失效位置和候选语义区域。

```bash
./scripts/replay_benchmark_rviz.sh Z6MFQCViBuw_01 false 1.0
./scripts/replay_benchmark_rviz.sh Z6MFQCViBuw_05 false 1.0
./scripts/replay_benchmark_rviz.sh Z6MFQCViBuw_08 false 1.0
```

bag 播放结束后 RViz 保持打开；第三个参数统一控制 RGB、轨迹、box 和状态回放倍速。
