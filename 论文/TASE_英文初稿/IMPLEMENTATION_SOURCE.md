# 论文实现事实来源

目标识别和 MINCO 轨迹优化的论文描述以远程 `real-fly` 分支为准，不以本地 `BYQ` 分支的仿真代码推断真机能力。

- 独立工作树：`/home/byq/Main_vln/MAP_VLN_real-fly`
- 跟踪分支：`origin/real-fly`
- 本次核对提交：`b06986675a2516fda4e7347702a04469c97b75ce`

## SAM 3.1 / Boxer

- SAM 3.1 用于映射阶段的单帧文本提示目标框和掩膜；当前没有使用其视频跟踪能力。
- 掩膜在候选框内过滤背景深度，保留前景和框外场景上下文；Boxer 据此生成 3D 框并进行多视角融合。
- 同帧中空间分离的候选可通过 cannot-link 约束防止错误融合。
- 方法和系统实现章节统一将目标识别模块写为 SAM 3.1--Boxer：SAM 3.1 提供文本提示目标框与掩膜，掩膜深度用于 3D 定位，Boxer 提供 3D 目标支持与多视角实例融合。
- 真机运行时是否已将识别结果接入任务状态转移，只在 Physical UAV Experiments 中按当前实验链路说明，不用来限制方法章节对完整算法的描述。

主要实现入口：

- `boxer_ext/sam31_wrapper.py`
- `boxer_ext/sam2_depth_filter.py`
- `boxer_ext/fusion_frame_constraints.py`
- `patches/boxer/sam31_frontend_and_fusion.patch`
- `config/sam31_backend_manifest.json`
- `scripts/run_boxer_habitat.sh`

## MINCO

- 离散任务--视点路线选定后，主机基于审核的静态地图和安全走廊生成 MINCO 轨迹。
- `final_minco.txt` 保存多项式系数、分段时间、yaw 样本和观察停留时间。
- manifest 绑定原地图、路线、碰撞地图、规划参数和执行包的 SHA256。
- NX 侧校验地图身份、起点、净空和动力学限制后直接执行已保存轨迹，不重新运行 A*、CIRI 或 MINCO。
- 当前实时净空监视只读，未接入自动制动；自动降落禁用。
- 方法章节的轨迹优化统一写为：A* 生成碰撞可行的几何引导路径，安全走廊约束连续可行空间，MINCO 优化平滑且满足动力学限制的连续轨迹。

主要实现入口：

- `real_fly/stage2_offline/scripts/generate_final_minco.sh`
- `real_fly/stage2_runtime/SAVED_MINCO.md`
- `real_fly/stage2_runtime/scripts/saved_minco_artifact.py`
- `real_fly/stage2_runtime/launch/full_smooth_stage2.launch`
- `real_fly/vendor/dls_ws_src/mission_planner/`

