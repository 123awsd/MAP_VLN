# 2026-10-31 定向修改与统计核查

范围：摘要、15 子任务补充实验、Table II 及对应结论、实验参数。Runtime protocol 和真机实验正文保持原文；原始实验数据和实验脚本未改动。

## Table II

读取 `VLN_Baselines/消融/outputs/benchmark_cptg_jtvp_full_20261009/report.json`，每个变体 546 个唯一 slot。

- 完整方法复用正式主实验轨迹。复用分支使用 `habitat_execution.json:elapsed_wall_s`，另外两个变体使用外层进程计时。统一读取完整方法 `source.original_native_result` 的 `task_metrics.task_completion_time_s` 后，CT 从 6.390435296470197 修正为 11.786090331042281 秒；528 个成功回合全部核对 case/seed/run 及成功状态。
- 无 CPTG 的 30 个 native_timeout 曾被排除出 SPL 分母；对应完整方法记录均有适用的参考路径。按失败 SPL=0 纳入后，SPL 从 61.84941736629784% 修正为 58.45109773078697%。SR 不变。
- 固定视角 CT=11.653604483505301 秒，因此移除 JTVP 降低 CT 的原结论，保留 SPL 提升、CT 相近的结论。

复核：`python3 audit_table_ii.py /home/byq/Main_vln`，结果见 `table_ii_audit.json`。脚本只读原始记录。

## 15 子任务

来源 `tmp/scale_capacity_scoped_cases_v4/STEP1_2_REPORT.md` 及其指向的 00337 连续端到端结果：15/15 完成，解析 59.24 秒、导航 89.72 秒、总耗时 148.96 秒，规划调用 p95=7.946 秒，在 240 秒预算内完成。为单次补充验证，不表述为稳定最大容量或无延迟保证。

## 参数来源（相对工作区根目录）

- `MAP_VLN_real-fly/scripts/run_wall_structural_rooms.py`：closing、核心距离/面积、表面/多高度 FREE/轨迹/射线支持、合并门槛。
- `MAP_VLN_real-fly/scripts/iterate_room_planes.py` 和 `run_multifloor_room_pipeline.py`：0.05 m grid。
- `MAP_VLN_real-fly/scripts/classify_coverage700_rooms_multiview.py`：两视图、0.65 confidence。
- `MAP_VLN/stage2/candidate_poses.py` 和 `joint_planner.py`：视角/转移权重；论文用 1 m、1 s 标准尺度表达现有数值。
- `MAP_VLN/stage2/semantic_fusion.py`：0.75 m association、两次新实例支持。
- `MAP_VLN/scripts/run_stage2_habitat.py` 和 `stage2/semantic_region_search.py`：H=3、恢复预算、先验与更新/评分系数。

摘要按空白分词 178 词。移除结尾强制换栏与提前 balance 命令，以适应新增内容并保留 13 页；图件未改。
