# 失败探索实验清理摘要（2026-10-15）

用户要求只保留摘要，删除失败实验。以下失败版本未晋升为基线；删除后不再保留它们的源码快照、录包、缓存、图像、日志和专用脚本。

## 保留的有效工作与位置

- 原始认可基线：outputs/exploration_recovery_local_trial/round6_extended；对应后续保存提交277fcf7。
- 冻结基线复跑：outputs/exploration_baseline_restore_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_baseline_restore_504。已知体积1133.322m³，路程442.285m，分层346.730/494.844/291.748m³。
- 既有较好对照：outputs/exploration_threshold_double_half_504。
- 当前入口、共用诊断工具、房间分割、墙结构和其他未确定失败的工作保留。已有16个跟踪文件的修改保持原样；未执行git reset、git clean或覆盖基线。
- 后续大型输出仍存/data/map_VLN/PRE_MAP_VLN/outputs；项目outputs通过同名软链接定位。

## 已确认的经验与因果边界

- 发现代码缺陷不等于证明探索退化主因。配置不变也不等于行为不变；此前一次同时调整速度反馈、时间戳、轨迹交接、停止方式、碰撞处理和时间分配，无法单独归因。
- 基线反馈速度恒零、旧位姿重新盖时间戳、轨迹晚于标称起点发布、多段轨迹时间未初始化等是代码/日志发现；不能把任一项未经隔离验证地解释为所有漏房间的根因。
- 最后接口修复v2完整运行504s：514.323m³，172.406m；分层330.057/182.311/1.955m³。2号旁卫生间进入，9号右上未进入，三楼未实际飞入。
- 最后修复轮421条发布迟发0；2518帧采集时间单调，速度传输与采集差分一致；321个碰撞轨迹编号均处理并确认，322次停止位置检查误差0。但这些局部指标不能作为探索成功验收。
- 约203s开始同处受阻；其后320次有效碰撞处理、轮换38个远处目标，目标距离中位数4.58m；213s后位置各轴变化不足7cm。局部碰撞后屏蔽远处视点20秒并换目标，没有解决当前位置离开问题。
- 确认了反复失败循环，尚未通过反事实验证确定哪项改动造成首次进入该状态。不要将“找到循环”写成“全部根因解决”。
- 第一轮接口诊断因用当前时刻预测与采集时刻位置比较而中止；v2已修正该时间对齐错误，但全局效果仍未通过。
- A*超时与空间不可达、碰撞模型差异及目标完成判断仍需用同一失败片段核对，不能再用大量参数组合替代因果验证。

## 历史结果汇总

体积为固定高度带内FREE+OCCUPIED已知体积，并非真实场景覆盖率。short_test_only不是完整验证；requires_review也不代表通过。下表基线被历史自动规则标成not_accepted，仍按用户指定保留为对照。

| 实验 | 原状态/验收 | 总体积m³ | 1/2/3层m³ | 路程m | 本次处理 |
|---|---|---:|---|---:|---|
| exploration_execution_contract_504 | audited/not_accepted | 940.633 | 329.443/307.550/303.640 | 306.808 | 此前已清理 |
| exploration_execution_contract_v2_504 | audited/not_accepted | 855.101 | 329.329/357.673/168.099 | 269.729 | 此前已清理 |
| exploration_observation_lifecycle_120 | audited/short_test_only | 323.799 | 320.231/3.568/0.000 | 105.467 | 此前已清理 |
| exploration_observation_lifecycle_504 | audited/not_accepted | 833.274 | 343.204/479.628/10.442 | 384.686 | 此前已清理 |
| exploration_arc_prefix_504 | audited/not_accepted | 914.999 | 344.067/480.609/90.323 | 406.115 | 此前已清理 |
| exploration_single_local_owner_504 | audited/not_accepted | 339.415 | 314.843/24.572/0.000 | 94.766 | 此前已清理 |
| exploration_exact_lattice_504 | interrupted/not_accepted | 468.152 | 320.932/141.872/5.348 | 未知 | 此前已清理 |
| exploration_baseline_restore_504 | audited/not_accepted | 1133.322 | 346.730/494.844/291.748 | 442.285 | 保留基线 |
| exploration_baseline_speed3_504 | audited/requires_review | 986.863 | 342.075/352.350/292.438 | 361.748 | 此前已清理 |
| exploration_clearance001_diagnostic_60 | audited/short_test_only | 299.025 | 298.778/0.247/0.000 | 53.713 | 删除 |
| exploration_baseline_clearance001_diagnostic_504 | interrupted/not_accepted | 346.895 | 342.194/4.701/0.000 | 未知 | 删除 |
| exploration_baseline_clearance001_bathroom_504 | audited/requires_review | 744.889 | 328.305/410.555/6.029 | 321.858 | 删除 |
| exploration_native_handoff_baseline_504 | running_or_incomplete/requires_review | 332.822 | 323.667/9.155/0.000 | 未知 | 删除 |
| exploration_native_search_baseline_504 | audited/requires_review | 690.125 | 336.348/349.688/4.089 | 288.782 | 删除 |
| exploration_native_completion_baseline_504 | audited/not_accepted | 961.542 | 344.131/359.924/257.487 | 374.390 | 删除 |
| exploration_native_handoff_v2_baseline_504 | audited/not_accepted | 832.057 | 340.204/481.427/10.426 | 319.040 | 删除 |
| exploration_native_combined_baseline_504 | audited/requires_review | 886.193 | 304.397/487.396/94.400 | 266.796 | 删除 |
| exploration_handoff_transaction_504 | audited/not_accepted | 803.632 | 351.245/447.809/4.578 | 340.071 | 删除 |
| exploration_regional_selection_504 | audited/requires_review | 834.066 | 350.701/475.730/7.635 | 300.431 | 删除 |
| exploration_single_selection_504 | audited/requires_review | 799.814 | 354.829/432.018/12.967 | 325.932 | 删除 |
| exploration_abc_A_smoke90 | audited/short_test_only | 227.702 | 222.867/4.835/0.000 | 58.854 | 删除 |
| exploration_abc_A_504 | audited/not_accepted | 797.975 | 336.262/456.739/4.974 | 291.819 | 删除 |
| exploration_abc_control_504 | audited/not_accepted | 991.162 | 334.160/370.436/286.566 | 420.335 | 删除 |
| exploration_abc_AB_candidate_504 | audited/not_accepted | 754.151 | 340.236/400.704/13.211 | 275.121 | 删除 |
| exploration_execution_async_smoke90 | audited/short_test_only | 323.478 | 297.957/25.521/0.000 | 63.807 | 删除 |
| exploration_execution_async_504 | audited/not_accepted | 1082.509 | 342.672/473.480/266.357 | 399.546 | 删除 |
| exploration_execution_timing_smoke90 | audited/short_test_only | 314.259 | 312.179/2.080/0.000 | 61.293 | 删除 |
| exploration_execution_timing_504 | audited/not_accepted | 974.432 | 332.910/480.695/160.827 | 330.707 | 删除 |
| exploration_search_geometry_smoke90 | audited/short_test_only | 17.110 | 17.110/0.000/0.000 | 0.000 | 删除 |
| exploration_search_geometry_v2_smoke90 | audited/short_test_only | 310.766 | 309.771/0.995/0.000 | 75.514 | 删除 |
| exploration_search_geometry_shared_smoke60 | audited/short_test_only | 257.261 | 257.207/0.054/0.000 | 50.784 | 删除 |
| exploration_baseline_interface_fix_504 | running_or_incomplete/requires_review | 659.803 | 329.797/320.693/9.313 | 未知 | 删除 |
| exploration_baseline_interface_fix_v2_504 | audited/not_accepted | 514.323 | 330.057/182.311/1.955 | 172.406 | 删除 |

## 删除清单

实验输出目录：25个（连同项目软链接）。
- outputs/exploration_clearance001_diagnostic_60 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_clearance001_diagnostic_60
- outputs/exploration_baseline_clearance001_diagnostic_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_baseline_clearance001_diagnostic_504
- outputs/exploration_baseline_clearance001_bathroom_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_baseline_clearance001_bathroom_504
- outputs/exploration_native_handoff_baseline_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_native_handoff_baseline_504
- outputs/exploration_native_search_baseline_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_native_search_baseline_504
- outputs/exploration_native_completion_baseline_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_native_completion_baseline_504
- outputs/exploration_native_handoff_v2_baseline_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_native_handoff_v2_baseline_504
- outputs/exploration_native_combined_baseline_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_native_combined_baseline_504
- outputs/exploration_handoff_transaction_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_handoff_transaction_504
- outputs/exploration_regional_selection_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_regional_selection_504
- outputs/exploration_single_selection_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_single_selection_504
- outputs/exploration_abc_A_smoke90 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_abc_A_smoke90
- outputs/exploration_abc_A_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_abc_A_504
- outputs/exploration_abc_control_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_abc_control_504
- outputs/exploration_abc_AB_candidate_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_abc_AB_candidate_504
- outputs/exploration_execution_async_smoke90 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_execution_async_smoke90
- outputs/exploration_execution_async_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_execution_async_504
- outputs/exploration_execution_timing_smoke90 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_execution_timing_smoke90
- outputs/exploration_execution_timing_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_execution_timing_504
- outputs/exploration_search_geometry_smoke90 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_search_geometry_smoke90
- outputs/exploration_search_geometry_v2_smoke90 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_search_geometry_v2_smoke90
- outputs/exploration_search_geometry_shared_smoke60 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_search_geometry_shared_smoke60
- outputs/exploration_baseline_interface_fix_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_baseline_interface_fix_504
- outputs/exploration_baseline_interface_fix_v2_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_baseline_interface_fix_v2_504
- outputs/exploration_decision_revision_baseline_504 → /data/map_VLN/PRE_MAP_VLN/outputs/exploration_decision_revision_baseline_504

失败实现目录：11个。
- runtime/falcon_root_ablation_20261015
- runtime/falcon_baseline_interface_fix_20261015
- runtime/falcon_baseline_interface_fix_v2_20261015
- runtime/falcon_search_geometry_20261015
- runtime/falcon_search_geometry_shared_20261015
- runtime/falcon_execution_async_20261015
- runtime/falcon_execution_timing_20261015
- runtime/falcon_decision_revision_20261015
- runtime/falcon_handoff_transaction_20261015
- runtime/falcon_regional_selection_20261015
- runtime/falcon_single_selection_20261015

专用脚本：24个。
- scripts/add_verified_corridor_cache.py
- scripts/analyze_abc_trial_regression.py
- scripts/analyze_combined_missing_room.py
- scripts/analyze_native_ablation_failures.py
- scripts/audit_abc_state_feedback.py
- scripts/audit_baseline_interface_fix.py
- scripts/compose_native_ablation.py
- scripts/finalize_execution_contract_trial.py
- scripts/finalize_native_ablation_batch.py
- scripts/prepare_abc_parallel.py
- scripts/prepare_abc_stage_a.py
- scripts/prepare_baseline_interface_fix.py
- scripts/prepare_double_half_batch.py
- scripts/prepare_execution_async.py
- scripts/prepare_root_ablation_batch.py
- scripts/prepare_search_geometry_revision.py
- scripts/prepare_sequential_revision_trial.py
- scripts/render_baseline_combined_floor_comparison.py
- scripts/run_execution_contract_trial.sh
- scripts/run_native_ablation_batch.sh
- scripts/summarize_execution_revision.py
- scripts/summarize_native_ablation.py
- scripts/summarize_structural_trials.py
- scripts/unify_astar_execution_edges.py

重复文档、索引及旧失败归档已合并到本文件。原归档包含以下已清理实验：
exploration_arc_prefix_504, exploration_baseline_speed3_504, exploration_clock2_costrole_smoke_60, exploration_costrole_3mps_120, exploration_costrole_3mps_clock2_504, exploration_doublehalf_v2_cross9_504, exploration_doublehalf_v2_gain80_504, exploration_doublehalf_v2_near03_504, exploration_doublehalf_v2_repeat_504, exploration_exact_lattice_504, exploration_executed_velocity_smoke, exploration_execution_contract_504, exploration_execution_contract_smoke_90, exploration_execution_contract_v2_504, exploration_flight_2mps_control_120, exploration_flight_3mps_120, exploration_frontier_dependency_trial, exploration_goal_unified_local_504, exploration_height_aware_candidates_trial, exploration_low_frontier_trial, exploration_observation_lifecycle_120, exploration_observation_lifecycle_504, exploration_retained_fine_504, exploration_retained_goal_trace_504, exploration_retained_unknown0_504, exploration_rootfix_doublehalf_504, exploration_rootfix_doublehalf_v2_504, exploration_round6_cross_floor_cost30_trial, exploration_single_local_owner_504, exploration_structural_execution_504, exploration_structural_search_504, exploration_threshold_gain48_504, exploration_threshold_triple_half_504, exploration_threshold_visible4_504

## 清理验证

清理完成：删除11个失败实现目录、25个实验输出目录及同名软链接、24个专用脚本；重复文档与217个旧归档文件合并后移除。删除清单占用约28.87GiB。Git状态由922项降为106项（16个已有文件修改、90个未跟踪文件）。

逐文件哈希检查：已有跟踪文件及关键基线配置、结果均未改变；删除清单无残留。保留脚本未导入已删除的专用脚本模块。基线、较好对照及房间分割数据仍可访问。

当前默认工作区原有修改仍在，不能把它与冻结基线source_snapshot等同；本次清理没有进行算法回退或新实验。
