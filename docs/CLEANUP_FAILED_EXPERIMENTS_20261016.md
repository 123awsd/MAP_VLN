# 失败实验工作区清理

按用户“删除之前失败、没用的东西，保留700秒版本”的要求清理。未提交Git、未回退已有已跟踪修改、未改变探索算法。

## 已完成

- Git展开状态从1286条降到363条：仍为16个已跟踪修改，其余347个未跟踪文件。
- 移除10个旧runtime目录，650份源码/配置逐文件SHA256与保留的运行`source_snapshot`完全相同。仅移除重复的工作区副本，完整运行快照仍可恢复。
- 另84个小型旧工作区条目、275个文件（剩余旧版本副本、一次性诊断脚本、旧报告及补丁）从工作区移除。为避免不可逆丢失，先逐文件校验复制到Git忽略的`outputs/cleanup_recovery_20261016`；恢复索引为该目录`index.json`。
- 运行入口默认版本改为保留的`A3_observation_lifecycle`，共享搜索回归配置指向同一版本，避免继续引用已收起的旧目录。没有新建实验。

## 保留

1. 当前700秒实际版本：`runtime/consistency_validation_20261015/A3_observation_lifecycle`及manifest；全部源码/配置哈希与700秒运行manifest逐项一致。
2. 700秒完整数据：`outputs/coverage_lifecycle_700_r1_20261016`，含bag、捕获、源码快照、有效参数、体积、审计和检查点。
3. R0冻结版本及两轮轻量对照、原`exploration_baseline_restore_504`启动模板。
4. 上一轮生命周期完整验收、最新700秒及HM3D对照报告、关键回归和机器证据。
5. 其它模块和已跟踪的既有源码修改，包括boxer和房间处理，均未删除。

R0与700秒冻结目录清理前后逐文件哈希一致，700秒源码另与运行manifest逐项匹配；保留脚本语法检查和9项纯监控回归通过。没有重新跑ROS编译或探索仿真。

## 尚未删除的大型数据

自动审批拒绝一次性不可逆删除约4255文件/25.11GiB的广泛清单，认为用户没有逐项确认范围、恢复成本高。没有绕过这项拒绝：没有删除任何原始失败运行bag或其源快照。

拟删除大目录清单及精确路径已记录`outputs/consistency_validation_20261015/cleanup_failed_20261016.json`；计划状态仍为planned，不代表已经执行。运行摘要事先保存为同目录`deleted_run_summaries_20261016.json`。这些大型目录本来已在Git忽略范围，不影响Git状态条目数。

如果后续确认删除大型数据，需重新生成清单：重复源码工作区现已移除、旧脚本已收起，不能直接重跑旧广泛计划。

## 目录证据

- `outputs/consistency_validation_20261015/duplicate_source_cleanup.json`：650文件逐一副本路径和哈希。
- `outputs/cleanup_recovery_20261016/index.json`：小文件可恢复收起索引。
- `docs/COVERAGE_700_ACCEPTANCE_20261016.md`：保留当前结果说明。

历史报告中的旧路径仅表示当时证据出处；被收起的运行入口不再作为当前可运行版本。不要将仍保留的失败bag与当前700秒可用版本混淆。

注：复核发现R0已有manifest中`exploration_diagnostics.h`, `exploration_fsm.cpp`, `frontier_finder.cpp`, `frontier_finder.h`, `exploration_manager.cpp`与当前文件不同（清理前即如此，实际差异项以检查结果为准）；本次以清理前逐文件哈希证明未改变R0，不将它谎称为原manifest全部通过。

## 用户再次确认后的最终清理

用户明确确认删除后，已彻底删除35个旧失败/诊断运行和2个重建缓存，约25.11GiB。先前“尚未删除”的段落是审批前过程记录，现由本段取代。容器root创建的文件导致主机权限不足，后使用原专用容器仅按相同清单完成删除。

700秒完整bag、最终网格、当前冻结源码、上一轮生命周期验收、R0两轮轻量对照和原冻结启动模板仍在。当前及R0源码清理前后哈希复核一致；700秒源码同时与运行manifest一致。

部分先前作为650个重复源码恢复来源的旧运行快照也在本次授权列表中，已一并删除，不再声称这650文件仍可从那些快照恢复。小文件收起目录`outputs/cleanup_recovery_20261016`仍保留，但不在Git工作区显示。

最终逐项记录：`outputs/consistency_validation_20261015/confirmed_failed_data_cleanup.json`；失败结论摘要：`deleted_run_summaries_20261016.json`。原广泛清理计划不再用于执行，不能重跑。
