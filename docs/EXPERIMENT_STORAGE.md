# 实验数据存储约定

2026-10-15按用户要求清理确认失败的实验。结果、失败原因和完整删除清单统一保存在[失败实验摘要](EXPLORATION_FAILED_TRIALS_SUMMARY_20261015.md)，原失败索引和分散归档已移除。

- 项目：`/home/uav/map_VLN/PRE_MAP_VLN`。
- 后续大型输出：`/data/map_VLN/PRE_MAP_VLN/outputs/<实验名>`；项目`outputs/<实验名>`使用同名软链接。
- 启动前确认`/data`独立挂载；Docker需挂载data盘输出根目录。
- 保留`exploration_baseline_restore_504`、`exploration_threshold_double_half_504`及原认可探索、房间分割结果。不要将历史失败路径当作现存数据。
- 失败实验仅保留结论摘要，避免重新复制整套源码和数百份元数据归档到Git工作区；正在评估的数据需先完成验收，再按用户要求清理。
- 本次未启动新实验，也未修改当前算法入口或回退已有工作。

2026-10-16追加清理：经用户再次确认，35个旧失败/诊断运行和2个中间导出缓存已删除，约25.11GiB。保留700秒完整数据、当前生命周期冻结版本、上一轮验收及R0轻量两轮；详情见`CLEANUP_FAILED_EXPERIMENTS_20261016.md`。历史报告引用的已删路径仅作历史出处，不保证原bag仍存在。
