# 两个卫生间均实际进入：700秒版本

当前提交保存`A3_observation_lifecycle`冻结版本，以及对应运行、止损监控和核心验证脚本。700秒运行中2号旁卫生间进入47帧、9号进入68帧；这是进展，不代表两个区域已完整观测，也不代表整体效率超过R0。

## 复现

需已配置的ROS Noetic/FALCON工作空间与专用容器`consistency-audit-20261015`、项目挂载`/workspace/shared`、`/data`产物目录，以及项目`.envs/habitat`环境。HM3D原始资产不随Git分发。

```bash
python3 scripts/run_coverage_driven.py A3_observation_lifecycle <新的运行名> --resource-cap 700
```

当前源码与桥接快照位于`runtime/consistency_validation_20261015/A3_observation_lifecycle`；manifest记录完整源码哈希。`R0/config.yaml`仅为当前配置对照，不表示本提交重新整理或晋升了R0全部源码。

本轮700秒完整bag和地图位于`outputs/coverage_lifecycle_700_r1_20261016`（外部数据，不随Git提交）；小型可追踪结果保存在`validation/coverage700`。复现需同场景、种子7、planner yaw、0.07m执行模型和修复壳体。同步运行仍可能因计算时序产生路线差异，不能承诺逐帧一致。

启动模板已独立纳入`config/trials/falcon_file_bridge.launch.template`，不再依赖被Git忽略的历史trial.launch。除该入口依赖整理和回归fixture路径外，未改动700秒冻结源码。时间为实际相机采集墙钟区间，不是独立物理时钟。

## 验证

- 当前全部冻结文件哈希与700秒manifest一致。
- 700秒实际任务生命周期、反馈与安全接管检查见验收报告及机器结果。
- 无仿真回归：`python3 scripts/test_coverage_progress_monitor.py`。
- 已有ROS工作空间中可编译/执行`test_observation_lifecycle_regression.py`及`run_observation_lifecycle_regression.sh`；它验证真实manager状态边界，不是历史地图重放。共享搜索回归亦保留。
- 普通/ROS脚本先做Python语法、Shell与启动XML检查；这不替代整机重复验收。

## 下一步优化顺序

1. **细分候选前处理开销。** 本轮459次规划调用总计182.077s，其中候选搜索前178.165s、候选处理区间约3.899s。该前处理区间包括全局选点与评分，不能把178s都叫重复地图更新。先为Frontier更新、HGrid/Tour、候选构建、可见性/独立增益和路径代价分别计时，确认占比后再只优化最大项。保持评分、候选集合与安全检查结果一致。
2. **减少旧轨迹自然到期前交付不足。** 54次SEARCH_PENDING停止中41次原轨迹已到期；另21次active_expired_before_plan。不能直接延长过期轨迹或删除安全停止。需要核对可用运动时域与前处理预算，然后考虑复用同一地图版本的纯计算结果或减少重复计算，而非继续改净空。
3. **残余一二楼机会优先于卫生间专用规则。** 二楼仍有13.12m²、一楼3.62m²历史视点经几何检查的未匹配机会；这不等于本轮在线可达。700秒两个旧重点区本轮已大幅补齐，不再据旧遗漏强改收益或新增区域任务层。

对任何后续优化，先用固定输入对照候选xyz/yaw/gain/utility和最终选择，再以最多700秒比较共同300/503秒体积、实际新观测、停止、安全。若路线变化，不能把单次体积差直接当成算法提升；有明显收益才重复，不扩大无效矩阵。

当前仅给出上述证据驱动的优化方向，没有启动新实验或合并下一项算法改动。
