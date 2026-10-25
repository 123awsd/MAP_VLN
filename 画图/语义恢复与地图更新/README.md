# Recovery rules with abstract visual elements

Fig. 8 独立重绘稿，**88 × 45 mm，单栏尺寸**。当前版本用抽象区域、视锥、观测标记和场景图节点解释恢复规则。无家具、房间、具象目标或大面积彩色阶段框；保留直观图形，不采用纯文字状态机。

## 阅读方式

1. 左侧浅蓝区域及采样点表示一个已搜索区域。三个带叉的视锥表现局部观测失败和视点重试，箭头表示先后关系。变淡的采样点和 `belief ↓` 表示新覆盖的负观测降低区域搜索信念。
2. `Exhausted` 表示当前位置候选耗尽或局部限制到达。对允许恢复的任务，推断并落地其他区域和可行视点，再重新规划；这不是一次没看到就立即换区域。
3. 右侧紫色区域表示另一搜索区域。灰色轮廓是其他候选视点，绿色视锥和勾号表示成功验证；区域内的绿色角框是检测符号，不代表某种语义物体。
4. 两个检测框表示跨视图证据融合，小型图结构高亮新增节点。新实例须满足空间一致和确认条件，执行结束后插入场景图。正验证解决任务的时点仍在执行中。

整个过程保持同一任务。只展示允许语义恢复且最终找到目标的路径；未展开任务级最终未找到、可选房间前沿回退或条件任务激活。区域几何、采样点数量和视锥摆放均为抽象表达，不是运行记录、地图或实测结果。连线表示搜索顺序，不是 MINCO 或其他规划器求得的飞行轨迹。

## 实现依据

`mechanism_audit.py` 对合成输入运行实际 `ViewpointRecovery`、`validate_hypotheses`、`SemanticRegionBelief`、`MissionState` 和 `OnlineSemanticFusion`，核对局部重试、位置耗尽、候选过滤、恢复区域验证、负证据、新覆盖去重、正验证和新实例确认。另检查预算可能阻止仍有候选的局部重试，以及任务级 not-found 结果的含义。

`run_stage2_habitat.py` 在执行循环后调用 `fusion.materialize_scene_graph(...)`，所以末端标注为 `After execution`。时点依据是源代码检查，未声称已运行完整端到端仿真。输入和代码 SHA256、执行校验结果见 `mechanism_evidence.json`。

## 参考与设计取舍

参考过 [Inner Monologue, Fig. 5](https://proceedings.mlr.press/v205/huang23c/huang23c.pdf) 的执行反馈表达、[SayPlan, Figs. 1–2](https://proceedings.mlr.press/v229/rana23a/rana23a.pdf) 的环境与图结构对应，以及 [DovSG, Fig. 3](https://arxiv.org/pdf/2410.11989) 的局部记忆更新表达。已查看原论文配图，仅参考信息组织，不使用其图片或将其算法当作本项目实现。

按用户最新意见，抽象掉场景搭建而保留具有含义的图形。图中不重复解释语义模型、具体目标类别、家具和房间布局，也不拆成多个彩色模块。

## 输出与复现

- `semantic_recovery_en.pdf`：原生矢量 PDF。
- `semantic_recovery_en.svg`：可编辑矢量。
- `semantic_recovery_en.png`：600 dpi PNG。
- `semantic_recovery_en_preview.png`：300 dpi 预览。
- `render_figure.py`：绘图、文字边界与重叠检查。
- `mechanism_audit.py`：实际实现的机制校验。

```bash
cd /home/uav/map_VLN
/home/uav/miniconda3/envs/env_isaaclab/bin/python -B 画图/语义恢复与地图更新/render_figure.py
```

脚本只更新本目录的图稿和审计记录。TASE 中的图件副本保持独立，本次先提供新版图稿查看。

## Suggested caption

**Observation-driven recovery within one task.** Failed verification triggers permitted local viewpoint retries. Newly covered negative observations lower the current region's search belief. Once the location is exhausted or its local limit is reached, recovery grounds alternative regions and feasible views for replanning. A positive verification resolves the task online. Consistent novel detections are fused across views, and confirmed instances are inserted into the scene graph after execution. Colored planes and frusta are abstract regions and observation poses; arrows show search order, not a flight trajectory. The task-level terminal not-found exit is omitted.
