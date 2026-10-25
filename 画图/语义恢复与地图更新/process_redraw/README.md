# Fig. 8：英文过程版

保留已确认的 HM3D 完整住宅场景，重新组织为 5 个动作：Inspect armchair → Try another view → Search another room → Verify cushion → Update object map。左侧观察、中间恢复、右侧验证与地图更新按编号阅读。场景中的 1、2、4 与对应观察步骤一致。

## 交付

- `fig8_process_en.png`：英文预览。
- `fig8_process_en.pdf`：论文用 PDF，184 × 98.13 mm，最小字号约 7.39 pt。
- `fig8_process_en.svg`：自包含可编辑源文件；图像嵌入，文字、路线、箭头、节点均为矢量。
- `compose_figure.py`：可重现排版脚本。
- `caption.tex`：对应英文图注。
- `provenance.json`：图像来源、裁剪、示意边界和验证结果。

## 表达调整

放大关键 RGB 观察，采用连续编号和动作标题；去掉起点进场路径、无关房间名称、覆盖采样图例和复杂包围框。蓝色路线只表达从卧室换到客厅，橙色对应未检出和局部重试，绿色对应正验证及新增实例。保留同一任务持续、达到局部限制后才恢复，以及执行中验证和执行后插入之间的时序。

新覆盖和重复覆盖的区别、锚物体保留、多视角关联等机制细节在图注中说明。图中不展示未经记录的定量置信度。

## 数据和示意边界

场景复用 `../multiroom_redraw/assets/cutaway.png`，来自 HM3D `00166-RaYrxWt5pR1` 的真实纹理网格；完整 Blender 场景位于 `../multiroom_redraw/fig8_textured_scene.blend`。RGB 图来自已有 Habitat 视图，通过 SVG 裁剪展示，未进行生成式补绘。

路线沿用前一版经过官方 navmesh 通道位置检查的展示路线，不是算法实测结果；视锥、视点和相机图之间是机制示意关系，不作为标定后的轨迹回放。示意中假设靠垫不在初始物体地图里，以表达新增实例流程；不声称源运行实际新增了该靠垫。

该版独立保存在本目录；论文中的原图、正文、任务日志和原始相机图均未修改。

## 重现

在项目根目录执行：

```bash
/usr/bin/python3 画图/语义恢复与地图更新/process_redraw/compose_figure.py
```
