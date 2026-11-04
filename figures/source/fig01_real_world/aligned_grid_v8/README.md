# 主图 V8：统一网格、紧凑对齐

根据“不够紧凑、像随意排版的 PPT”的反馈，重新组织为同一六列网格。保留白底、实测三维轨迹为主体和不标房间名称的要求。

## 输出

- `main_figure.png`：3600 × 1250，508 dpi；相同宽度下，画布高度比 V7 减少约 24.7%。
- `preview.jpg`：1800 × 625。
- `main_figure.pdf`：180 × 62.5 mm，文字、轨迹与视锥为矢量。
- `main_figure.svg`：自包含、可编辑。
- `build.py`：使用准备好的 `assets/` 独立重新排版。
- `render_views.py`：从原实测体素派生场景重新渲染两种相机视角，导出严格对应的投影数据。

```bash
/usr/bin/python3 build.py
```

## 布局规则

- 画布左右边距各 26 像素，6 列等宽 578 像素，5 个列间距各 16 像素，严格填满 3600 像素宽度。
- 上层三维总览占 4 列，恢复局部占 2 列；两个视图使用同一个 680 像素高的版面区域。
- 下层 6 张照片统一为 578 × 325 像素，共用顶部、底部和图注基线；前 4 张是四个任务的跟拍画面，后 2 张是恢复搜索的机载证据。
- 全图只使用 Liberation Sans 字体，取消各部分不同的字体风格。
- 记录路线、恢复段和起点的图例放进共享的顶部行，移除单独堆放的说明块。

## 几何与图像来源

底图继续使用 V7 的实测占用体积派生场景：0.2 m 占用单元直接生成的边界面，显示高度裁切为 −0.3 至 1.9 m。未推断或补造墙体、地板、家具。

总览相机改为朝向与长走廊主轴对齐的视角，减少斜向包围盒造成的空角；局部相机聚焦于实际 A → 1 的恢复区域，省去无关的远处场景。两者均由 Blender 相机重新投影，不在二维中拉伸建筑。地图与路线、相机视锥分别按同一相机投影坐标与统一尺度放置，坐标变换记录在 `layout_audit.json`。

轨迹是归档 `/ekf_quat/ekf_odom` 记录的机体位置；相机视锥使用归档外参和 1.25 m 显示深度。路线作为矢量投影叠加，不作墙体遮挡测试。

前四张跟拍图分别对应编码时间 53、69、129、163 秒；右侧两张为机载视频 35.5、46 秒。照片来源、裁剪和近似事件对应沿用 V7 的 `assets/evidence_sources.json`。不同视频不声称严格同步；微波炉仅展示检查动作。

本稿仍位于 `source/`，未替换 `figures/final/` 或修改正文。

## Suggested caption

Real-world multi-task execution with LogicFly. (a) The recorded onboard EKF flight trajectory and calibrated observation poses are overlaid on the measured occupancy geometry. (b) The enlarged recovery segment shows movement from the initial laptop check (A) to the subsequent laptop observation (1). Four third-person inspection photographs and two onboard recovery observations share the lower row.
