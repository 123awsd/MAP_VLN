# 主图 V9：将 b 改为真实恢复观测

针对“b 这个图不合适”的反馈，仅重做右侧 b 区。a 区保留 V8 的完整三维飞行轨迹、四个任务的跟拍图与所有位置、尺寸和文字。

## 修改

- 移除 b 中重复的局部三维放大图。
- b 改为两张较大的机载真实画面：A 处检查未找到笔记本，随后在位置 1 找到笔记本。
- 两张机载图统一为 1172 × 475 像素，上下排列，并与左侧的整体上、下边界对齐。
- 底部只保留四个任务的跟拍图，避免再次重复机载证据。

## 输出与生成

- `main_figure.png`：3600 × 1250，508 dpi。
- `preview.jpg`：1800 × 625。
- `main_figure.pdf`：180 × 62.5 mm。
- `main_figure.svg`：自包含可编辑图件。
- `build.py`：使用准备好的 `assets/` 重新导出。

```bash
/usr/bin/python3 build.py
```

系统 Python 依赖 NumPy、Pillow、Cairo、PyGObject/Rsvg。

## 素材来源

三维场景、记录轨迹、相机外参和四张跟拍图与 V8 相同。a 区没有更换坐标、视角、图片或数据。机载图来自同一归档飞行的 RGB 视频：编码时间 35.5 秒和 46 秒；两张照片分别显示原期望位置与随后发现目标的位置，并非同一个房间或同一个固定机位。

图像仅作矩形裁剪、缩放与编码。跨视频仍为近似语义事件对应，不声称严格硬件同步。具体来源见 `assets/evidence_sources.json`。

本稿在 `source/` 中独立保存，未替换正文定稿。

## Suggested caption

Real-world multi-task execution with LogicFly. (a) The recorded onboard EKF trajectory and calibrated observation poses are shown over the measured occupancy geometry, with third-person photographs of the four inspections below. (b) Onboard images show the initial laptop location where the target was not found (A), followed by the laptop observation at the recovered location (1).
