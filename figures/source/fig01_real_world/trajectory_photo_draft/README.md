# 真机主图草稿：实测轨迹、跟拍照片与条件搜索

这是独立的主图构图草稿，未替换 `figures/final/real_world_execution.png`，未修改正文。

当前修改版位于 `no_room_labels/`：根据作者反馈移除了地图中的房间名称、文字底框和房间分区配色，同时去掉照片下方的房间名称，缩短图例与分图标题。仅保留实测墙体、记录轨迹和任务编号。原第一版导出文件保留在本目录供对照，`build.py` 现在默认生成修改版。

## 查看与编辑

- `preview.jpg`：1800 × 1160 的快速预览。
- `main_figure.png`：3600 × 2320 的高清图，物理宽度 180 mm，约 508 dpi。
- `main_figure.pdf`：180 × 116 mm；文字、轨迹、标记及房间边界保留为矢量，照片和实测墙体底图为嵌入栅格。
- `main_figure.svg`：自包含的可编辑文件，照片已嵌入，可用 Inkscape 或 Illustrator 调整。
- `build.py`：当前修改版的版面源文件；PNG 和 PDF 从同一 SVG 导出到 `no_room_labels/`，保证版面一致。
- `manifest.json`：素材出处、视频时刻、裁剪范围与数据解释。

已准备好的 `assets/` 足够重新生成版面，不依赖原始视频或系统代码仓库：

```bash
/usr/bin/python3 build.py
```

依赖本机系统 Python 中的 NumPy、Pillow、PyGObject/Rsvg 与 Cairo。`prepare_assets.py` 仅用于从原素材重新提取资产，需要原工作区和系统 OpenCV。

## 构图

上方保留任务指令的英文缩写。左侧以实测结构地图承载完整任务执行段的飞行轨迹，右侧为四个任务的跟拍照片。底部用三张较大的画面呈现检查原位置、跨房间搜索和机载视角发现笔记本的过程。

蓝色轨迹表示飞行记录中的机体位置估计；橙色为笔记本恢复搜索段。A 为最初检查笔记本的位置，1–4 为随后执行的四个目标检查位置。编号按照实际执行顺序排列。下方箭头表达事件顺序，不表示照片来自固定机位。

## 数据依据

- 场次：`flight_20261001_184127`，实验室四任务加主动搜索。
- 地图：归档 LiDAR 结构栅格中的实测占用墙体，以及已批准房间多边形。仅做视野裁剪与配色，没有重建或改写建筑几何。
- 轨迹：`/ekf_quat/ekf_odom` 记录的机体位置，直接使用其世界坐标，与结构地图同坐标系；按每 8 个样本取 1 个用于绘图，附上最后样本，未平滑或手工重画。
- A、1–4：根据飞行日志阶段到达时刻，读取最近的里程计位置；恢复搜索段使用阶段 2 到阶段 4 的日志锚点。
- 轨迹是机载 EKF 估计，不是外部动捕真值，也不是规划的 MINCO 路径或跟拍摄像机轨迹。
- 照片：原始真机跟拍视频与机载 RGB 视频的截图。仅做矩形裁剪、缩放和编码，无生成式修改。
- 两种视频时刻属于各自的编码时间轴，通过已观察到的事件近似关联，不声称严格同步。
- 微波炉画面表述为检查，不据此声称食物加热状态已被视觉验证。

## Suggested caption

Real-world multi-room execution with LogicFly. (a) The recorded onboard EKF flight trajectory is overlaid on a measured LiDAR-derived structural map. The orange segment highlights recovery search after the laptop is absent from its expected location (A). (b) Third-person photographs show the UAV inspecting the laptop, toolbox, microwave, and cup; numbered markers associate these observations with their recorded locations. (c) A sequence of third-person and onboard observations illustrates the conditional search, from checking the lounge table to locating the laptop in the reading room.
