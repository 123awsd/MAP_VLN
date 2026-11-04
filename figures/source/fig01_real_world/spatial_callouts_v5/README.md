# 主图 V5：空间对应式白底构图

本版重新组织布局：四张真机照片围绕三维轨迹，细引线关联相应观察位置。中间为主空间视图，顶部中间为缩写指令，底部中间放单独的真机平台照片与图例。不标房间，不使用底部整行照片或照片与地图的渐变叠加。

## 输出

- `preview.jpg`：1800 × 1095 预览。
- `main_figure.png`：3600 × 2190，508 dpi。
- `main_figure.svg`：可编辑、自包含的嵌图文件。
- `main_figure.pdf`：180 × 109.5 mm，文字、轨迹、视锥和引线保留为矢量。
- `build.py`：使用本目录 `assets/` 可独立重新生成当前版面。

```bash
/usr/bin/python3 build.py
```

依赖系统 Python 的 NumPy、Pillow、Cairo、PyGObject/Rsvg。

## 数据与显示

主空间图来自 V4 已准备的实测 RGB-D 投影，使用同一三维投影和同一记录轨迹坐标。将点云颜色转换为浅灰显示材质，保留原图的每个像素透明度，不增删或挪动点云。该显示处理只用于视觉层次，不表示重新重建了场景。原实测 RGB-D 点云、录制轨迹和运行输出保留在论文项目之外。

记录的机体轨迹、阶段时刻以及外参标定的相机视锥都直接复用 `assets/scene_az-10.json`。相机视锥使用统一的 1.45 m 显示深度。轨迹为机载 EKF 估计，不是规划路线、外部真值或跟拍摄像机轨迹。

四张辅助照片统一使用第三人称真机跟拍帧：笔记本 53 秒、工具箱 69 秒、微波炉 129 秒、水杯 163 秒。只做矩形裁剪、缩放和编码；引线表示语义事件与记录观察位姿的对应关系，不声称严格同步或透视配准。照片与原始视频、裁剪范围的来源详见前一版 `trajectory_photo_draft/manifest.json`。

下方平台图来自原有 `hardware_assets/drone_cutout.png`。它是独立的硬件照片，未将这个平台图当作虚拟飞行实例加入场景或用于生成轨迹。

顶部指令为原始四任务指令的英文缩写；微波炉照片只表示检查动作，不据此声称加热状态已被验证。

本稿仍在 `source/`，未替换正文使用的主图。

## Suggested caption

Real-world multi-task execution with LogicFly. The recorded onboard EKF trajectory and calibrated observation poses are shown over a neutral rendering of the measured RGB-D scene. Blue indicates the executed flight and orange highlights recovery search from the initial laptop check (A) to the subsequent laptop observation (1). Third-person photographs show laptop, toolbox, microwave and cup inspections, linked to their observation positions. The UAV platform is shown separately below.
