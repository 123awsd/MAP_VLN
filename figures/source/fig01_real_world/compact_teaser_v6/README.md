# 紧凑主图 V6

根据“太丑陋、不紧凑”的反馈，改为紧凑的两层结构：上层为完整三维飞行记录，下层为四张窄幅真机观测图。取消四角布局、跨画面的长引线与底部硬件照片。

## 输出

- `main_figure.png`：3600 × 1381，508 dpi；相同宽度下高度比 V5 减少约 36.9%。
- `preview.jpg`：1800 像素宽的快速预览。
- `main_figure.pdf`：180 × 69.05 mm，轨迹、视锥、标记和文字为矢量。
- `main_figure.svg`：自包含的可编辑图件。
- `build.py`：使用本目录中准备好的资产重新生成版面。
- `prepare_scene.py`：从原归档 RGB-D 点云和 EKF 记录重新投影，需要原工作区数据。

```bash
/usr/bin/python3 build.py
```

依赖系统 Python 的 NumPy、Pillow、Cairo 和 PyGObject/Rsvg。

## 紧凑化与真实性

视角由上一版的俯视角 50°改为 22°，方位角为 0°。这是从同一实测三维点云重新计算的正交投影，没有在二维图像中压扁建筑、缩短路线或改变坐标。根据实际非透明像素范围裁掉外部空边，并以同一个比例缩放地图、路线和视锥。具体裁剪范围及版面尺寸记录在 `layout_audit.json`。

几何来源仍为 `画图/主图/local_vggt/mapping_rgbd/mapping_rgbd.ply`；路线为记录的 `/ekf_quat/ekf_odom` 机体位姿；视锥使用归档相机外参，显示深度统一为 1.45 m。几何只做了原有范围裁剪和浅灰显示着色，不补造点云或移动实际位置。没有绘制房间名称或房间分区。

四张第三人称真机照片分别取自跟拍视频 53、69、129、163 秒的既有帧。照片采用一致的窄幅比例，工具箱画面优先保留上部货架与无人机；其余画面以居中矩形裁剪保留无人机及目标。照片与阶段位姿是语义事件近似对应，不声称硬件严格同步。

原始点云、视频和运行记录保留在论文项目外。当前工作稿未替换 `figures/final/` 或修改正文。

## Suggested caption

Real-world multi-task flight with LogicFly. The upper view shows the recorded onboard EKF trajectory and calibrated observation poses in the measured RGB-D scene. Blue denotes executed flight and orange highlights recovery search from the initial laptop check (A) to the laptop observation (1). The lower photographs show laptop, toolbox, microwave and cup inspections.
