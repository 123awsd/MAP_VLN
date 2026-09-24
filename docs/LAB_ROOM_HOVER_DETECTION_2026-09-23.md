# 实验室 demo：仅悬停观察片段显示检测框

来源：`lab_room_v2/check_cup_basket_trashcan_07/flight_20260923_014428`。
不修改原视频、bag、任务、飞控或 NX。播放速度不变。

输出目录位于原场次下的 `hover_detection_v1/`，视频为 `rgb_detected.mp4`。
模型：本地缓存 Grounding DINO base；逐帧推理，阈值 0.30，同类 NMS 0.45。
同时提供背景类别以减少目标类别强行匹配；包含背景类别的混合描述不绘制。
只显示检测框、类别与模型分数，不添加任务文字、完成标记或手绘目标框。

## 显示片段

依据目标在画面中的位置、相邻画面运动及静止片段人工选取以下源视频时间窗：

- 42.90–44.40 秒：台面保温杯（thermos / bottle / cup 识别类别）。
- 62.90–64.25 秒：椅下篮球。
- 73.00–76.70 秒：风扇旁垃圾桶，最终观察停留片段。

时间窗以右开区间实现；窗外不运行检测、不画框，窗内只允许对应目标类别。
这些是画面核对得到的近似悬停片段，不是基于精确视频/bag同步获得的飞控状态边界。
旧视频缺少真实图像时间戳，不能声称二者精确同步。
框由该帧的模型预测产生，未识别时不补画或沿用其他帧的框。
因此可能有漏检、闪烁和类别不稳定；不代表在线识别或任务成功判定。

## 复现

在仓库根目录运行（输出目录必须不存在）：

```bash
SESSION_DIR=real_fly/stage2_runtime/runtime/flight_bags/lab_room_v2/check_cup_basket_trashcan_07/flight_20260923_014428
.envs/boxer/bin/python scripts/annotate_flight_rgb.py \
  "$SESSION_DIR/flight_20260923_014428_rgb.mp4" \
  "$SESSION_DIR/hover_detection_new" \
  --threshold 0.30 --windows "$SESSION_DIR/hover_detection_windows.json"
```

编码为 H.264，x264 quality=18，原分辨率、帧数与名义帧率不变。是一次重新编码，
不宣称无损。每帧检测记录保存在 `detections.jsonl`；`report.json` 只在输出视频
全帧解码数量核对成功后写入。各类别 best.jpg 为代表帧供人工检查。

`detection_preview_v*` 是抽样诊断；`detection_overlay_v1` 是用户进一步限定
“只在悬停时识别”后中止的全片尝试，不是交付视频。
