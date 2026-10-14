# 探索验收视频生成说明

基线：实际执行规划yaw、跨层代价18、执行受阻目标暂避20秒。有效片段为本次录包开始至503.93秒，轨迹437.32米、已知体积1129.30m³。输出统一40秒、30fps、1200帧，不包含后段飞出过程。不是缩短实际轨迹或篡改观测。

## 输入及保留位置

`outputs/exploration_yaw_consistent_trial/trial.bag`为原始录包；`video_cache`含800帧RGB、increments.npz、timeline.npz、camera_pose.npy；`original_style_video/geometry.npz`含原始观测点及法线。第一视角用实际RGB，不离线重渲染。输入及缓存均保留，视频不纳入Git。

## 从录包生成

在ROS容器中依次运行该试验目录的export_video_cache.py、export_camera_commands.py、restore_original_cloud_sampling.py。路径约定/workspace/shared/outputs映射仓库outputs。这些脚本也保存于本检查点。采样为seed42、5%深度像素、间隔至少0.25秒、深度0–5米；z<8.05只是视频筛选。

用.envs/habitat/bin/python运行original_style_video/prepare_geometry.py，随后运行：

```bash
.envs/habitat/bin/python scripts/render_exploration_three_view.py --trial-dir outputs/exploration_yaw_consistent_trial --output outputs/exploration_yaw_consistent_trial/original_style_video/full40s.mp4
.envs/habitat/bin/python scripts/export_accepted_exploration_tracks.py outputs/exploration_yaw_consistent_trial/original_style_video
```

配置生成配置.json须设置yaw_matched_camera=false；原版布局左大俯视、右上全局、右下RGB、无字、蓝青绿紫渐变和扇形视野。单轨脚本取原40秒视频前840帧，按floor(i*840/1200)映射为1200帧，三个轨道时间一致。合并视频使用相同映射；最终保留exploration_accepted_baseline_40s.mp4及accepted_top_40s.mp4、accepted_global_40s.mp4、accepted_rgb_40s.mp4。

原始完整数据保留。删除的是旧探索视频，MINCO、阶段二、房间和实飞视频不删除。编码依赖GStreamer x264enc、OpenCV、NumPy；点云法线依赖SciPy。全帧解码验证1200帧/30fps；不要用视频裁剪后的范围冒充完整720秒统计。
