# 实验室成功 demo

用户已于 2026-09-23 明确确认本次飞行成功，将本场次标记为“实验室成功的 demo”。此标记记录用户现场结论，不代表另行完成了 bag 飞行性能分析。

- RUN_ID：`lab_room_v2`
- TASK_ID：`check_cup_basket_trashcan_07`
- 场次：`flight_20260923_014428`
- session.yaml 记录开始时间：`2026-09-23T01:44:53+08:00`
- 播放器配置上限：速度 `0.8 m/s`，角速度 `0.60 rad/s`；不是实测峰值。
- 任务：去休息室检查台面上的水杯，再去活动室检查椅子下面的篮球，最后去杂物间检查风扇旁边的垃圾桶。

## 本地数据

已从 NX 拷回完整场次目录（包含 context、日志、session.yaml、bag 和 RGB 视频），远端原件保留。

目录：
`real_fly/stage2_runtime/runtime/flight_bags/lab_room_v2/check_cup_basket_trashcan_07/flight_20260923_014428/`

- Bag：`flight_20260923_014428_0.bag`，23,790,146 bytes。
- 第一视角视频：`flight_20260923_014428_rgb.mp4`，145,486,444 bytes。
- session 声明 RGB 配置为 H.264、1280×720、30 Hz、12000 kbps；点云录制关闭。视频是独立 MP4，不应根据 bag 大小判断视频是否完整。

## 拷贝完整性

以下本地 SHA-256 均与 NX 一致：

```text
51e3a24881f4e1fd8d09ec8d4533febd5ca0e993e516c00a0bf93de567cbfff3  flight_20260923_014428_0.bag
52dd1d7b71c4d26750aa24b65262a5c3dcaec8ce93ecfcc2115f4c04dd095eb1  flight_20260923_014428_rgb.mp4
088953ea8c8493c6abb547dc9469ffb37ebebb2de3be216824057aaaa47b8a3b  session.yaml
```

轨迹修复与离线验证见 [LAB_ROOM_DEPARTURE_FIX_2026-09-23.md](LAB_ROOM_DEPARTURE_FIX_2026-09-23.md)。未修改已封存轨迹或提交 Git。
