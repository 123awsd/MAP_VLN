# 实时净空只读试验

当前仅实现 SHADOW 检测和性能试验，尚未接入自动制动/悬停。
不会发布任何 ROS 话题或调用服务，也不改变现有 full_smooth 启动链路。
22 cm 是测试阈值，尚未根据实际整机包络、跟踪误差和制动性能验证。

在 NX 定位和实时 world 坐标点云已启动、飞机停在地面时运行：

```bash
cd /home/nv/SL_WS/PRE_MAP_VLN_real_fly
source /opt/ros/noetic/setup.zsh
source /home/nv/dls_ws/devel/setup.zsh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 real_fly/stage2_runtime/scripts/live_clearance_shadow.py --duration 300
```

默认输入 /cloud_registered (PointCloud2) 和 /ekf_quat/ekf_odom。
必须确认前者为单帧实时点云，二者 frame_id 为 world，EKF twist 为 world 速度。
其他坐标系拒绝检查，不通过改 frame 名称冒充坐标转换。
无点云、非有限点、过期消息、过大消息均报告 INPUT_UNAVAILABLE。
每 0.1 s 取最新消息；不做密度过滤、体素降采样或自体点盲删。
NO_HIT_IN_RECEIVED_POINTS 只表示收到的点未命中，不能证明视野外空间安全。

几何检测：全向中心最小距离，以及按当前 world 速度外推的平滑制动路径包络。
该试验不检查完整未来 MINCO 曲线；最大减速度 1 m/s²、jerk 2 m/s³
只是计算假设，尚非实测制动能力。反应时间 0.30 s 加当前点云年龄。
门框单个点也保留，门洞中央与偏心位置有合成测试覆盖。

后续接管前必须实现并验证：任务锁止、轨迹时间冻结、连续制动指令、停止后保持、
数据失效与节点死亡处理、实际机体包络和时间同步、人工恢复后的重新连接轨迹。
不要直接将 WOULD_STOP 接到 kill 或解锁服务。

性能报告中单核 CPU=100% 表示占用一个完整核心；RSS 包含解释器与 numpy。
合成测试不包含 ROS 传输、雷达驱动及全机负载，不能用于声称飞行延迟合格。

## NX 初次测量（2026-09-20）

NX uav26，numpy 单线程，10 Hz，各 20 秒/200 次：

| 输入 | 单核 CPU | 计算 p95 | 最大耗时 | 进程峰值 RSS |
| --- | --- | --- | --- | --- |
| 20,000 点 | 3.11% | 3.29 ms | 3.85 ms | 30.90 MiB |
| 200,000 点 | 19.70% | 20.58 ms | 21.03 ms | 43.38 MiB |

本机和 NX 的 5 项单元测试通过。NX 真实订阅 3 秒期间没有点云，
输出 INPUT_UNAVAILABLE；没有完成真实定位延迟对照测试，没有启用控制。
测试范围只覆盖点云解码和制动包络几何计算，不能据此批准实飞。
