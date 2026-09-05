# RTX 5060 离线处理

`Drone_room` 的完整离线流水线由主机执行，原始 bag 只读，输出均在被忽略的
`real_fly/stage2_offline/data/` 下。流程不会启动 MAVROS、飞控、PX4Ctrl 或任何
运动节点。

```bash
cd /home/uav/map_VLN/PRE_MAP_VLN
./real_fly/stage2_offline/scripts/run_drone_room_5060_complete.sh
```

流程会在 Noetic Docker 中重跑完整 FAST-LIO（必要时自动降低 rosbag 回放速率），
导出 RGB-D 关键帧，在 RTX 5060 上用 CUDA 完整运行 Boxer，进行深度反投影、目标
聚类、voxel 快照、scene/task、离线规划和质量审计；已有完整结果会安全复用，Boxer
支持 `--resume`。

主要输出：

- `data/Drone_room/fastlio_complete/`：完整位姿、PCD、轨迹和审计
- `data/Drone_room/boxer_final_5060_complete/`：检测 CSV、处理清单和类别汇总
- `data/Drone_room/localization/`：原始/聚类三维目标
- `data/Drone_room/stage2_complete/`：scene、task、规划结果和可视化
- `data/Drone_room/reports/`：验证与质量审计 JSON

当前数据的相机—机体外参仍是临时估计值；它足够验证离线链路，但真机执行前必须
用实测刚体外参替换，不能直接用于有安全要求的飞行。
