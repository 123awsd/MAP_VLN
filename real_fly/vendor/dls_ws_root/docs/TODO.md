# TODO（项目级任务板）

> 宏观任务视图，记录项目方向与待办。功能级细节见 `docs/AI_PROCESS.md`。

## 已完成里程碑

- [x] 工作区扁平化重构为 `src/` 布局（`2635891`）
- [x] 删除竞赛视觉 perception 模块与相关文档（`760dfb2`、`47bb5a2`、`9c17bf6`）
- [x] Livox 雷达型号/IP 支持环境变量配置（`dc6d94c`）
- [x] `traj_server` 迁移到 `src/control/`，修复 `run_ctrl.launch` 默认参数（`a08149d`）
- [x] 八字轨迹支持 N 周期自动退出，默认 3 周期（`e43cd35`、`18f2c75`）
- [x] lite 构建模式（只编定位+控制+驱动，不碰规划）（`54567a2`）
- [x] 圆形/八字轨迹发布脚本（`da33848`）
- [x] `/etc/uav/uav.env` 环境加载 + `lidar.sh` 独立启动脚本（`913dab3`、`cecbdbf`）
- [x] NX 部署问题定位：ego_ws 与 dls_ws 混淆导致 env 不生效，用 `./lidar.sh` 解决
- [x] FAST_LIO 共享 PCD 全局定位启动链路（代码与 Noetic 容器编译完成，待真机多机验证）
- [x] GCS 统一下发定位模式、PCD 与每机初值，并校验/比较实际地图 SHA256
- [x] FAST_LIO global 模式全图自动初始位姿：SCDB 描述子检索 + top-K GICP + 结果落盘（容器验证 + NX 实测成功，v3 建库无需视场带）

## 待办

- [ ] 定位、控制器在真机（NX）上的完整链路验证（mavros + lidar + FAST_LIO + ekf + px4ctrl）
- [ ] 规划部分（mission_planner / super_planner / rog_map）按需编译与验证
- [ ] 多机编队配置（每机一份 `/etc/uav/uav.env`，IP/ID 区分）
- [ ] 两架以上无人机用同一 PCD/hash 完成静态对齐与相对位置真机验收（INIT-AUTO 单机已通过：8.5m 处自动定位，接受位姿与真值偏差 <2mm）
- [ ] GCS Prepare 增加 Agent commit 版本上报，阻止旧代码混入多机 READY
- [ ] GCS 在 READY/EXECUTING 持续监控 MAVROS、定位和控制器，为 Prepare 增加总超时和明确失败状态
- [ ] 建立 Agent 物理身份握手（DRONE_ID/UAV_NAME/IP/端口）和部署后版本审计，防止错机重标与版本漂移
- [ ] 清理部署机上 `/home/nv/ego_ws`（确认无依赖后归档/删除），避免再混淆
- [ ] 编写真机飞行测试 SOP（起降、安全开关、轨迹测试流程）
