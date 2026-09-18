# AGENTS.md

## 项目简介

多无人机 Drone Light Show 编队平台，基于 HKU SUPER 无人机导航栈改造，ROS1 Noetic + catkin。
开发机（本机）只有 ROS2，**所有 ROS1 编译/测试必须在 Docker 容器 `ros-noetic-gpu` 内执行**；
部署目标是 Jetson Orin NX（`/home/nv/dls_ws`）。

## 项目结构与模块

- `src/drivers/livox_ros_driver2`：Livox MID360/MID360S 驱动（含 SDK2，已加 env 配置支持）
- `src/localization/`：FAST_LIO（激光里程计）、ekf_quat_pose（状态估计）
- `src/control/`：px4ctrl（PX4 控制）、traj_server（圆/八字测试轨迹）、uav_utils
- `src/mission_planner` / `src/super_planner` / `src/rog_map`：规划部分
- `scripts/`：辅助脚本（`load_uav_env.sh` 读取 `/etc/uav/uav.env` 等）
- 根目录脚本：`lidar.sh` / `localization.sh` / `fly.sh` / `ctrl.sh` / `plan.sh` / `circle.sh` / `eight.sh` / `takeoff.sh` / `land.sh` / `build.sh`
- `build/`、`devel/`：编译产物，**不要直接编辑**

## 构建、测试与开发命令

```bash
# 容器内编译（本机必须这样做）
docker exec ros-noetic-gpu bash -lc 'cd /root/experiment/dls_ws && ./build.sh lite'
./build.sh              # 全量
./build.sh lite         # 只编译定位+控制+驱动相关包
./build.sh --pkg <pkg>  # 单包
BUILD_JOBS=4 ./build.sh # NX 8GB 内存建议
```

容器内工作区路径为 `/root/experiment/dls_ws`（挂载自 `/home/fast/experiment/dls_ws`）。
最小验证：容器内跑 `./build.sh lite`，然后 `bash -n` 检查脚本、`roslaunch` 冒烟测试。

## 常用运行脚本（在部署机 NX 上）

- `./lidar.sh`：单独启动雷达驱动，**强制使用 dls_ws 的包 + 读取 `/etc/uav/uav.env`**（推荐入口）
- `./localization.sh mapping|global ...`：定位链路；`mapping` 为启动点原点建图，`global MAP.pcd [x y z yaw]` 为共享 PCD 全局定位
- `./fly.sh mapping|global ...`：完整链路；无参数兼容原来的 `mapping`，场地 PCD 优先放在 dls_ws 根目录
- `./ctrl.sh`：仅启动 px4ctrl（`run_ctrl.launch`，默认 `ctrl_param_fpv.yaml`）
- `./circle.sh` / `./eight.sh`：发布圆形/八字测试轨迹到 `/planning/pos_cmd`（八字默认 3 周期自动退出）
- `./plan.sh [full_smooth|super|hybrid]`：启动规划
- `./git-sync.sh`：局域网内多机 git 代码同步（纯 SSH，无外网；主机清单见 `scripts/drones.conf`）

## 环境变量与部署

- 无人机配置统一放在 `/etc/uav/uav.env`（纯 bash export，非 zshrc），包含
  `DRONE_ID`、`UAV_NAME`、`LIVOX_LIDAR_TYPE`（mid360/mid360s）、`LIVOX_LIDAR_IP`（逗号分隔多 IP）、`LIVOX_HOST_IP` 等
- 手动运行优先使用 `./fly.sh global <场地>.pcd [x y z yaw_deg]`，不需要把地图和初值写入环境文件；
  GCS/systemd 无命令行模式仍兼容 `GLOBAL_MAP_PCD`、`GLOBAL_MAP_SHA256`、`INIT_BODY_*`
- `scripts/load_uav_env.sh` 逐行解析该文件（已有环境变量优先、支持注释/引号），启动脚本先 source 它
- **不要 source 整个 `.zshrc`**（zsh 语法会让 bash 报错）
- 雷达 IP 由 `LIVOX_LIDAR_IP` 覆盖 JSON 配置；`lidar.launch` 中节点为 `required=true`，驱动退出会导致 roslaunch 整体退出
- **重要**：NX 上存在 `/home/nv/ego_ws`（旧工作区）和 `/home/nv/dls_ws`（本仓库）两套代码，
  直接 `roslaunch` 可能命中旧工作区（env 不生效）。一律用 `./lidar.sh` 等仓库脚本启动，或先 `source /home/nv/dls_ws/devel/setup.bash`

## 处理问题原则

- 先做根因分析再改代码：先判断是架构/接口/数据流/参数问题，不要一上来调参数
- 连续 3 次修复验证失败时，回滚并重新分析，不要继续堆补丁
- 排查部署问题优先复现（如 SSH 到 NX 实测），不靠猜测下结论
- 一次只解决一个问题，改完在容器/NX 上验证再提交

## 不要做的事

- 不要在本机（无 ROS1）直接编译 ROS 包
- 不要编辑 `build/`、`devel/`、`__pycache__` 等生成目录
- 不要 source 整个 `~/.zshrc`；不要擅自修改用户的 `~/.zshrc`（只允许改本仓库内文件）
- 不要用 `rm -rf`（环境安全策略禁止）；用 `git rm` 或 Python `shutil.rmtree`
- 不要 `pkill -f "roslaunch"` 这种宽匹配（会误杀外层命令），用精确匹配或 PID
- 部署机上不要把 `/home/nv/ego_ws` 和 `/home/nv/dls_ws` 混用

## 项目记忆文档

- `docs/TODO.md`：项目级宏观任务板
- `docs/AI_PROCESS.md`：功能级进度日志（先读索引，再读对应条目）
- 新任务前阅读顺序：`AGENTS.md` → `docs/TODO.md` → `docs/AI_PROCESS.md` 相关条目
- 完成一个有意义的进展后更新 `docs/AI_PROCESS.md`；项目级里程碑变化时更新 `docs/TODO.md`
