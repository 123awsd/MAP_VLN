# AI_PROCESS（功能级进度日志）

> 阅读顺序：先看「索引」，再读对应条目。功能完成后更新对应条目。

## 索引

| ID | 功能/工作流 | 状态 | 最后更新 |
|----|-------------|------|----------|
| [LIVOX-ENV](#livox-env-雷达环境变量配置) | Livox 雷达型号/IP 环境变量配置 | ✅ 完成（NX 实测可用） | 2026-08-06 |
| [TRAJ-SERVER](#traj-server-控制器测试轨迹) | traj_server 迁移与圆/八字轨迹 | ✅ 完成（容器实测） | 2026-08-06 |
| [BUILD-LITE](#build-lite-lite-构建模式) | 只编定位+控制+驱动的 lite 构建 | ✅ 完成 | 2026-08-06 |
| [UAV-ENV](#uav-env-环境加载与启动脚本) | /etc/uav/uav.env 加载 + lidar.sh | ✅ 完成 | 2026-08-06 |
| [NX-DEPLOY](#nx-deploy-部署排障) | NX 部署排障（ego_ws/dls_ws 混淆） | ✅ 完成 | 2026-08-06 |
| [GIT-SYNC](#git-sync-局域网多机代码同步) | 局域网多机 git 代码同步脚本 | ✅ 完成（待真机联调） | 2026-08-06 |
| [GCS-UI](#gcs-ui-地面站机队控制台) | GCS 机队页与底部任务控制台优化 | ✅ 完成 | 2026-08-08 |
| [GLOBAL-MAP](#global-map-fast_lio-共享-pcd-全局定位) | 多机共享 PCD 坐标系定位 | 🟡 单机 NX 对齐成功，待多机验证 | 2026-08-10 |
| [GCS-SAFETY](#gcs-safety-真机控制链路收紧) | GCS 真机契约、鉴权与运行时安全 | 🟡 本机验证完成，待真机验证 | 2026-08-10 |
| [VALIDATION-GATE](#validation-gate-无硬件端到端验证) | DLS/GCS 构建、启动、失败注入与跨仓契约 | 🔴 发现多机阻断项，待修复 | 2026-08-10 |
| [GCS-LOCALIZATION](#gcs-localization-地面站统一定位配置) | GCS 下发定位模式、地图与每机初值 | 🟡 无硬件验证完成，待多机真机 | 2026-08-10 |
| [INIT-AUTO](#init-auto-全图自动初始化) | global 模式全图自动初始位姿（SCDB 检索 + top-K GICP） | ✅ NX 实测成功 | 2026-08-16 |

---

## LIVOX-ENV 雷达环境变量配置

**目标**：每台无人机通过环境变量配置雷达型号（mid360/mid360s）与 IP，不改代码/配置文件。

**改动**（`dc6d94c`）：
- `lidar.launch`：`$(eval optenv(...))` 按 `LIVOX_LIDAR_TYPE` 选 `MID360_config.json` / `MID360s_config.json`
- SDK 解析器（`parse_cfg_file.cpp`）+ driver 解析器（`parse_livox_lidar_cfg.cpp`）：`LIVOX_LIDAR_IP` 逗号分隔多 IP 覆盖，`LIVOX_HOST_IP` 覆盖 host NIC

**验证**：容器内用独立测试程序编译 driver 解析器，确认 `LIVOX_LIDAR_IP=192.168.1.143` 时 handle 变为 .143；NX 上实测 JSON=非雷达 IP + env=雷达 IP 仍能连接（IMU 200Hz / lidar 20Hz）。

**教训**：`strings | grep -c` 对相同字符串字面量会合并（string pooling），不能证明两个文件都编进二进制。

## TRAJ-SERVER 控制器测试轨迹

**目标**：迁移 ref 下的 traj_server 到 `src/control/`，用于测试 PX4Ctrl。

**改动**（`a08149d`、`e43cd35`、`18f2c75`、`da33848`）：
- `src/control/traj_server/`：circle + eight 测试节点、分析脚本
- 八字默认 `num_periods=3` + `shutdown_after_finish=true` 自动退出
- 根目录 `circle.sh` / `eight.sh` 发布到 `/planning/pos_cmd`
- `run_ctrl.launch` 默认参数修复为 `ctrl_param_fpv.yaml`（competition 参数已删）

**验证**：容器内实测 circle 自动退出、eight 3 周期自动退出。

## BUILD-LITE lite 构建模式

**目标**：编译提速——定位/控制/驱动基本不变，只编一次即可。

**改动**（`54567a2`、`9b83ff5`）：`build.sh lite` 用 `CATKIN_WHITELIST_PACKAGES` 只编
`quadrotor_msgs livox_ros_driver2 fast_lio ekf_quat uav_utils px4ctrl traj_server`；`--pkg` 单包；`BUILD_JOBS` 适配 NX。

## UAV-ENV 环境加载与启动脚本

**目标**：bash/sh 启动脚本能读取部署机上的无人机环境变量（在 `/etc/uav/uav.env`，非 zshrc）。

**改动**（`913dab3`、`cecbdbf`）：
- `scripts/load_uav_env.sh`：逐行解析 `/etc/uav/uav.env`（已有 env 优先、去注释/引号、文件缺失不报错）
- `fly.sh` source 它；新增 `lidar.sh` 独立启动雷达（强制 dls_ws 包 + 加载 env）

**注意**：`set -u` 与 ROS setup.bash 不兼容（`ROS_DISTRO` 未定义会报错），脚本先 source 再开严格模式。

## NX-DEPLOY 部署排障

**现象**：NX 上 `roslaunch fast_lio lidar.launch` 无雷达数据；改 JSON 里的 IP 才能连，env 不生效。

**根因**：NX 上有 `/home/nv/ego_ws`（旧）与 `/home/nv/dls_ws`（新）两个工作区；
roslaunch 的 cwd/ROS_PACKAGE_PATH 指向 ego_ws → 跑的是旧驱动（无 env 支持）→ env 传进去也没用。

**解决**：用仓库内 `./lidar.sh`（强制 dls_ws + 加载 env）；不要直接 `roslaunch`，不要依赖用户 shell 配置。
**禁止**：修改用户的 `~/.zshrc`（用户明确要求只改 dls_ws 项目）。


---

## GIT-SYNC 局域网多机代码同步

**目标**：多台无人机在同一局域网、无外网环境下，通过 git 直接同步 dls_ws 代码（已知每机 IP/用户名）。

**方法选择**：
- 主推**直推模式**：本机给每架飞机配 git remote（`nv@10.1.1.10x:/home/nv/dls_ws`），飞机端
  `receive.denyCurrentBranch=updateInstead`，一条 `git push` 同时更新所有飞机工作区——最方便。
- 可选**中央裸仓库模式（hub）**：任选一机建 `~/dls_ws.git`，各机以它为 origin 拉/推，支持双向与飞机间互相同步。
- 全部走局域网 SSH，零外网依赖。

**IP 规则**：1 号机 = `10.1.1.101`，11 号机 = `10.1.1.111`，即 `10.1.1.100 + 机号`。

**改动**：
- `git-sync.sh`：`status` / `setup` / `push`（直推，默认分支 main）/ `hub init|push|pull|status`
- `scripts/drones.conf.example`：1..12 号机清单模板（用户默认 `nv`，路径 `/home/nv/dls_ws`）
- `scripts/drones.conf`：实际本地配置（已加入 .gitignore，避免各机配置被同步覆盖）

**验证**：`bash -n` 通过；`help` 输出正常；用假配置（含回环/不可达 IP）实测 `status` 统计与
`push --only/--skip`：离线机标 `[offline]` 跳过、汇总区分 成功/离线/跳过/失败，全程不中断。
待真机联调：`setup` 配 SSH 免密 + 各机 `updateInstead`，再 `push`/`hub pull` 实测。

**注意**：
- 直推要求飞机当前分支与推送分支一致，否则工作区不会被更新（脚本会跳过并提示）。
- push 到非 bare 仓库时，飞机端有未提交改动会被 git 拒推（属保护行为）。
- **部分飞机离线**：`status` 有在线/离线统计；`push` 离线机自动标 `[offline]` 跳过，不中断其他机；
  支持 `--only <name...>` / `--skip <name...>` 定点操作；离线机恢复后重跑即可增量补齐。
  全离线时 status/push 逐个 SSH 探测会较慢（默认 ConnectTimeout=5s），可用 `SSH_OPTS` 调短。
- **本机识别**：`push`/`setup` 用 `hostname -I` + `ip addr` 收集本机网卡 IP，与清单 IP 比对，命中即跳过自己
  （如 uav1 上跑时 drone01 被跳过），不把代码推给自己；`status`/`hub pull` 不过滤本机
  （hub pull 时本机也需要从中央仓库拉取）。注意过滤了 127.0.0.1/::1 回环地址避免误判。

---

## GCS-UI 地面站机队控制台

**目标**：提升 GCS 机队页的操作可读性，重点放大底部控制栏，确保真机批量操作时选择范围、程序栈和动作按钮清晰可辨。

**改动**（GCS 独立仓库）：
- 顶部导航由 54px 放大到 80px，放大左侧品牌文字、链路状态和 FASTLAB Logo；Logo 固定在顶栏正中，机队/日志/配置改为 44px 高的分段式主导航。
- 机队页控制范围、程序栈、飞机状态和动作按钮统一只保留中文，技术缩写（CPU、RAM、BAT 等）继续保留。
- 程序栈标签与下拉选择器改为桌面端同一行，选择器宽度和高度与无人机选择器统一；移动端自动纵向排列。
- 底部栏改为大尺寸任务控制台，桌面端最小高度 172px，动作按钮高度 66px；降落与停止栈拆为两个独立动作，飞行中禁用停止栈，要求先明确降落。
- 控制范围、已选数量、程序栈和四类动作建立明确的信息层级。
- 准备、执行、降落、停止栈按钮使用独立语义色与图标，移动端使用稳定的两列布局。
- 机队页默认程序栈从 `simulation` 改为真机已验证的 `hover`，降低误选风险。
- 地图/遥测列表增加未配置与离线空状态，在线徽标增加部分在线和离线语义。
- 降落仅在执行任务状态可用；停止栈在飞行中禁用，避免两个动作语义混用；移动端控制栏取消覆盖式 sticky 定位。
- 3D 视图增加 WebGL 初始化失败的局部降级，避免渲染错误阻断机队数据加载。
- 3D 无人机模型改为引用 DLS `yunque-M.dae` 离线生成的 `GCS/frontend/public/models/yunque-M-lite.glb`；模型约 6,400 三角面、776KB，加载一次后由各无人机实例共享，加载失败保留低面数几何降级。
- 移除无人机旁的状态光环；修正 DAE Z-up 到场景 Y-up 的四元数姿态叠加。仿真当前 `roll/pitch` 为 0，之前的倒立主要来自前端欧拉角与模型坐标转换混用，而不是仿真输出翻滚。
- 基于模型包围盒计算接地点偏移，保证遥测 `z=0` 时机体贴地而不穿过网格。
- 按真机约 250 mm 轴距校准 3D 模型比例：GLB 保持原生米制尺寸（含桨约 `0.414 × 0.453 m`），移除额外 `6 × 1.35` 放大；备用模型和选中环同步缩小，选中状态只通过颜色与圆环表达，不再改变机体物理比例。
- 3D 视图网格由 `2 m/格、每边 50 格` 调整为默认 `1 m/格、每边 100 格`；新增每边格数与单格尺寸设置，修改时仅重建网格辅助对象，并持久化到浏览器本地。
- 真机模式 `drones.real.yaml` 清单由 12 架扩展至 30 架（drone01–drone30）；后端继续按 ID 派生各机 Agent 地址和端口。
- 日志页改为飞行事件时间线：支持按飞机、级别、事件分类和关键事件筛选，显示告警/飞行统计并每 5 秒刷新；Pinia 接收 Agent WebSocket 日志，飞机详情页复用后端历史日志。
- 后端命令日志按 Agent 实际返回记录成功/拒绝/不可达；补充 Agent 上线、断开、心跳超时/恢复及模块失败/恢复事件，避免把未执行成功的请求误记为成功。

**验证**：`npm run build` 通过；后端 Python 文件通过 `py_compile`；运行时/程序栈测试 9 项通过；直连 `/api/drones` 返回 6 架在线仿真飞机。

---

## GLOBAL-MAP FAST_LIO 共享 PCD 全局定位

**根因**：原有代码已有 `initial_align` 和从 PCD 初始化 FAST_LIO 的能力，但 `fly.sh` 与 GCS 的 LIO 模块始终启动
`mapping_mid360.launch`，所以每次都以雷达启动位置为局部原点；地图路径、每机初值也没有接入部署环境。

**改动**：
- 新增 `global_localization_mid360.launch`，同时启动初始 GICP 对齐和等待 `/initial_odom_for_lio` 的 FAST_LIO；
  默认 `map_incremental=false`、`save_new_map_to_pcd=false`，作为纯定位运行。
- `initial_align` 和 FAST_LIO 支持同一个绝对 PCD 路径；地图裁剪中心改为每机 `Init_Body_pos`，支持初始 yaw，
  并对 GICP convergence、fitness、有限值和空地图做失败门控。
- 新增 `localization.sh`，支持 `mapping` 与 `global MAP.pcd [x y z yaw]` 命令行；地图优先从工作区根目录读取，
  自动计算 SHA256，并等待真实 `/Odometry`
  后才启动 EKF；`fly.sh` 和 GCS `lio_start.sh` 统一复用该入口。
- 修复 Ubuntu 20.04/Bash 5.0 下 `fly.sh` 的子进程等待回归：MAVLink 消息频率命令改为前台一次性执行，
  并避免用 `wait -n` 监控 MAVROS/定位两个常驻进程，防止短命 `mavcmd` 误触发整栈清理；每条 `mavcmd` 设置超时，不因 FCU 未回应而卡住定位。

**直接使用**（场地 PCD 放在 `/home/nv/dls_ws/`）：
```bash
./fly.sh mapping
./fly.sh global 场地A.pcd
./fly.sh global 场地A.pcd 5 -2 0 90
```
所有飞机使用同名、同 hash 的 PCD；最后四项是该飞机在地图 `world` 坐标系下的粗略起始位姿，可以不同。
无参数 `./fly.sh` 保持原始 mapping 行为。GCS/systemd 无 CLI 启动仍兼容原来的环境变量接口。
启动时飞机应静止，待 `/Odometry` 出现且位置与实地已知点一致后再进入控制器 READY。

**验证状态**：脚本语法、launch XML 和 diff 静态检查通过；`ros-noetic-gpu` 内 `./build.sh --pkg fast_lio`
编译通过，`roslaunch --nodes` 正确解析出 `initial_align`、`laserMapping`、`cpu_monitor`。NX 单机已从
`/home/nv/dls_ws/new.pcd` 完成 GICP 对齐，发布初始里程计后 FAST_LIO 进入 LIO；仍需在至少两架 NX 上
核对同一基准点、相对距离和航向，并在 RViz 确认 ICP 点云与 target cloud 重合。

---

## GCS-SAFETY 真机控制链路收紧

**改动**（GCS 独立仓库，禁止与 dls_ws 混合提交）：
- ROS 模式只公开/允许 `hover` 栈；控制器 Prepare 只确认进程存活，不能用
  `/debugPx4ctrl` 作为启动门槛（RC/PX4 未就绪时它可暂不发布）。命令仍必须等该话题确认进入
  `AUTO_TAKEOFF`/`AUTO_LAND` 后才推进运行时状态，失败保留原状态；Land 支持幂等重发。
- Agent HTTP/WebSocket 支持可选 Bearer token，地面站代理在配置时统一携带，加载环境时不打印秘密值。
  默认可信局域网模式不需要 token，`bash start.sh real` 可直接启动；如配置，GCS 与所有 Agent 必须使用同一值。
- Agent 重启后检测到存量 controller 时拒绝重复 Prepare，同时保留对已解锁存量栈的 Land 路径；落地判定要求新鲜 odom 和 FCU state。
- 修复 Agent 非 2xx 传播、遥测扩展字段、每机心跳 ID、进程 stdout 管道阻塞和 `land.sh` 错误命令值。
- 修复从 GCS 仓库外执行 `bash GCS/start.sh` 时 Backend 导入失败：启动器固定工作目录，逐项等待 Agent、Backend API 与 Frontend 健康后才显示 Ready；启动失败或任一常驻服务退出时返回非零并按独立进程组清理全部子进程。
- GCS controller 改为无参数复用 DLS `ctrl.sh`，与手工成功启动路径一致并固定使用 dls_ws；不设置 `no_RC`，继续保留实体 RC 启动门和人工接管能力。

**验证状态**：GCS 完整单元测试、shell 测试、Python 静态检查及前端构建在开发机验证；真机仍需验证命令回执、
Agent 重启后落地与网络中断场景。

---

## VALIDATION-GATE 无硬件端到端验证

**已证明通过**：
- Ubuntu 20.04 / Bash 5.0.17 容器复现 `wait -n` 会被非指定短任务触发；当前 `fly.sh`
  未再使用该机制。
- `./build.sh lite` 通过；MAVROS、LiDAR、FAST_LIO mapping/global、EKF、px4ctrl launch 全部可解析。
- 新增无硬件进程仿真，覆盖 mapping/global、MAVROS 早退、service 超时、`mavcmd` 失败、
  LIO 早退、odom 超时、地图 hash 错误和子进程清理，当前全部通过。
- GCS 68 个 Python 测试、Agent shell/Python 3.8 检查、Noetic smoke、sim/real 启停和前端构建通过。
- 首次执行 catkin tests 发现 `uav_utils-test` 错误链接不存在的 `libuav_utils`；本地修正后
  7 个 gtest 全部通过。

**已修复阻断项**：
- GCS 现作为定位配置权威，在 Prepare 时显式下发 mode/map/每机初值；Agent 启动前检查本机 PCD、计算 hash，
  GCS 对选中机队比较实际 hash，任一失败或旧 Agent 未回报验证结果时整批回滚，不再依赖逐机环境变量。

**仍待处理**：
- GCS 只在 PREPARING 做一次健康检查；READY/EXECUTING 后 MAVROS、定位或控制器退出不会及时改状态。
- Prepare 没有模块级总超时，话题不达标时可永久 PREPARING；全局坐标落地判定错用绝对 `z < 2`。
- GCS MAVROS 启动器与 DLS `fly.sh` 的 service-ready/命令超时逻辑分叉；DLS `localization.sh`
  在 Lidar/LIO 已退出时仍可盲等 odom 至超时。
- Agent/Backend 没有物理机身份握手或 commit 版本上报，配错 IP/端口可静默将物理 A 机当成 B 机，
  且无法发现多机运行不同内存版本。

**边界**：当前环境没有 PX4 SITL 与可回放的 LiDAR/IMU bag；上述验证能证明软件启动和契约，
不能代替 Livox 网络、PX4 串口、ICP 场景唯一性与真实飞行动力学验收。

---

## GCS-LOCALIZATION 地面站统一定位配置

**目标**：换场地时只在 GCS 配置一次定位方式和 PCD；每架飞机只保留不同的粗略初值，不再编辑
`/etc/uav/uav.env` 中的 FAST_LIO 参数。

**实现**：
- GCS 使用独立 `backend/config/localization.yaml` 保存公共 mode/map/hash 和每机 `x/y/z/yaw_deg`，配置页统一编辑；
- Backend 在每个 ROS `READY` 请求中自动合成该机配置，Agent 缺少显式定位配置时拒绝准备；
- Agent 仅在 IDLE 接收配置，限定地图为 dls_ws 根目录文件，启动前读取并计算 SHA256，再通过 localization 子进程私有环境调用 DLS 现有 `localization.sh`；
- Agent status/WS 和 READY 回执携带实际生效配置；前端对整批响应检查拒绝、验证标志和实际 hash，异常时向全部选中飞机发送 IDLE 回滚；
- 机队 Execute 使用 Backend 批量门禁：再次比较各 Agent 生效配置与当前 GCS 配置，并要求多机全部为 global 且实际 PCD hash 唯一；防止 READY 后切换场地、后端重启或逐机准备绕过坐标系检查；
- 原始 mapping 仍可显式选择，但只允许一次准备一架；多机 Prepare 会回滚并要求使用共享 PCD。

**验证**：GCS 68 个 Python 测试通过；新增覆盖 map 自动 hash、错误 hash、路径越界、mapping 单机互斥、
Backend 下发、子进程私有环境、机队 Execute 门禁与部分失败 Land 补偿；Python 3.8 Noetic 容器完成全量
py_compile、shell/launch/rospy smoke；前端生产构建通过。

---

## INIT-AUTO 全图自动初始化

**问题**：global 模式多机使用时，无人机离地图原点超过约 5–10m 即 LIO 启动失败。
根因在 fast_lio 的 initial_align 节点：Scan Context 粗定位被关闭（advanced_by_scan_context: false），
直接从 GCS 下发的初值跑 fast_gicp；即便打开 SC，候选区也只是以地图原点为中心的 10m×10m 网格
（coarse_init_pose 平移整张地图云 ±5m 生成候选）。GICP 收敛域只有几米，超出即 fitness>2.0 拒绝 →
不发布 /initial_odom_for_lio → laserMapping 90s 超时退出（宁败不飞）。每机手动配置初值繁琐且
飞机摆好后再挪回原点不现实。

**方案**：全图描述子检索自动初始化。离线把场地 PCD 按 1m 网格生成 Scan Context 描述子库（.scdb，
每格 22m 半径、29.5° 垂直视场带、体素 0.3、与雷达同高度 z 基准），生成入口为根目录脚本
./build_map_scdb.sh MAP.pcd（内部调用 fast_lio 包的 build_map_scdb 可执行文件，随 catkin 构建）；
initial_align 新增 init_mode=auto：
累积 10 帧扫描 → 环键 KNN（top-5）→ 逐对 SC 距离取 top-3 候选 → 每个候选 fast_gicp 精配准 →
fitness≤2.0 的最优者接受；全部失败回退 GCS 初值路径，再失败拒启。结果（实际位姿/source/fitness）写入
/tmp/lio_init_result.yaml 供诊断。**init_mode=auto 已是默认**：无需任何 GCS 改动与 env 设置，每次 global
启动都会自动定位；`INIT_MODE=pose` 可强制回到传统初值路径。GCS 侧位姿学习回写闭环经确认不需要，
已从计划移除。

**改动文件**：
- tests/sc_init_validate.cpp（新增，本地验证工具不入库）：离线验证 harness——合成 10m/20m 多航向扫描，SC 检索 + top-K GICP 全链路
- src/localization/FAST_LIO/tools/build_map_scdb.cpp（新增，catkin 目标）：PCD → .scdb 生成工具（格式 SCDB1：magic/version/grid_step/count，每格 x,y + 40×120 desc + 40 ringkey）
- build_map_scdb.sh（新增，根目录入口）：./build_map_scdb.sh MAP.pcd 一行生成 MAP.pcd.scdb
- .gitignore：*.scdb 与本地 harness 不入库
- src/localization/FAST_LIO/src/initial_align.cpp：load_scdb / auto_align_by_scdb / align_from_pose / write_init_result；main 按 init_mode 分支；顺手修复 test() 重复调用会累积修改 icp_convergence_condition_for_rot_ 的 bug
- config/initial_align.yaml、launch/initial_align.launch、launch/global_localization_mid360.launch：init_mode/scdb 参数与 env 透传

**容器内验证（new.pcd，53×38m，2106 格）**：
- 离线 harness：5/5 地图内用例通过——10m/20m、航向 0/45/90/135/180°，SC 粗定位误差 0.30–0.34m、0°，GICP 后 1–2mm、≤0.01°，fitness 0.001；检索耗时 ~1.5ms，建库 3.1s
- 冒烟：auto 模式 scdb 加载 2106 条；pose 模式行为不变；auto+缺 scdb+禁回退 → 快速失败并写 success: false 结果文件

**真机问题定位与修复（2026-08-16，scdb v3）**：
- 现象：真机 ~8.5m 处检索全挂，真位置 SC 距离 0.61，top 候选全部错位
- 定位：失败落盘扫描 + 离线诊断。真值位姿处 GICP fitness=0.128（地图与现场一致）；
  扫描地面 z≈-1.45 vs 地图地面 z≈-0.35 —— 测试时无人机比建图时高约 1.1m（放台面上）
- 根因：建库的垂直视场带滤波假设"无人机在地面"，高度偏移后 DB 与真实扫描切的是
  不同纵向切片，描述子系统性错位（z 常量偏移本身无害，切片内容错位致命）
- 修复：v3 去掉视场带滤波（最近表面带编码已保证单视角一致性）；新增 top-2 候选
  位姿一致性保险（>3m 分歧即拒绝，防相似结构误判）；版本号防旧库混用
- 验证：真实扫描 + v3 DB，真位置进 top-8（0.393），top-2 GICP 收敛同一位置
  (8.449, -1.344, 5.62°) fitness 0.02–0.07，与真值 GICP 结果一致

**NX 实测成功（2026-08-16）**：v3 复测通过——检索 top 候选集中在正确区域，3 个候选 GICP
全部收敛（fitness 0.024–0.072），top-2 一致性通过，接受位姿 (8.4488, -1.3451, 0.163) 与
离线预测 (8.449, -1.344) 偏差 <2mm，LIO 带地图正常启动。

**待办**：
- 多机（2 架以上）同一 PCD 自动定位真机验收；30 机编队实验前在目标场地做一次全机位验证
- .scdb 随 PCD 分发：`<map>.scdb` 与 PCD 同机分发（不入库）；地图更新后跑 ./build_map_scdb.sh MAP.pcd 重建
- 观察项：GICP 最终姿态相对重力对齐有 ~13° 残余（LIO 由 IMU 快速收敛，暂不处理；如后续起飞品质异常再评估）
