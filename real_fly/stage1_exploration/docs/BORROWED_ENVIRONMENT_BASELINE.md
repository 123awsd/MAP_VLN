# 借用环境基线与实验后恢复清单

记录时间：2026-09-04（Asia/Shanghai）

这是一份只读基线记录，服务于本次 Falcon 真机手持建图实验结束后的核对。
它不是学长目录的副本，也不包含密码、代理订阅、Token 或其他认证信息。

## 基线范围

- 项目目录：`/home/nv/SL_WS/PRE_MAP_VLN_real_fly`
- 本次实验前项目提交：`e80e475`（`real-fly`）
- 本次 Stage-1 适配提交：`6665711`（`feat(real-fly): add isolated Stage-1 handheld mapping pipeline`）
- 学长 underlay：`/home/nv/dls_ws`
- underlay 分支/提交：`main` / `f2e83d7aa42c79d679933bf9f803342749a7b16b`
- 记录时 `/home/nv/dls_ws` 工作区无 Git 修改。

本次适配只新增/修改了项目内的
`real_fly/stage1_exploration/`。没有修改 `/home/nv/dls_ws`、共享仿真
workspace、`/opt/ros/noetic`、用户 shell 启动文件、飞控或持久化网络配置。

## ROS 与网络基线

以下是实验前检查时观察到的状态；ROS 节点和链路状态是动态信息，恢复时应
重新读取，不能按名称批量 kill。

- ROS：Noetic，underlay 为 `/opt/ros/noetic`。
- 学长原 ROS master：`http://localhost:11311`。
- 当时观察到的节点包括 `/rosout`、GCS 节点、`/debugPx4ctrl` 和
  `/ekf_quat/ekf_odom`；其中后两个在检查时不可联系。它们不属于 Stage-1，
  不得由恢复脚本停止。
- Wi-Fi：`wlan0`，`192.168.0.250/24`，默认路由经 `192.168.0.1`。
- 当前有线接口：`eth0`；检查时无地址、`NO-CARRIER`、`Link detected: no`。
- 本次 Stage-1 没有给 `eth0` 设置地址，没有改变 `wlan0` 或默认路由。
- Stage-1 使用的隔离 master 约定为本机 `11312`；只有显式启动时才存在，
  不应替换 `11311`。

## APT 基线与当前阻塞

实验准备期间只执行过一次允许的 `apt-get update`，没有执行升级、自动清理、
卸载或降级。当前只读核对显示：

- `clash-verge` 版本 `2.5.2` 处于 `install ok unpacked`、尚未配置状态；
- `libayatana-appindicator3-1` 未安装，但 focal arm64 有候选；
- `libwebkit2gtk-4.1-0` 未安装，当前 focal arm64 软件源没有候选；
- RealSense 和 `v4l-utils` 安装模拟因此被 APT 拒绝；没有任何目标包被解包；
- 未执行 `apt --fix-broken`，未执行 `dpkg --configure -a`，也未修改 Clash
  配置或服务。

后续处理必须先由负责人决定如何处理 `clash-verge` 的半安装状态。任何会
删除、替换或强制配置 Clash 的方案都不能作为 Stage-1 的隐含步骤。

## 学长 Livox 配置基线

原文件（只读记录）：

`/home/nv/dls_ws/src/drivers/livox_ros_driver2/config/MID360_config.json`

文件中的历史配置值为：

- 主机 IP：`192.168.1.5`
- MID-360 IP：`192.168.1.107`
- LiDAR/主机端口：命令 `56100/56101`、推送 `56200/56201`、点云
  `56300/56301`、IMU `56400/56401`、日志 `56500/56501`
- `pcl_data_type=1`

这些地址只是原文件值，不是本次设备的实测值。实验中不能直接把它们恢复到
当前 LAN1，也不能据此判断 MID-360 IP。原文件 SHA-256：

```text
496c1a01ce61798d5722117f1f999cd13c3a501718e3a3087ff8fb059b52300c
```

相关原始 launch：

`/home/nv/dls_ws/src/localization/FAST_LIO/launch/lidar.launch`

- 默认 `LIVOX_LIDAR_TYPE` 为 `mid360s`。
- 可通过环境变量选择 LiDAR 类型、设备 IP、配置文件和主机 IP。
- 原 launch 启动 `livox_ros_driver2_node`，没有接入本次 Stage-1 的录包器。
- 原文件 SHA-256：

```text
b72bb464617029838351ac991cfea499cc9e8f1303ef1ed15c39ec79ac3d459c
```

## 学长 FAST-LIO 配置基线

### `mid360.yaml`

路径：`/home/nv/dls_ws/src/localization/FAST_LIO/config/mid360.yaml`

- LiDAR topic：`/livox/lidar`
- IMU topic：`/mavros/imu/data`
- 包含飞机安装方式的约 15° LiDAR 外参：
  `T=(-0.011,-0.02329,0.04412)`
- `pcd_save_en=false`
- 启用了原有 CPU 监控相关参数。
- SHA-256：

```text
a0730bd7a618318b2d76f7e3c1050f3be917556c5afa8f810544bc7437159565
```

### `mid360_1.yaml`

路径：`/home/nv/dls_ws/src/localization/FAST_LIO/config/mid360_1.yaml`

- LiDAR topic：`/livox/lidar`
- IMU topic：`/livox/imu`
- 包含原有机体安装假设；`T=(-0.011,-0.02329,0.04412)`。
- `pcd_save_en=true`
- 启用了原有 CPU affinity 和 debug 发布参数。
- SHA-256：

```text
06d073ba96871e57e744a7883052868ca2caf1b1292f23d707bb08bc1ebea1fd
```

以上两个文件均不是本次手持装置的最终标定，Stage-1 使用项目内独立的
`config/fast_lio_mid360_handheld.yaml`，不会覆盖它们。

## 本次 Stage-1 的隔离方式

- `scripts/env.sh` 只影响当前 shell/子进程，不写 `.bashrc`、`.profile` 或
  ROS 全局配置。
- 本地 package `stage1_fast_lio` 只读取学长 FAST-LIO 源码和已编译消息头，
  输出二进制放在项目内 `real_fly/stage1_exploration/ros_ws/`。
- 每次建图的 FAST-LIO `ROOT_DIR` 应放在
  `real_fly/stage1_exploration/data/<run_id>/mapping/`，避免写入学长原始
  FAST-LIO 的 `Log/`、`PCD/`。
- 录包器只接受显式验证的传感器/TF topic，拒绝飞控、控制和仿真 topic；不
  使用 `rosbag record -a`。
- Stage-1 不启动 MAVROS、PX4、ArduPilot、MAVLink、规划器、电机或任何
  运动控制节点。
- 本次只编译了本地映射程序；没有启动传感器 launch、rosbag 或实时建图。

## 实验结束后的恢复顺序

1. 只停止本次 Stage-1 自己启动的传感器 launch、录包器、回放器和隔离
   master。优先使用对应脚本的 `Ctrl-C`/记录的 PID；不要使用 `pkill`、
   `killall` 或按名称杀进程。
2. 退出曾经 `source scripts/env.sh` 的实验 shell，或直接打开新 shell。
   不需要编辑 shell 启动文件；`11311` 不需要重启。
3. 若实验中曾经临时配置 LAN1，只能根据当次记录的“配置前状态”恢复实际
   LAN1 接口；不得触碰 `wlan0` 和默认路由。本次实验没有进行过这一步，
   因此目前没有网络恢复命令。
4. 不要把 Stage-1 配置复制回 `/home/nv/dls_ws`，也不要用 `cp`、`rsync`
   或 `git checkout` 覆盖学长文件。恢复的目标是确认原文件未被改动。
5. 保留 `data/<run_id>/` 和 `outputs/<run_id>/` 实验产物；它们是被忽略的
   实验数据，不应在“恢复环境”时顺手删除。

## 恢复核对命令（只读）

```bash
cd /home/nv/SL_WS/PRE_MAP_VLN_real_fly
git -C /home/nv/dls_ws status --short
sha256sum /home/nv/dls_ws/src/drivers/livox_ros_driver2/config/MID360_config.json
sha256sum /home/nv/dls_ws/src/localization/FAST_LIO/config/mid360.yaml
sha256sum /home/nv/dls_ws/src/localization/FAST_LIO/config/mid360_1.yaml
sha256sum /home/nv/dls_ws/src/localization/FAST_LIO/launch/lidar.launch
ip -br addr
ip route
git status --short
```

恢复通过的最低条件：underlay Git 状态为空、四个原文件 SHA-256 与本记录
一致、`wlan0` 和默认路由与基线一致、未残留本次 Stage-1 进程，并且没有
停止学长原有 ROS 节点。若任何一项不一致，应先记录差异，不要自动覆盖或
删除。

## 实验后追加变更记录（2026-09-05）

上面的“基线”描述保留实验开始时的历史状态。经设备负责人后续明确授权，
实际发生了以下系统变更，恢复或重刷前应以本节为准：

- 2026-09-04 执行 `apt-get --fix-broken install`，APT 删除了半安装状态的
  `clash-verge 2.5.2`；没有修改 Mihomo/Clash 订阅文件的记录。
- 随后新装 `ros-noetic-realsense2-camera 2.3.2`、
  `ros-noetic-realsense2-description 2.3.2`、`ros-noetic-librealsense2 2.50.0`、
  `v4l-utils 1.18.0` 及其必要库。
- 2026-09-05 新装 `python3.8-venv 3.8.10`。该次 APT 交易为 0 升级、1 新装、
  0 删除、0 降级，没有重启网络服务。
- NetworkManager 中新增持久连接 `lidar-eth`，UUID
  `a169cb9f-8140-4dc1-a309-aebecdcaa527`，绑定 `eth0`，静态地址
  `192.168.1.5/24`，无网关；MID-360S 实测地址为 `192.168.1.122`。
- 核对时默认路由仍为 `wlan0 -> 192.168.0.1`，Wi-Fi 地址仍为
  `192.168.0.250/24`；SSH、NoMachine 和 ROS Noetic 全局配置没有因阶段二
  离线准备而修改。
- 阶段二依赖、Boxer 源码、权重和 Python 包均位于项目
  `real_fly/stage2_offline/` 下。没有安装 CUDA Toolkit，也没有修改 JetPack、
  内核或系统时间。

若不采用整机重刷，恢复 LAN1 可先只读确认连接 UUID，再由负责人决定是否执行
`nmcli connection delete a169cb9f-8140-4dc1-a309-aebecdcaa527`；不要按名称猜测
删除其他连接。APT 包也应按上述精确清单人工评估，不要执行 `apt autoremove`。
