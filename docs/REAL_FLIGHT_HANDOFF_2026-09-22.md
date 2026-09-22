# PRE_MAP_VLN 真机实验交接（2026-09-22）

> 这份文档用于把当前工作交给新的 Codex 对话。先完整阅读，再检查现场状态；不要自动起飞、解锁、切模式、发送控制命令或 push。
>
> **敏感信息警告：本文含 NX 明文登录密码，仅限当前主机本地使用。不要提交、push、上传网盘、发送到群聊或放入论文附件。**

## 给新对话的直接说明

请接手 `/home/uav/map_VLN/PRE_MAP_VLN` 的真机分支开发。目前分支为 `real-fly`，HEAD 为 `d1f9d77`，工作树有尚未提交的真机规划改动。不要切换分支、不要覆盖或删除现有改动、不要自动 push。

当前正式验证任务是：

```bash
RUN_ID=dorm_room_v3
TASK_ID=check_pot_water_toiletpaper_cup_static_3s_xy_clearance_speed04_yaw06
```

主机已成功生成最终 MINCO：4 个 phase，总时长约 `70.6117 s`；连续最小净空 `0.266 m`，速度峰值 `0.367 m/s`，角速度峰值 `0.540 rad/s`。配置上限分别为 `0.4 m/s` 和 `0.6 rad/s`，CIRI 半径保持 `0.25 m`。输出已经通过 `saved_minco_artifact.py verify`，但尚未同步 NX、尚未真机执行。

接下来优先做：先用 RViz 人工检查这条最终 MINCO；确认无误后检查 Git diff 并提交代码；再以非破坏方式同步代码和指定任务产物到 NX。任何真机动作仍需用户现场明确操作。

## 机器和目录

- 主机仓库：`/home/uav/map_VLN/PRE_MAP_VLN`
- 主机用户：`uav`
- 当前 Git 分支：`real-fly`
- 当前 HEAD：`d1f9d77 feat(real-fly): improve mapping, planning, and execution workflows`
- 远端 `origin/real-fly` 当前可见基线：`75adb99`
- NX 用户：`nv`
- NX 密码：`nv`
- NX 当前局域网地址：`192.168.0.149`
- NX 局域网 SSH：`ssh nv@192.168.0.149`
- NX 项目：`/home/nv/SL_WS/PRE_MAP_VLN_real_fly`
- 学长 DLS 工作空间：`/home/nv/dls_ws`
- 旧地址 `192.168.0.250` 不应再写入新指令。
- NX Tailscale 地址曾为 `100.89.13.29`，公网 SSH 曾使用 `ssh nv@100.89.13.29`；网络状态可能变化，使用前必须重新验证。

局域网优先使用 `192.168.0.149`。实验场地网络变化后，先从笔记本确认 NX 当前地址，再更新命令；不要把旧的 `.250` 地址继续复制到新脚本。

## 项目目标和当前边界

总流程是：手持建图 → FAST-LIO 地图与轨迹 → 主机离线语义/房间 → 任务解析与候选观察位姿 → 3D A* → 净空优化 → CIRI 安全走廊 → MINCO → RViz 人工确认 → NX 重定位 → PX4Ctrl 执行已保存轨迹。

当前采用的是“主机生成最终 MINCO，NX 原样播放”的链路，不再把最终任务重新拆成一串 SUPER 航点。SUPER/CIRI 仍作为主机离线生成安全走廊和 MINCO 的组成部分；NX 执行时不应在线重规划这条已认证轨迹。

现阶段仍不具备完整自主语义闭环：在线目标检测、任务完成判断、条件分支和 Qwen 主动恢复没有完整接入。允许做离线规划、RViz 检查和受控运动测试，不能宣称完整自主语义飞行。

## 最新正式任务与产物

```bash
RUN_ID=dorm_room_v3
TASK_ID=check_pot_water_toiletpaper_cup_static_3s_xy_clearance_speed04_yaw06
```

主机任务输入：

```text
real_fly/stage2_offline/data/dorm_room_v3/tasks/
  check_pot_water_toiletpaper_cup_static_3s_xy_clearance_speed04_yaw06/
```

最终运行产物：

```text
real_fly/stage2_runtime/missions/dorm_room_v3/
  check_pot_water_toiletpaper_cup_static_3s_xy_clearance_speed04_yaw06/
    collision.pcd
    execution_bundle.json
    final_minco.txt
    final_minco_manifest.json
    final_minco_preview.json
    full_smooth_route.txt
    planner.yaml
```

关键验证结果：

- `final_minco_manifest.json`：4 phases，`70.6117108409926 s`
- 地图 SHA256：`0e945332b4458a73430aa3e626f30c99eb892ef735206f145348193fd7779b9b`
- `execution_bundle.json` 状态仍为 `preview_only`
- 连续净空：`>= 0.266 m`（离散采样约 `0.271 m`）
- 峰值 `v/a/j/snap/yaw_rate`：`0.367 / 0.261 / 0.343 / 1.694 / 0.540`
- CIRI corridor radius：`0.25 m`
- CIRI seed length：`0.8 m`
- 碰撞图：原地图 `11,057,183` 点，经 `0.10 m` voxel 且每格至少 100 点后为 `18,992` 点

注意：上述任务和运行产物被本仓库 `.git/info/exclude` 中的 `/real_fly/` 规则隐藏，不会自动出现在 `git status`，也不会仅靠普通代码提交恢复。同步 NX 或归档前必须显式检查文件存在和哈希。

## 本次修复的真实问题

此前报错：

```text
seed line too long
```

失败 seed 约 `0.837 m`，而 full_smooth 根据 `2 × robot_r` 把 CIRI seed 长度隐式压成了 `0.5 m`。这个限制不是安全净空，只是一次送给 CIRI 的引导线跨度；它与 SUPER 配置中的 `corridor_line_max_length: 0.8` 不一致。

现已修复为：

1. full_smooth 使用 SUPER 明确配置的 `0.8 m` seed span。
2. 超长相邻引导边按原始直线几何细分，每个短段仍必须通过同一 filtered PCD、同一 `robot_r=0.25 m` 的 CIRI 验证。
3. CIRI 候选失败时向前回退到较短 seed，而不是立即抛出 `seed line too long`。
4. 最后一个重复/近重复引导点若已被上一走廊覆盖则正常闭合；否则只允许用与上一 corridor 有有效重叠的 point-CIRI 闭合。
5. `GeneratePolytopeFromLine` 的边界恢复与诊断日志会输出 seed、1 cm 采样净空、最近障碍和 CIRI 失败原因。
6. 生成脚本的 `speed=0.4` 现在同时写入 `traj_opt.boundary.max_vel`；此前只修改路线时间，规划器仍从模板读到 `0.6 m/s`。

安全约束没有被放宽：没有降低 `robot_r=0.25 m`，没有手工插绕行点，没有改观察位姿，也没有绕过 CIRI。

## 当前尚未提交的工作树

运行 `git status --short` 时可见：

```text
 M real_fly/stage2_offline/rviz/drone_room.rviz
 M real_fly/stage2_offline/rviz/publish_drone_room_visualization.py
 M real_fly/stage2_offline/scripts/export_full_smooth_route.py
 M real_fly/stage2_offline/scripts/finalize_real_mission.py
 M real_fly/stage2_offline/scripts/generate_final_minco.sh
 M real_fly/stage2_offline/scripts/run_real_stage2_task.sh
 M real_fly/vendor/dls_ws_src/mission_planner/Apps/ros1_full_smooth_mission.cpp
 M real_fly/vendor/dls_ws_src/super_planner/include/traj_opt/exp_traj_optimizer_s4.h
 M real_fly/vendor/dls_ws_src/super_planner/src/super_core/corridor_generator.cpp
?? stage2/clearance_refinement.py
?? tests/test_clearance_refinement.py
```

这些改动形成一条完整链路：

- 保留 raw A* 路径；
- 使用 ESDF 的 XY 梯度，在不改 z、端点和硬碰撞约束的前提下将路径轻推向高净空区；
- RViz 分色显示 raw A*、净空优化路径、B-spline 或最终 MINCO；
- `clearance_optimized` 可作为 full_smooth 输入；
- 修复 CIRI seed/边界/末端连接；
- 生成脚本支持净空、路径来源、速度和最大角速度参数。

不要用 `git reset --hard`、`git checkout --` 或整目录覆盖。需要回退某个试验时，只处理明确文件并先保存 diff。

## 已完成的离线验证

```bash
python3 -m unittest tests.test_clearance_refinement
bash -n real_fly/stage2_offline/scripts/generate_final_minco.sh
git diff --check
```

结果：2 个单元测试通过，shell 语法通过，diff whitespace 检查通过。

MINCO 产物验证：

```bash
cd /home/uav/map_VLN/PRE_MAP_VLN

python3 real_fly/stage2_runtime/scripts/saved_minco_artifact.py verify \
  --directory "real_fly/stage2_runtime/missions/dorm_room_v3/check_pot_water_toiletpaper_cup_static_3s_xy_clearance_speed04_yaw06" \
  --map "real_fly/stage2_offline/data/dorm_room_v3/fastlio_complete/handheld_map_dorm_room_v3_complete.pcd" \
  --mission "real_fly/stage2_offline/data/dorm_room_v3/tasks/check_pot_water_toiletpaper_cup_static_3s_xy_clearance_speed04_yaw06/planning/mission_plan.json" \
  --clearance 0.25
```

期望输出：

```json
{"status": "verified", "phases": 4, "duration_s": 70.6117108409926}
```

## 现在最先执行的命令

先只做 RViz 人工检查：

```bash
cd /home/uav/map_VLN/PRE_MAP_VLN

RUN_ID=dorm_room_v3
TASK_ID=check_pot_water_toiletpaper_cup_static_3s_xy_clearance_speed04_yaw06

./real_fly/stage2_offline/scripts/view_real_scene_rviz.sh \
  "$RUN_ID" 1 "$TASK_ID"
```

该查看器检测到 `final_minco_manifest.json` 后，会先校验任务产物，并显示：

- 橙色：raw A*
- 绿色：XY 净空优化路径
- 青色粗线：最终 MINCO
- 观察视锥和净空诊断标记

人工重点检查门洞、狭窄通道、观察位姿、轨迹高度，以及最终青色 MINCO 是否贴墙。

## 如需重新生成相同配置

现有正式任务已包含最终产物，脚本会拒绝覆盖。不要直接删除；需要重跑时先使用新 TASK_ID，或在用户明确要求后归档旧产物。

新 ID 示例：

```bash
cd /home/uav/map_VLN/PRE_MAP_VLN

bash real_fly/stage2_offline/scripts/generate_final_minco.sh \
  dorm_room_v3 \
  NEW_TASK_ID \
  0.25 \
  clearance_optimized \
  0.4 \
  0.6
```

参数顺序是：

```text
RUN_ID TASK_ID CLEARANCE_M PATH_SOURCE SPEED_MPS MAX_YAW_RATE_RAD_S
```

生成过程只使用主机 Docker 的隔离 ROS master，`--network none`，不会连接 PX4 或真机。

## 同步 NX 前的原则

当前最新代码和 `speed04_yaw06` 任务尚未确认已同步 NX。同步前必须：

1. RViz 人工检查通过。
2. 检查主机 Git diff，只提交真机相关文件。
3. 先检查 NX 的 Git 状态并备份冲突文件，不能覆盖 NX 原有未提交修改。
4. 只同步本次所需代码和指定任务目录，不同步全部实验数据。
5. 同步后在 NX 再运行 artifact verify，地图 SHA 必须一致。

不要自动 push。不要因为“已生成 MINCO”就自动起飞。

## 真机执行链路（仅供交接，不代表授权执行）

正式执行预计为：

```text
NX 全局重定位（终端保持）
→ 只读监控 PASS
→ 启动保存轨迹播放器 start_full_smooth_mission.sh
→ 可选低负载 Bag 录制
→ 启动 PX4Ctrl
→ 用户现场手动起飞并确认稳定悬停
→ 用户显式运行 trigger_saved_minco.sh --confirm-start
→ 任务结束保持悬停
→ 用户手动执行 land.sh
```

播放器必须先加载并验证保存轨迹，但轨迹不能在起飞后自动触发。触发必须由用户显式执行。任务结束不要自动降落。

真机前至少确认：

- 地图、RUN_ID、TASK_ID 完全一致；
- 重定位在 `world` 坐标系且静止时无跳变；
- 当前位姿与保存轨迹起点误差在允许范围；
- PX4Ctrl、遥控器、IMU、电池和 odometry 都新鲜；
- CH5/CH6 的含义以当前 PX4Ctrl 日志和实际配置为准，不能靠历史记忆猜；
- 门洞和现场障碍与建图时一致；
- 紧急接管和降落方式已由现场人员确认。

## 历史结果与不要混用的内容

- `check_pot_water_toiletpaper_cup_static_3s_xy_clearance_restored_01`：此前成功的 `0.6 m/s / 1.5 rad/s` 参考结果，不是当前低速正式任务。
- 大量 `centered_v*`、`repeat25_*`、`clearance_*_test` 是历史试验，不要当成当前正式任务。
- 当前正式低速任务只有 `check_pot_water_toiletpaper_cup_static_3s_xy_clearance_speed04_yaw06`。
- `.minco-build-*` 是失败或成功生成过程的诊断目录，可用于追溯，但执行时只认顶层已 seal 的产物。
- 早期报错 `seed endpoint outside boundary` 已定位并修复；不要通过降低净空或手工绕行来掩盖它。

## 安全硬约束

- 不自动起飞、解锁、切模式、发送控制命令或降落。
- 不连接 PX4 做“验证”式试飞。
- 真机前必须先离线验证和 RViz 人工检查。
- 不删除或覆盖 NX 原有修改。
- 不自动 push。
- RUN_ID、TASK_ID、地图、Bag 和结果必须隔离，防止混用。
- 不降低 `robot_r=0.25 m` 来换取规划成功，除非用户明确重新评估并授权。
- 不把 `preview_only` 误称为已获准飞行。
