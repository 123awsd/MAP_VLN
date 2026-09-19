# 主机生成，NX 原样执行

主机 `run_real_stage2_task.sh` 在静态任务通过后自动调用
`generate_final_minco.sh RUN_ID TASK_ID`。已有 mission_plan 可单独运行后者。
首次会构建 real-minco Docker 镜像及真机 vendor 源码的 x86 程序。
构建和运行目录在 stage2_offline/runtime；运行容器网络隔离，不连接飞控。

产物在 stage2_runtime/missions/RUN_ID/TASK_ID：
final_minco.txt 保存多项式、每段时间、yaw 样本和观察停留时间；
collision.pcd 为同次检验的过滤地图；planner.yaml 为参数快照；
final_minco_manifest.json 绑定上述文件、原地图、路线和 execution_bundle 的 SHA256。
manifest 最后写入；不存在时不允许 NX 执行。过滤采用现有 0.1 m/100 点规则，
不会清空起点周围障碍。净空保持 0.25 m。

主机原 RViz 命令在 manifest 存在时自动读取 final_minco_preview.json，
由保存的多项式按 20 ms 采样生成，曲线折线是显示近似，不是另一次优化。
不存在时明确提示仅为 Stage2 几何预览。

同步整个任务目录（不要只复制 execution_bundle.json），例如：

```bash
rsync -avP --partial \
  "real_fly/stage2_runtime/missions/$RUN_ID/$TASK_ID/" \
  "nv@192.168.0.149:/home/nv/SL_WS/PRE_MAP_VLN_real_fly/real_fly/stage2_runtime/missions/$RUN_ID/$TASK_ID/"
```

NX 重定位后：

```bash
cd /home/nv/SL_WS/PRE_MAP_VLN_real_fly
./real_fly/stage2_runtime/scripts/start_full_smooth_mission.sh \
  "/home/nv/dls_ws/${RUN_ID}.pcd" \
  --bundle "$PWD/real_fly/stage2_runtime/missions/$RUN_ID/$TASK_ID/execution_bundle.json"
```

执行端仅加载系数、校验净空/动力学与起点，不重新运行 A*/CIRI/MINCO。
不启动旧 SUPER fsm 或 stage2_super_adapter；PX4Ctrl 单独启动。
任务不会被 PX4Ctrl 自动触发。播放器启动后等待人工确认；无人机起飞并稳定悬停、确认 PX4Ctrl 状态正常后，执行：

```bash
cd /home/nv/SL_WS/PRE_MAP_VLN_real_fly
./real_fly/stage2_runtime/scripts/trigger_saved_minco.sh --confirm-start
```

该命令只发布一次 `/pre_map_vln/start_saved_minco`，播放器收到后才开始输出保存的 MINCO。播放器启动脚本本身不会起飞。
自动降落禁用；结束后指令停止，悬停依赖 PX4Ctrl 的状态机处理。
实时障碍检测当前仍是只读试验版，未接入自动制动。

任务产物不默认提交到 Git；源码、消息依赖和 Dockerfile 应提交。
