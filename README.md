# PRE_MAP_VLN

PRE_MAP_VLN 是一个 Habitat-Sim + FALCON 的室内预探索项目，配合 Boxer 生成全局 3D 物体框、OccuSG 生成房间几何区域，并在第二阶段使用任务图、候选视角和在线 RGB-D 感知执行多任务 VLN。

本文档以“新机器从 Git clone 开始”为目标，区分最小验证、第一阶段探索、房间/物体后处理和第二阶段演示。大型数据、模型权重、第三方源码、Docker 镜像和运行结果不进入主 Git。

## 先了解三个事实

1. git clone 只得到主仓库代码，不会得到 HM3D 场景、Boxer 权重、.envs、Bag 或 Docker 镜像。
2. 通用第一阶段入口 `run_hm3d_stage1_3d.sh` 是三维无人机探索：保留三维 viewpoint、三维 A*、可变 z、规划 yaw 和三维碰撞检查。旧的 `run_00166_stage1.sh` 与七场景批处理仍是兼容性单层入口；第二阶段底层目前仍是二维自由栅格 A*。
3. Docker 只承载 ROS1 FALCON 和 ROS2 OccuSG。Habitat-Sim 在宿主机 Python 环境中运行，Boxer 也在宿主机独立 Python 环境中运行。

## 项目结构

~~~text
PRE_MAP_VLN/
├── scripts/                   # 安装、运行、后处理、评估和回放入口
├── stage1/                    # 结构物体策略等第一阶段逻辑
├── stage2/                    # task graph、候选视角、联合规划、感知与恢复
├── ros_ws/src/pre_map_bridge  # Habitat/FALCON 的 ROS 文件桥和 RViz 节点
├── containers/                # FALCON Noetic、OccuSG Humble 镜像定义
├── patches/                   # 对第三方源码的可审计补丁
├── config/                    # Boxer 词表和语义别名
├── tests/                     # 单元测试
├── data/                      # 本地场景、episode 和基线数据（Git 忽略）
├── checkpoints/               # 模型权重（Git 忽略）
├── outputs/                  # Bag、地图、日志、场景图和评估结果（Git 忽略）
├── runtime/                  # Docker/ROS 临时桥接文件（Git 忽略）
├── third_party/              # 固定版本的第三方源码（Git 忽略）
├── environment.lock.yml      # 已验证环境的版本记录，不是完整 Conda 导出文件
└── third_party.lock.yaml     # 第三方仓库 URL 和 commit 锁定记录
~~~

## 1. 克隆和获取第三方源码

~~~bash
git clone https://github.com/123awsd/MAP_VLN.git
cd MAP_VLN

# 当前主仓库没有必须的 Git submodule；保留这条命令以兼容未来版本
git submodule update --init --recursive

# 按 third_party.lock.yaml 获取固定 commit
./scripts/fetch_dependencies.sh
~~~

fetch_dependencies.sh 会获取 FALCON、Boxer、OccuSG、Habitat-Lab、Open3D、NLopt，以及可选的 Open-Nav、VLN-Zero、Spatial-X。third_party/ 被 .gitignore 忽略，这是有意的；Docker 构建前必须先执行该脚本。

当前 FALCON 的项目修复以 patches/falcon/*.patch 保存，Docker 构建时自动应用。不要依赖 third_party/FALCON 中未提交的临时修改来复现结果。

## 2. 准备场景和模型

### HM3D 场景

HM3D 不进入 Git，需要按照许可证和官方渠道单独获取。项目开发用的 example 数据布局如下：

~~~text
data/scene_datasets/hm3d/example/
├── hm3d_annotated_example_basis.scene_dataset_config.json
├── 00861-GLAQ4DNUx5U/
│   ├── GLAQ4DNUx5U.basis.glb
│   ├── GLAQ4DNUx5U.basis.navmesh
│   ├── GLAQ4DNUx5U.semantic.glb
│   └── GLAQ4DNUx5U.semantic.txt
├── 00337-CFVBbU9Rsyb/
└── 00770-NBg5UqG3di3/
~~~

如果数据包实际放在其他目录，可以建立链接：

~~~bash
mkdir -p data/scene_datasets
ln -s /absolute/path/to/hm3d data/scene_datasets/hm3d
~~~

七场景第一阶段和 00166 专用入口支持通过环境变量指定训练集路径：

~~~bash
export PRE_MAP_VLN_HM3D_TRAIN_ROOT=/absolute/path/to/hm3d/train
export PRE_MAP_VLN_HM3D_SCENE_CONFIG=/absolute/path/to/hm3d/hm3d_annotated_basis.scene_dataset_config.json
~~~

这两个变量会被 scripts/run_00166_stage1.sh 和 scripts/run_hm3d_stage1_seven.sh 使用。评估脚本还需要对应的 survey.json；survey 只用于离线真值评估，不会喂给在线规划器。

### Boxer 权重

Boxer 代码和权重均不进入 Git。获取 Boxer 源码后，按其上游说明下载权重到：

~~~text
third_party/boxer/ckpts/
~~~

上游提供 `third_party/boxer/scripts/download_ckpts.sh`。当前三份必需权重及 SHA-256 已写入 `environment.lock.yml`，可用复现检查脚本逐字节验证。

### DashScope/Qwen 凭据（可选）

只有 Qwen task graph、结构物体策略、房间语义命名或 VLM 复核需要 API key。密钥只写入 Git 忽略的 .secrets/：

~~~bash
.envs/habitat/bin/python scripts/configure_secrets.py --qwen-stdin
~~~

Qwen 调用默认使用 qwen3.7-plus，结果和估算费用写入 API ledger；密钥不要写入命令记录、README、日志或 Git。

## 3. Python 环境

.envs/ 被 Git 忽略。项目使用两个相互隔离的宿主机环境：

| 环境 | 用途 | 已验证版本 |
|---|---|---|
| .envs/habitat | Habitat-Sim、Habitat-Lab、第一/二阶段 Python、评估 | Python 3.9.23，Habitat-Sim/Lab 0.3.3 |
| .envs/boxer | Boxer 离线 2D/3D 检测和融合 | Python 3.12.14，PyTorch 2.13.0，CUDA 由本机驱动决定 |

新机器可以按当前已验证版本创建两个环境：

~~~bash
./scripts/fetch_dependencies.sh
./scripts/setup_host_envs.sh
~~~

`setup_host_envs.sh` 默认安装 PyTorch CUDA 13.0 构建。不同 GPU/驱动可显式指定 PyTorch wheel 源和版本：

~~~bash
PRE_MAP_VLN_TORCH_INDEX_URL=https://download.pytorch.org/whl/cu130 \
PRE_MAP_VLN_TORCH_VERSION=2.13.0 \
./scripts/setup_host_envs.sh
~~~

宿主机准备完成后，先验证：

~~~bash
.envs/habitat/bin/python -c 'import habitat, habitat_sim; print(habitat.__version__, habitat_sim.__version__)'
.envs/boxer/bin/python -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
~~~

只验证纯 Python 模块时：

~~~bash
.envs/habitat/bin/python -m unittest discover -s tests -v
~~~

## 4. Docker 构建和影响

Docker 镜像不会随 Git clone 一起出现，第一次使用必须构建：

~~~bash
docker compose build falcon occusg
~~~

镜像职责如下：

| 服务 | 镜像 | 作用 | 是否需要 GPU |
|---|---|---|---|
| falcon | pre-map-vln/falcon-noetic:local | Ubuntu 20.04 + ROS1 Noetic + FALCON + ROS bridge | 不需要，当前容器按 CPU 使用 |
| occusg | pre-map-vln/occusg-humble:local | Ubuntu 22.04 + ROS2 Humble + OccuSG | 不需要，构建和分割主要使用 CPU |

Habitat-Sim 的真实 RGB/Depth 渲染在宿主机 .envs/habitat 中运行；有 NVIDIA GPU 时使用宿主机 EGL/OpenGL。Boxer 的推理由 .envs/boxer 负责。容器中的 nvidia-smi 兼容脚本只是 FALCON 启动信息，不代表 FALCON 在容器内完成 GPU 推理。

只修改 scripts、被挂载的 ROS bridge 脚本/配置、outputs 或 runtime 时通常不需要重建镜像。

修改 FALCON 源码、patches/falcon/ 或 FALCON Dockerfile 后重建 falcon：

~~~bash
docker compose build falcon
~~~

修改 OccuSG 源码或 OccuSG Dockerfile 后重建 occusg：

~~~bash
docker compose build occusg
~~~

如果出现 Docker daemon 权限错误，推荐把当前用户加入 Docker 用户组并重新登录：

~~~bash
sudo usermod -aG docker "$USER"
newgrp docker
docker info
~~~

第一阶段脚本会自动优先使用普通 `docker`，不可用时再尝试 `sudo -n docker`。特殊环境也可设置 `PRE_MAP_VLN_DOCKER='sudo docker'`。新机器仍推荐配置 Docker 用户组。

### 新机器一键初始化

安装好 Git、Conda、Docker Compose 和 NVIDIA 驱动后，可执行：

~~~bash
./scripts/bootstrap_new_machine.sh --download-boxer-weights
~~~

该命令会获取锁定的第三方源码、创建两个宿主机环境、下载 Boxer 权重并构建 FALCON/OccuSG 镜像。HM3D 受许可证和体积限制，仍需单独复制或下载，然后设置前述两个 `PRE_MAP_VLN_HM3D_*` 环境变量。

最终检查（可替换成任意已有场景）：

~~~bash
./scripts/check_reproducibility.py --scene-id 00166-RaYrxWt5pR1
~~~

## 5. 最小可运行验证

准备 example 场景后，先验证 Habitat-Sim 能加载 GLB、navmesh、RGB、Depth 和 Semantic：

~~~bash
.envs/habitat/bin/python scripts/smoke_habitat.py
~~~

成功后检查 ROS 容器：

~~~bash
docker compose run --rm falcon bash -lc 'rosversion -d'
docker compose run --rm occusg bash -lc 'ros2 --version'
~~~

## 6. 第一阶段运行

### 00337 三维探索验证基线

2026-08-28 已在 `00337-CFVBbU9Rsyb` 完成一次真实三维 FALCON 探索验证。Habitat 执行完整三维 `PositionCommand` 和规划 yaw，FALCON 保留三维 A*、三维 viewpoint 与发布前碰撞检查；轨迹没有固定或压平 z。该次运行完整探索了两层楼，并由 FALCON 以 `No frontier detected` 正常进入 `FINISH`：

- Habitat elapsed：288.5 s；FALCON exploration duration：283.0 s；
- 最终 map coverage：1229.23；轨迹 z 范围：0.808–10.002 m；
- 条件式安全 A* 前缀成功发布 26 次，最长连续静止约 3 s，未出现持续振荡；
- 44 次预测碰撞均在轨迹发布前被拒绝，没有把碰撞轨迹作为正常探索执行；
- 验证镜像：`pre-map-vln/falcon-noetic:local`，image ID `sha256:1f539038fb69e30246da8b0743333b52ae2c176258d5f2edd3ceca2e6d9a8b6c`。

成功 Bag 名为 `hm3d_stage1_00337_conditionalprefix_full_20260828_raw.bag`，约 17.2 GB。Bag、HM3D 数据和本地 Docker 镜像不进入 Git；在其他机器上应从本提交重新获取固定版本依赖并构建镜像。

其他 HM3D 场景不需要手写三维边界。通用入口会根据固定 seed 的 Habitat 起点和 navmesh 自动生成场景专用三维 YAML，并将同一精确起点传给实际运行：

~~~bash
./scripts/run_hm3d_stage1_3d.sh 00166-RaYrxWt5pR1 full3d_v1 600 5 5 true
~~~

参数依次为场景 ID、实验名、最大时长、Habitat 频率、关键帧间隔和是否打开 RViz。默认录制 compact Bag：保留最终点云地图、轨迹和稀疏 RGB，避免重复保存每一帧完整占据图。

### 单场景 00166

这是最适合第一次完整运行的入口。它拒绝未登记的场景名，并且如果输出已存在会拒绝覆盖：

~~~bash
export PRE_MAP_VLN_HM3D_TRAIN_ROOT=/absolute/path/to/hm3d/train
export PRE_MAP_VLN_HM3D_SCENE_CONFIG=/absolute/path/to/hm3d/hm3d_annotated_basis.scene_dataset_config.json

./scripts/run_00166_stage1.sh ground_v6 300 10 5 hm3d_00166_v6
~~~

参数依次为实验名、最大时长（秒）、Habitat 频率、关键帧间隔和登记的 FALCON 配置名。输出包括关键帧、raw Bag、run.log 和 run_result.json。

FINISH 只表示 FALCON 状态机完成；真实覆盖率、规划失败、预测碰撞、楼层和 z 范围必须再用评估脚本确认。

### 七场景批次

场景列表在脚本内显式登记，只包含项目的七张 HM3D 图。先单跑一个场景：

~~~bash
./scripts/run_hm3d_stage1_seven.sh 300 10 5 stage1_seven_v6 00087-YY8rqV6L6rf 0
~~~

全部七张图：

~~~bash
./scripts/run_hm3d_stage1_seven.sh 300 10 5 stage1_seven_v6 all 0
~~~

该旧批处理入口仍是单层结果。不要用 Bag 正常关闭、无人机还在移动或 frontier 数量变少单独宣称探索完成，应查看 truth_metrics.json、run_result.json 和 REPORT.md。新的三维实验应使用上一节的通用入口逐场运行。

## 7. 第一阶段后处理：Boxer、OccuSG 和房间语义

已有 episode 可以离线生成物体框、结构占据图和场景图：

~~~bash
./scripts/run_stage1_postprocess.sh hm3d_stage1_complete_v3
.envs/habitat/bin/python scripts/validate_stage1.py --episode hm3d_stage1_complete_v3
~~~

典型数据流为：

~~~text
Habitat RGB-D/Pose
       ↓
Boxer 2D/3D boxes + 跨帧融合
       ↓
Qwen 结构物体策略（墙/柱/门框保留）
       ↓
OccuSG 结构网格和几何房间区域
       ↓
物体按多边形/有限边界距离归属
       ↓
Qwen 根据每个房间的物体清单推断 bedroom/kitchen 等语义
~~~

多层结果可使用通用离线入口自动从 FALCON 轨迹/点云推断楼层（不读取真值楼层），为每层生成导航网格、结构网格、OccuSG 区域和场景图：

~~~bash
.envs/habitat/bin/python scripts/run_multifloor_room_pipeline.py \
  outputs/stage1_3d/<scene-id>/<run-name> \
  outputs/boxer/<box-run>/episode/<vocab>_3dbbs_fused.csv \
  outputs/room_pipeline/<output-name> \
  --max-floor 3 --auto-policy
~~~

实验性的纯几何墙提取（不用真值、语义或物体框）可在上述逐层导航网格基础上运行：

~~~bash
.envs/habitat/bin/python scripts/extract_multifloor_wall_grid.py \
  <run-dir>/bag_export/map_occupied.pcd \
  <run-dir>/bag_export/map_free.pcd \
  outputs/room_pipeline/<output-name> \
  outputs/wall_extraction/<output-name> --max-floor 3
~~~

多楼层 RViz 可视化会根据真实换层轨迹、occupied/free 点云离线重建跨层通道候选，并用独立图层显示，不修改 OccuSG：

~~~bash
./scripts/view_floor_boxes.sh <run-dir> <fused-boxes.csv> 0 3
~~~

算法、置信度与颜色说明见 `docs/跨层通道重建.md`。

基于几何墙图运行 OccuSG，并在其原始输出之后应用 transition mask：

~~~bash
.envs/habitat/bin/python scripts/run_transition_room_pipeline.py \
  <run-dir> <geometry-wall-grid-dir> <output-dir> \
  --max-floor 3 --decomp-threshold 1.8
~~~

Qwen 不生成房间坐标，也不创建、合并或拆分几何房间。房间几何来自 Occupancy/OccuSG；房间语义是物体证据驱动的提示，不是 HM3D 人工房间真值。

论文图建议使用房间外墙/外轮廓；第二阶段候选视角必须使用 free 区域和障碍物安全距离。两者应共享同一个 room_id，不能把外墙轮廓直接当作可飞行空间。

## 8. 第二阶段演示

第二阶段依赖已经生成的 scene graph、occupancy grid、候选任务输入和 Habitat 场景：

~~~bash
./scripts/run_stage2_demo.sh
~~~

常用验收和回放：

~~~bash
.envs/habitat/bin/python scripts/validate_stage2.py
./scripts/replay_stage2_rviz.sh hm3d_stage2_complete false 1.0
~~~

默认使用本地 OWLv2 做开放词表感知；Qwen 只在 task graph、结构策略或显式 VLM 复核/语义恢复路径中调用。第二阶段当前的底层规划仍是二维自由栅格 A*，不能写成完整三维 ESDF 或真实动力学飞行验证。

完整演示：

~~~bash
./scripts/run_final_demos.sh
.envs/habitat/bin/python scripts/validate_final_demos.py
~~~

## 9. RViz / Bag 回放

先准备 X11 授权，再通过项目脚本回放。脚本参数依次为 Bag 名（不含 .bag）、是否循环、倍速、是否打开俯视窗口：

~~~bash
./scripts/replay_bag_rviz.sh <bag_name> false 1.0 false
~~~

例如：

~~~bash
./scripts/replay_bag_rviz.sh stage1_seven_v6_00087-YY8rqV6L6rf_final false 1.0 false
~~~

如果 Bag 报告 unindexed，只对本地副本执行：

~~~bash
docker compose run --rm falcon \
  rosbag reindex /workspace/shared/outputs/bags/<bag_name>.bag
~~~

回放只读取 outputs/bags/，不会重新运行 Habitat，也不会改变地图规划结果。

## 10. 输出和版本管理

以下目录默认被 Git 忽略：

- data/：场景、episode、survey 和外部基线数据；
- checkpoints/、third_party/*、.envs/：权重、源码和环境；
- outputs/：Bag、点云、轨迹、场景图、报告和缓存；
- runtime/、logs/：临时桥文件、ROS 日志和 X11 授权。

发布一个可复现实验时，需要同时记录：

- 主仓库 Git commit；
- third_party.lock.yaml 中的第三方 commit；
- Docker 镜像构建时间和 image ID；
- Python 环境版本；
- 场景文件、survey 和模型权重的路径/校验和；
- 运行配置、随机种子、Bag、评估报告和失败原因。

不要把 Bag、模型权重、API key 或宿主机绝对路径提交到 Git。需要共享结果时，单独打包 outputs/<experiment>/ 和对应报告，或使用外部对象存储。

提交前检查：

~~~bash
git status --short
git diff --check
git -C third_party/FALCON status --short
~~~

确认代码、Dockerfile、patch 和脚本都已进入暂存区后，再由项目维护者执行 commit/push。自动初始化脚本只安装依赖和构建镜像，不会提交或推送。

## 11. 常见问题

### third_party/FALCON 或 third_party/nlopt 不存在

~~~bash
./scripts/fetch_dependencies.sh
~~~

### habitat_sim 导入失败

确认使用 .envs/habitat/bin/python，并检查 Habitat-Sim 0.3.3、Python 3.9 和对应的 Bullet 构建。不要把 Habitat 环境和 Boxer 的 NumPy/PyTorch 混装。

### Docker daemon 权限错误

优先配置 Docker 用户组；不要因为权限错误反复重建镜像。检查 docker info 和 docker compose version。

### RViz 没有窗口或 X11 错误

确认在有图形桌面的终端运行，并先执行：

~~~bash
./scripts/prepare_rviz_xauth.sh
echo "$DISPLAY"
~~~

无图形桌面时可以只运行 Habitat、Boxer、OccuSG 和评估，不启动 RViz。

### 运行输出已存在

项目脚本默认拒绝覆盖实验目录和 Bag。使用新的实验名或批次名；不要删除旧结果来“修复”运行。

### 哪一部分是完整三维？

通用第一阶段入口执行三维 FALCON 探索，不固定 z，也不把 A* 压成二维；旧单层入口和第二阶段二维自由栅格 A* 不应被写成完整三维。无论入口如何，`FINISH` 都必须结合覆盖率、轨迹、预测碰撞和日志判定，不能把超时、振荡或 Bag 正常关闭冒充探索完成。

## 12. 设计文档

- docs/第一阶段设计.md：FALCON、Habitat、Boxer、OccuSG 和第一阶段边界；
- docs/第二阶段设计.md：task graph、候选视角、联合规划和在线感知；
- docs/房间分割与真值对比.md：房间几何和 Habitat navmesh 对照；
- docs/第六阶段语义恢复搜索.md：目标缺失后的语义恢复搜索；
- third_party.lock.yaml：第三方源码锁定版本；
- environment.lock.yml：已验证宿主机环境版本记录；
- docs/新机器完整复现.md：从 clone、外部资产到三维运行的换机清单；
- docs/跨层通道重建.md：轨迹播种、局部截面和不确定边界可视化；
- 记忆/环境与版本.md：数据、权重、镜像和系统版本记录。
