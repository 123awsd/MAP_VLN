# 三层完整流水线持续优化进度

更新时间：2026-10-14（Asia/Shanghai）。目标保持完整，尚未完成。

## 基线与实际状态

- 分支 real-fly、提交 caba850；初次检查只有 runtime/bridge_real_pcd/ 未跟踪，保留用户数据。
- 当前使用 runtime/falcon_current_checkpoint。容器 exploration-three-floor-trial 中六个 C++ 源文件 SHA256 与快照全部一致。未覆盖旧 falcon_gain_fix。
- 原 height_cap_trial 只有编译和配置，无 result.json、bag 或完整运行证据。
- 第八轮因屋顶外轨迹被拒绝，不是当前最佳合格结果；selected_run 历史名称也不能证明合格。
- 旧 记忆/当前状态.md 更新于八月，不能用于当前三层完成判定。

## 本次实际工作

1. 启动 outputs/exploration_height_cap_baseline_20261014 独立正常实时试跑：ROS master11314，同场景、seed7、原起点、5Hz、同步RGB-D、use-planner-yaw、碰撞检查、传感器高度上限6.25m。600秒为实验预算，不是完成依据。不启用倍速或暂停握手。
2. 新增 scripts/audit_exploration_stream.py：逐帧检查RGB/深度/位姿/里程计精确时间戳匹配、编码、位姿差、实际路径和高度；结果明确不声称屋顶安全或房间完整。
3. 已在第八轮2997帧真实bag上验证：缺匹配RGB/pose/odom均0，位姿差0；最高7.76464m，1072个深度位姿超过6.25m；路径445.17599m；timeout。负例证据保存新目录 round8_stream_audit.json。同步正常不能解释为几何安全。
4. 查询官方方法资料：FALCON https://github.com/HKUST-Aerial-Robotics/FALCON 及论文 https://arxiv.org/abs/2407.00577；Grounded SAM2 https://github.com/IDEA-Research/Grounded-SAM-2 。先查清现有输入、坐标和深度支持，未因检索而替换模型。
5. Boxer已有40帧对比中的scene_graph明确不是真值；候选同类别参考距离0.7789m、旧版本0.5198m，不能作为精度或召回率，也不足以判定SAM2改进。后续固定权重、词表及同一同步数据后复核。

## 当前实验与恢复

- 新实验目录 outputs/exploration_height_cap_baseline_20261014；trial_manifest.json 固定源哈希及配置，run_simulator.sh 保存原命令。
- 实时日志 run.log、simulator.log；live_trajectory.jsonl 保存实际桥接位姿；trial.bag.active 在录制时不应被当作最终bag。
- 本轮工具会话：planner/monitor 76438，Habitat12501。继续时先查这两个句柄和实际进程，不因观察超时重启。
- 容器旧master对应原roslaunch仅剩master/rosout，本轮使用独立master，没有停止旧会话。
- 结束后记录 result.json 原始termination，等待ROS录制正常关闭。必须用新bag重建点云、重提墙、重新分房，不能沿用第八轮红墙作为新结果。

## 下一步与未完成项

1. 完成6.25m整轮并跑独立同步/高度审计，检查每层实际轨迹、局部斜顶净空、建筑外部及受阻。
2. 建立各层墙面/自由空间覆盖与低收益重复对照，判定上限是否过度限制三层。
3. 从新深度提墙，独立检查斜顶、家具、补面、缺观测及门洞；重分房并验证错合并/错拆分/跨层/楼梯。
4. Boxer坐标、单位、相机内参、模型/权重和融合审计，固定人工样例与几何支持；识别和三维定位分别对比。
5. 更好版本才替换选定结果；最终完整重跑、联合报告、可视化和既有布局三视角视频尚未交付。

资源：首次观察磁盘可用约41GB（94%已用），RAM约21GiB available、swap已用7GiB；外层沙箱不能查询GPU，经授权执行环境可用16GB显卡，启动前约934MiB占用。只启动一个重型实验，不删除原数据、不推送远端。

## 续轮进展（实验仍在执行）

- 新增 scripts/export_synchronized_trial.py：按精确时间戳导出实际深度几何与固定40帧RGB-D/optical pose；4cm体素保留实际点，不做屋顶高度裁剪。用第八轮前三帧真实bag做小范围验证，得到5770个实际采样点及三个同步帧，证据 export_smoke/manifest.json。
- Boxer接口实测发现当前loader缺少SAM2要求的depth0/depth_projection，sam2_box_segmenter及sam2_depth_filter模块也缺失。现补齐loader字段、官方Transformers SAM2适配器及局部背景深度过滤，未修改预测框位置/大小。
- boxer_interface_validation.json：原分辨率及960方形resize各10000点，世界点逆投影到深度残差最大0；过滤验证包含重叠前景保护、框外深度保留、空mask回退及错误形状拒绝。只证明接口/几何计算，不证明模型识别或3D框更准。真实SAM2模型推理还待验证，避免与重型探索并行。
- SAM2接口依据 https://huggingface.co/docs/transformers/model_doc/sam2 ，当前Boxer环境Transformers4.57.6可导入Sam2Model/Sam2Processor。
- 实验约400秒时实际最高高度仅4.827m，尚未到三层；已出现多个碰撞受阻及FREE-space组件不可达日志。需完整数据判断局部停滞原因，不能仅凭日志判定失败，更不能因为没超过6.25m就宣称三层探索成功。

- 三个本地Boxer/DINOv3/OWLv2权重已逐文件SHA256固定，见 boxer_weight_manifest.json；尚未进行新数据模型质量比较。
- 新增两个SAM2模块位于版本跟踪目录 boxer_ext/，无需依赖忽略目录中的临时模块；adapter调用依据官方文档，实际权重推理待探索结束后单独执行。

## 6.25m基线整轮结束与拒绝选用

- Habitat12501及planner76438均正常终止；trial.bag已关闭，约1.2GB。result.json为timeout600.008s，2997帧，1098帧受阻。不可当作探索完成。
- stream_audit.json：RGB/深度/pose/odom精确匹配缺失均0，位姿差0；路径357.777m，实际高度0.20859–4.82724m，无超过6.25m位姿。未进入三层高度带，故本轮不能验证三层上限，更不能选为完整室内探索结果。
- 末段从sequence1931起持续静止，约213秒。受阻trajectory252已收到SceneGeometryFeedback并从实际里程计重规划；因此不能把问题简单归咎于漏反馈。
- stall_contact_diagnostic.json：实际点[4.21927,6.17284,0.74119]附近最近已执行接触点距离0.131654m，规划净空0.18m；这是安全退出诊断线索，不是证实全部失效因果。当前Astar存在仅0.15m起点豁免，FREE组件过滤也可能提前拒绝；需补充逐级诊断，禁止盲目放宽全局净空。
- 全量同步导出会话41769已完成；synchronized_export/manifest.json记录857421个4cm去重点和40个精确同步RGB-D样例，无屋顶裁剪。随后需用新观测重提墙，保留失败基线用于新旧比较。

- 已启动从新去重点云重新提墙：extract_new_geometry.py，工具会话68729，geometry.log；使用既有每层AE配置、实测局部PCA法向，不沿用旧墙点。该基线整体覆盖不合格，提墙用于失败分析而非最终交付。继续时核实进程和new_wall_extraction_manifest.json。

- 重新提墙已完成（68729正常exit0）：L1输入358773点、墙面108670点/42片；L2输入494189点、墙面133379点/73片；L3仅4459点、墙面0片。这些数量不能视作质量指标，但三层深度支持严重不足已得到新数据证据。下一步生成白底灰点红墙诊断并结合低收益frame_evidence.csv分析。

## 用户纠正回退范围：只选2026-10-12/13版本

- 用户明确不选八月，停止沿当前失败探索堆补丁。八月source保持独立未启用，独立编译已停止；manifest标为拒绝候选。
- 查Git277fcf7的三视角素材来源为round6_extended/preview_trimmed，其算法沿用10月12日晚round5。对应修复试跑结论明确旧round5效果较好、能进入三层；仍timeout，不能称完整验收成功。
- 现探索基线改为runtime/falcon_oct12_round5_checkpoint：manager_stable.cpp、astar_contacts.cpp、frontier_before.cpp；碰撞planner_contacts.cpp哈希与当前一致。只把三份探索源码安装回专用容器；墙体、房间、Boxer和原数据保留。
- 撤回最新观测权重/候选扩增/原地扫描恢复改动；在旧算法上单独保留manager/Astar的6.25m上限，配置沿用round5。高度安全仍需新的三层验证。
- 当前针对原workspace增量编译工具会话见本轮输出，日志oct12_restore_build.log。尚未重新跑探索，不能宣布回退后效果已验证。

- 10月12日探索恢复增量编译62794已exit0；已安装并编译成功。下一轮只使用outputs/exploration_oct12_restored_trial/config.yaml和该恢复源码，保持正常实时同步采集。

## 恢复的10月12日算法复跑已启动

- 工具会话：Habitat15173，planner/monitor68468；隔离master11314，目录outputs/exploration_oct12_restored_trial。先核实句柄，不重复启动。
- 新增可复跑scripts/run_realtime_trial_planner.sh；只处理ROS启动、触发、记录和退出，不产生探索目标或预写路线。bash -n通过。
- 首次ROS setup在nounset下失败；已修复为仅source时关闭nounset。Habitat先启动，因此触发在sequence85（约17s）。此延迟写入trial_manifest.json，新旧对照需以实际记录/执行时间为准，不能把600s预算算作600s活动探索。
- 本轮没有再添加恢复算法补丁；只验证用户选择的Oct12 round5源码及此前独立高度上限。

## 复跑进行期间的下游独立检查

- 房间自由射线导出原默认硬截断454.127331秒，绑定旧剪辑。改为默认完整bag，可用--end显式重现旧截断；旁路manifest记录bag来源、字节数、时间范围、随机种子及缺匹配pose数。三帧真实bag验证提取2个间隔采样帧，未修改探索算法。
- 新增audit_room_topology.py，并对既有R楼梯统一面积三层输出独立审计。三层9/11/10个标签，标签落墙0；二层出现一个标签具有多个>=0.5m²连通分量，须结合楼梯逻辑标签及门户复核，不能直接宣称错误或全部正确。证据independent_topology_audit_20261014.json。没有改旧房间输出。
- 新探索仍在运行；不得把旧房间审计当作新探索的下游结果。新数据结束后必须重提墙并分房。

## 分层观测对照准备

- 新增summarize_trial_observations.py，对实际同步深度位姿逐层计算停留、路程、静止且零采样新增时长；表面点支持独立统计，避免跨层可见表面冒充进入该层。高度带只供离线评估，不反馈在线规划。
- 弃用基线统计per_floor_observation_assessment.json：一层内路程200.132m，二层155.162m，三层0；一层静止且零采样新增270.598s，二层58.191s。三层0个相机深度帧，只有611个从其他高度看到的表面体素，不能当三层完整覆盖。全程357.777m。
- Boxer进一步源码审查：GroundingDINO wrapper对不在prompts中的返回短语使用类别索引0，会无证据映射为词表首类。尚无当前40帧原始text_labels证明实际触发；待单独模型试验记录原始短语后评估，不因此编造识别改善或直接调框。

## 恢复版本已进入三层（实时未结束）

- live_height_assessment.json记录首次实际z>=5.5的帧及当前最高高度；最近第三层实际6.09–6.16m，仍满足scalar cap。进入不等于内部墙面及房间边界观测充分，局部斜顶仍需独立检查。
- 对Oct12历史round5逐帧独立审计historical_round5_stream_audit.json：2398帧RGB-D/pose/odom精确同步，最大深度位姿步长6.54995m，6个>1m步长，最高7.5852m、523个超过6.25m深度位姿。深度位姿路径451.714m与旧高频odom565.254m不同，今后同采样基准对比，禁止混用。
- stream auditor已补充位姿步长和header-stamp派生速度诊断；极高派生速度可能受桥接stamp调度影响，不能当硬件实时速度，但位姿跳变是明确的复核对象。

## 实测斜顶离线审计

- 新增audit_measured_sloped_roof.py：仅接受10–70度实测屋顶平面，排除7.96m平顶补面，只在距实测支持点XY<=0.15m处评估，无支持处标为未验证。输入完全隔离在线算法。
- 第八轮负例验证：1175个三层位姿，182个局部支持，118个在屋顶上方，最小垂直余量-1.1194m，符合此前34秒出界事实；993个无局部参考支持。
- 恢复运行实时检查当时741个三层位姿，只有23个有局部支持、718个无支持，已支持部分未检出越界。这个支持率太低，不足以确认全轨迹屋顶安全，需要原场景离线独立检查和本轮新实测屋顶联合验证。
- 三层近期出现同位置停留，实验尚未结束，暂不宣称完整覆盖或算法完成。

## Oct12恢复复跑整轮结果与导出异常处理

- 结束timeout600.014s、2997个Habitat帧，执行碰撞受阻0；2911个实际bag深度帧，记录跨度582.600s（启动等待已单独记录）。路径410.506m，最大相邻深度位姿步长0.89309m，>1m步长0，最高6.25m，已知体积1040.167m³。不是FINISH，不声称全覆盖。
- 精确同步审计发现5个深度帧缺同stamp的pose和odom；原stream_alignment_passed=false。严格导出31985因缺pose拒绝，不重写原bag、不用近邻姿态冒充同步。
- exporter新增显式--skip-unmatched-poses，仅用于排除无精确匹配的深度帧并逐stamp写入manifest。工具75327导出synchronized_export_matched，原失败空目录保留；下游提墙脚本只读matched目录。同步原记录问题仍需分析缺帧时间，不把过滤视为原流全部正常。
- 最后记录的frontier marker有许多平均高度6.3–7.6m，但平均高度不是可执行viewpoint；需检查实际候选及高度限制关系，不能据平均高度判断全部剩余目标不可达。final_frontier_marker_diagnostic.json保存原标记。

## 恢复运行新数据与新墙完成

- 精确匹配导出完成75327，2906个有效深度帧，40个RGB-D样例，1086354个4cm采样去重点。排除的5个stamp连续且都在记录起始约1秒，疑似录制器各topic连接建立时差；需要启动等待修正后验证，不能伪造原记录一致性。自由射线1464帧，manifest记录缺匹配情况。
- 新墙23770已正常结束：L1输入351751/墙104995/35片；L2输入544200/墙156131/76片；L3输入190403/墙28865/35片。全部来自新深度重提，未叠加旧墙；候选数量不是质量准确率。
- 新分层评估：一层内路径152.472m、零新增静止9.396s；二层193.735m、20.901s；三层62.942m、196.193s。第一个三层位姿相对精确匹配观测起点323.301s；末180s仅55个新增采样表面体素。
- 初步源码线索：旧sampleViewpoints3D只按map occupancy/box检查候选，没有fast_planner/max_flight_z约束；后续manager/Astar才拒绝高位姿。因此合法较低观察位姿可能在top-view选择时被高收益超高候选挤掉。仍待具体候选或小范围实验验证，未立即修改旧算法。
- 新图new_geometry_diagnostic.png已生成；生成脚本已纠正复制来的第三层不足固定标注，按实际是否提取墙面显示。后续还需要墙面/门洞/屋顶/补面的独立质量检查和新房间分割。

## 单项试验：视点高度约束提前应用

- 源码确认getTopViewpointsInfo只返回最高排序且够远的单个视点，sampleViewpoints3D却未用max_flight_z过滤；manager拒绝超高第一候选可能隐藏合法低位候选。不是依据frontier均值高度推断，而是实际选择路径的问题。
- 基于Oct12恢复版，只改frontier采样：原采样高度在上限内保留，上限与原圆柱采样范围相交时加一个上限高度样本；仍通过原FREE/UNKNOWN/occupied/可见性/收益门槛。没有提高飞行上限、放松净空、添加房间坐标或语义目标。
- patches/falcon/height_aware_viewpoint_candidates.patch保存可复现单项改动；outputs/exploration_height_aware_candidates_trial保留before/after哈希、配置与测试。恢复基线不覆盖。
- 从实际源码摘取采样段的C++控制检查已通过：边界纳入、无合法标准候选、扩展恢复候选、无上限时与原行为一致、广泛中心高度的上限不变量。编译工具25029，build.log。端到端覆盖改善仍待正常实时复跑，未选为最终版本。

- 单项试验编译25029已exit0，planner76919已启动。启动Habitat前已验证四个RGB/depth/pose/odom topic的trial_recorder订阅就绪，避免上轮最初5帧跨topic连接时差。正常实时复跑中，不重复启动。

## 新房间分割首次联合验证

- iterate_room_geometry.py新增--trial-dir/--output-dir：读取本轮geometry、新提墙点、精确匹配free rays和实际轨迹，不再固定旧视频素材；输入manifest记录来源。新试验只跑新实测墙方案，不重复16种大批量图。
- 68099已正常完成，输出rooms/S_新实测墙自由空间_L*.npz。每层6/11/7个几何标签，不是房间真值数量。
- 独立实际轨迹投影：一层21/649墙栅格命中，二层20/937，三层3/1251；位姿已标注比例87.83%/89.75%/27.58%。三层大部分轨迹停留在尚未获得room core标签的区域，不能说分割全对。
- 需要下一步核查墙面高度支持与门洞：墙点全高度投影会把门楣或高处结构阻断低位自由通路，轨迹落墙还需以本轮实际z和墙点z支持复核，不应直接删墙或盲目拓宽。窄连接区/楼梯需保留明确自由空间而独立分类，不用三层强制等面积制造成功。
- 单项探索试验99340/76919仍在运行，最近已到二层；当前房间诊断仍属于未加height-aware候选的恢复基线，不能冒充新试验结果。

## 门洞与高位墙投影的定因试验

- wall_projection_height_audit.json：三个楼层二维墙命中21/20/3个，但与新墙点三维距离<0.15m的样本均0；中位最近墙点距离0.471/0.401/0.493m。此证据排除了这些样本的近距离墙点碰撞支撑，不能据二维命中宣称实际穿墙；也不能直接证明全部是门楣。
- 新分割对照T_低位自由门洞证据：只在0.25–1.25m相对地面无低墙点邻域支撑、并有至少4个相对高度0.35–1.25m的精确深度射线帧通过时，解除高处墙投影对低位开口的阻断。保留有低墙支持的格子，未知区不因没有低墙点就直接删除墙。
- 新试验输出room_opening_trial，旧S/rooms不覆盖；每层L*_opening_evidence.npz保存原墙、低墙支持、低位自由射线次数及解除的格子，可逐项复核。工具91719正在计算；没有使用审计轨迹作为开门规则输入。
- 高度候选探索试验99340仍在运行，目前已进入三层；完整结果及屋顶安全仍待结束验证。

- 91719对照已完成：T轨迹标注比例98.77%/98.72%/99.84%，S为87.83%/89.75%/27.58%；二维墙命中降至8/9/2。轨迹覆盖标签提升只证明连通可达解释改善，不证明房间语义、漏房间或错合并解决。
- T二层标签从11变10，必须检查是否正确打通同房碎片或错合并相邻房；opening_invariants_and_merge_review.json保存旧新标签重叠及开口证据检查。只有低位支撑保留且至少4帧自由证据规则通过，尚未替换选定房间版本。

## U门洞自由空间与区域标签保留试验

- T门洞方案出现多个旧标签汇入同一新标签的风险，不能凭轨迹标注率改善选用。
- U保留S已有正标签，仅用同一低位自由门洞证据允许在domain内四邻域扩展到未标注区；没有固定房间坐标或将door连接强制合并为同房。已有S错误拆分仍可能保留，需要独立复核，不能叫确认真值。
- 67340对照已结束：仍6/11/7个标签，轨迹标注98.77%/98.72%/99.84%，二维墙命中8/9/2。label_preservation_audit.json逐格验证所有原正标签未改，墙内标签0。输出room_region_preservation_trial，旧房间输出保留。
- 当前结论只说明补全自由连接区且没有引入T的原标签合并；剩余错拆分、漏房间、家具误墙、跨层/楼梯及门洞位置仍需独立几何和视觉复核。
- 单项探索99340仍存活，三层同位置停留，尚未结束。不要因为短暂观察超时重启。

## 门洞局部复核与高度候选试验结束

- 对二层S6/7、三层S2/4与S6/7的公共开口生成同步RGB局部复核图portal_rgb_review.png。开口标记必须在实际opening格子内，并用对应深度判遮挡；前景遮挡或无有效深度的帧排除。仅作为离线人工几何复核，不将RGB语义输入提墙/分房。
- 深度遮挡过滤后，二层可用frame9，三层分别frame25、frame23/25；图像仍不能单独证明同房关系。U保留标签方案保留作为候选，T合并方案不直接选用。
- 高度候选单项试验结束timeout600.025s、2997帧、18帧执行受阻；未FINISH，当前停留[-4.741,3.04,6.25]。不能仅凭更早到三层选为更好版本。已启动immutable bag同步/高度审计与精确导出，工具句柄见本轮输出；需要统一分层、墙面和局部安全比较后决策。

## 高度候选试验不选用，回到恢复基线

- stream_audit.json明确同步与高度均通过，但深度位姿路径259.365m，最大跳变4.043m、12个>1m跳变，18受阻帧；恢复基线最大0.893m、0个>1m、0受阻。已知体积971.439m³也低于恢复基线1040.167m³。更短路径不能当作成功。
- 不将该变体替换选定探索。保留patch、源码、bag与失败配置；已把frontier_finder恢复为runtime/falcon_oct12_round5_checkpoint，增量编译日志restore_baseline_build.log。其余源码未改。
- 数据导出77406仍存活，保留用于各层观测与墙面支持的完整对照，不因观察超时重启。

## 单项探索拒绝依据补充与Boxer真实基线启动

- 77406导出已完成：2997个精确匹配深度帧、908528个4cm观测点，无排除帧。分层表面支撑13392/19182/12461个0.2m体素，恢复基线14220/23163/10151；一、二层减少，三层增加但零新增静止346.988s（恢复196.193s）。不能只用三层体素增加选用。
- 84726恢复原Oct12 frontier的编译exit0。变体保存为失败试验，不覆盖用户选择的恢复基线。
- 探索均已停止后才启动Boxer真实基线1841：40固定精确匹配帧，26固定室内类别，OWLv2、2D0.20/3D0.20，本地原权重，固定seed7。输出outputs/boxer/oct12_fixed26_20261014，日志boxer_fixed26_baseline.log。
- 新增run_seeded_boxer.py作为现有CLI包装；不改变检测框或后处理，只固定Python/NumPy/Torch抽样随机种子。无真值，后续只做人工样例/深度几何支持对照，不编造AP/IoU。

- Boxer真实OWLv2基线1841已正常exit0，40帧处理完成；evaluation_input_manifest.json固定输入manifest哈希、26类完整词表、阈值、抽样seed和权重身份。3D CSV只说明输出完成，尚未做人工/几何质量评估或宣称准确率。

## Boxer格式与实测深度支持审计

- 对照BoxerNet.to_obb与ObbCsvWriter2源码，CSV为世界中心、wxyz四元数、对象局部轴全尺寸，不是角点/半尺寸。audit_boxer_depth_support.py按原采集内参反投影深度，再转到每个OBB局部坐标，未平移/放缩预测框。
- 176行基线均有精确帧、格式有效；5行在0.15m内无采样深度点支撑：frame8 door、frame14 curtain/sofa、frame16 railing、frame17 bed。视野裁剪、遮挡、薄物体和模型偏移均可能影响支持，不直接当假阳性。输出depth_support_audit.json保留逐行指标，不声称IoU/AP。
- 启动仅改2D识别器的GDINO对照65667，相同40帧、26词表、阈值、seed和Boxer权重；未启用SAM2。GDINO固定缓存快照12bdfa3120f3e7ec7b434d90674b3396eccf88eb。
- run_seeded_boxer.py增加可选PRE_MAP_VLN_DETECTOR_AUDIT_PATH，逐帧记录原短语/映射类别/2D框/分数，不改预测结果。用于确认是否实际发生非词表短语误映射到第0类，不依靠猜测就修改类别。

- GDINO65667已exit0并完成独立深度支撑审计及label_mapping_summary.json；其模型效果仍需人工复核样例与同类别匹配对照，不能据输出数推断准确率。

## GroundingDINO类别映射定因修复

- 原对照真实触发10个保留复合短语（如stairs railing/table dresser）错误映射到首类door；这不是模型框偏移，而是类别接口错误。
- 修改仅发生在2D adapter：未知短语在NMS前明确拒绝并记录，不猜选复合短语中的某个类别；合法精确词表短语按其索引传给Boxer，不再有else0回退。3D模型、尺寸、中心和朝向未人为修改。
- 原adapter备份保存oct12_gdino_fixed26_20261014/grounding_dino_wrapper.before.py，可复现patch保存patches/boxer/strict_grounding_labels.patch；未删除或覆盖旧预测数据。
- 同一40帧/26类/seed7/阈值/权重的strict对照76036已启动，audit同时记录unmapped_detections和原短语。保留可能的召回损失作为明确限制，不能声称拒绝条目都是错误对象。

- strict76036已正常exit0；strict_mapping_validation.json对所有保留预测逐条验证原短语==映射类别且属于词表，未知复合短语记录为未匹配。未做同物体人工语义准确率声明，Boxer几何质量仍需逐样例检查。

## SAM2消融与深度审计范围纠正

- SAM2真实40帧对照90622正常exit0，2D提案逐行与strict GDINO完全相同，329个3D预测一一对应；中心移动中位0.015867m。没有人工改尺寸或平移框。
- sam2_geometry_ablation.json记录中心/尺寸变化，sam2_box_projection_review.png展示每帧最大变化样例。不能把中心移动或深度保留率当作更准。
- 重要审计纠正：audit_boxer_depth_support原固定<=5m，而实际HabitatLoader给Boxer所有正深度。远处门框会被漏计。修为默认所有有限正深度，可显式--max-depth-m重現旧截断。旧depth_support_audit.json保留，新增depth_support_full_range_audit.json，旧5/23无支持统计不得用于模型优劣结论。
[{"run": "oct12_fixed26_20261014", "rows": 176, "zero_full_range_support": 2}, {"run": "oct12_gdino_strict26_20261014", "rows": 329, "zero_full_range_support": 5}, {"run": "oct12_gdino_sam2_strict26_20261014", "rows": 329, "zero_full_range_support": 5}]

## 映射同步源码核对与物体融合支持溯源

- MapServer按image header stamp调用Transformer.lookupTransform，Transformer精确容差1ms或夹住时间戳的两姿态插值；源码未显示直接用最新pose。没有据此宣称运行已完全同步，实际映射interpolated_pose尚需记录验证。未擅自改映射源码。
- 新增fuse_boxer_with_view_audit.py：沿用官方当前fusion、同样IoU匹配0.2/至少4检测/conf0.45及seed7，附加每个实例原始检测数、精确帧stamp、0.25m相机位置bins和0.25m/15deg朝向bins。只注释重复视角风险，不靠删掉单视角实例制造准确率提升。
- 启动baseline OWLv2 fusion66842，输出owl_fixed26_fused.csv及.view_support.json；支持成员索引按fuser的confidence过滤顺序映射原stamp，缺精确采集帧直接拒绝。预测间IoU只是内部聚类规则，不是假造与真值IoU。

- OWLv2融合66842已exit0，只有shower/26检测与bathtub/13检测两个实例，各仅1个0.25m/15deg相机pose-bin；不能称多视角验证成功。40均匀采样包含大量末段停滞重复帧，同时运动阶段物体帧不足4检测门槛。
- 这个结果把Boxer部分问题定位到采集后关键帧选择/重复视角支持，而不是坐标轴或单位错误。当前40帧作为固定消融仍保留；后续应从同一bag的精确同步RGB-D按真实位姿变化选择有上限的运动关键帧，明确固定新评估集后与均匀采样比较，不降低所有框置信阈值或扩大框尺寸制造更多实例。

## 按真实运动选择Boxer关键帧

- export_synchronized_trial新增可选--episode-only和运动keyframe模式；默认旧均匀采样保持。只用精确匹配相机位姿，距上个候选平移>=0.3m或旋转>=15度才记录候选，再确定性均匀选最多240个候选。没有房间坐标、真值或生成航点。
- 从恢复基线同一immutable bag导出motion_keyframes_240，工具83019；显式排除原5个缺精确pose的初始深度帧，保留原40帧固定对照。episode-only不重复生成点云或视频，只输出模型所需同步RGB-D/pose与manifest。
- 新评估集固定后需保持26类、原权重/阈值和seed7，比较采样对融合视角支持的影响；新增框数或实例数不证明语义或定位更准确。

- 83019导出正常完成：从2906精确匹配帧生成1111个运动候选，确定性选240帧；motion240_evaluation_identity.json固定manifest哈希和起止stamp。原数据与40帧对照保留。
- OWLv2/Boxer motion240真实推理80297启动，26类/阈值0.20/0.20/seed7不变；本次只比较采样策略，输出oct12_owl_motion240_20261014，不把数据来源改变误说成模型升级。

- motion240推理80297已正常exit0，240帧全部处理、1191个2D/3D预测。沿用原fusion seed7/IoU0.2/min4/conf0.45的融合14581已启动；只以融合成员的实际pose-bin检查多视角支持，不用框数增加宣称质量改善。

- motion240融合14581已正常exit0，79个实例均有至少3个相机pose-bin，40均匀样本的2实例各1bin。只说明采样补上多视角支持，不证明79个都正确。后续必须核查错类、重复对象与融合框几何；已开始name/sem_id一致性审计。

## 融合名称/类别ID不一致已定因修复

- 79融合行中5个name/sem_id不一致，例如window/id5(cabinet)、dresser/id16(refrigerator)。原代码分别选择text多数票和张量argsort索引，后者没有按与文本关联的类别投票，混合类别簇可产生矛盾元数据。
- 改为(name,sem_id)联合多数票，等票按确定性标签/ID排序，不依赖Python set随机次序；未改匹配IoU、置信权重、尺寸、中心或朝向。原源文件备份fuse_3d_boxes.before_label_fix.py，可复现patch fusion_label_id_vote.patch。
- 同raw1191预测重融合21772输出owl_motion240_fused_label_fixed.csv，旧79行结果保留。fusion_label_fix_validation.json逐行检查词表名称与ID一致，并比较全部几何与prob字段，不能把接口一致性当语义准确率提升。

## 实际执行速度未发布：已用bag证实

- odom_velocity_consistency_audit.json：恢复基线20380条odom速度全部0，但1741个相邻位置区间移动>1cm。这是明确输入状态不一致，不能仅因pose/depth stamp匹配就称运动状态正确。
- run_habitat_falcon新增可单独启用--publish-executed-velocity，按同帧实际传感器中心位置差/真实monotonic时间差发布FALCON世界轴速度，放入同步bundle。原file_bridge已有读取该字段并传给FALCON的逻辑，默认旧行为保留供控制比较。
- helper验证已通过初始静止、world轴/米每秒、受阻停留0速度、真实跳变不被截断掩盖和非正时间拒绝；证据executed_velocity_helper_validation.json。没有改clock、加速或暂停握手。
- 真实跟随器仍直接跟踪desired PositionCommand，可能产生超过动力学限制的速度；因此新速度字段不能声称解决跳变或探索停滞。下一步需受控验证正常动态规划对实际速度的响应，不能盲目把异常速度裁成上限或直接宣称全部修复。

## 120s实际速度发布单项试验启动

- outputs/exploration_executed_velocity_smoke，原Oct12算法/配置、正常实时5Hz、planner yaw、6.25m及碰撞检查不变；只启用--publish-executed-velocity。120秒为输入与动态状态验证预算，不是探索完成依据，不生成最终视频。
- planner5840已启动，已验证四topic录制订阅后再启动Habitat。工具句柄见本轮输出；先核实进程再继续，不因观察超时重启。
- 需检查实际速度与位姿差是否一致、计划发布/受阻/轨迹跳变及相应动力学限制；任何异常速度保留不截断，不据辅助函数测试就认定效果改善。

- 120s速度单项试验5840/75211当前仍存活。约286帧桥接记录已有243帧非零速度，最大2.853m/s，高于原planner max_linear_velocity2m/s。说明新字段反映了执行器的真实异常，而非裁成合规值；需完整bag核对，并不能因此宣称动力学可执行。

- 速度发布试验已结束timeout120.129s/600帧/0碰撞受阻，进入二层。120s只属输入与规划响应试验，不是完整探索完成。5840/75211均已退出，已启动stream/velocity response审计，工具句柄见本轮输出。

## 实际速度审计完成与执行接口对照

- 120s原执行器4224条odom中3466条非零，最大3.76438m/s，693条超过2.01m/s；600深度帧精确同步，无>1m相邻跳变。不能声称满足2m/s动力学限制。
- 新增默认关闭的--max-execution-speed：只限制真实执行位移，使用上帧观测到当前执行的真实时间，随后仍按实际观测差发布未经截断的速度。保留原直接跟随基线，不改Oct12探索源码；不保证加速度可行。
- exploration_bounded_execution_smoke为相同120s/seed7/5Hz对照，仅在速度发布试验上增加2m/s执行位移上限，碰撞检查/高度上限/yaw不变。

- 执行上限对照50608/47992正常exit0：120.122s超时，600帧全部RGB-D/pose/odom精确匹配，0受阻，路径129.949m，z0.285–4.770，最大相邻位移0.396m。实际发布速度最大1.977534m/s，超过2.01m/s的匹配帧为0。
- 尚不可选为完整修复：按ROS header stamp反算最大3.948004m/s，与发布速度最大残差1.974481m/s。原桥接源码file_bridge_node.py:270使用接收时rospy.Time.now，而模拟器速度使用观测monotonic时间；当前bundle无采集时间字段。已定位两种时间基准不同，须实测采集/接收间隔与桥接延迟，不能假定差异全部来自传输，也不能只看发布速度便宣称动力学合规。
- 原Oct12算法和默认直接执行未替换。后续先记录同帧采集时间与桥接接收时间，验证时间差后再决定如何保持ROS映射与实际速度时间一致；未启动下一场重型试验，11314实验进程已退出。

## 采集/接收时间差已实测定因

- 前一goal turn属于progress：完成执行器受控对照并发现新时间一致性问题。本轮核对当前源码、实验均已退出，磁盘余36GB，未并发重型任务。
- exploration_frame_timing_smoke工具44361/49961正常退出，150帧无丢帧。采集真实间隔0.188–0.211s；接收间隔0.097–0.304s；延迟4.7–111ms。采集时间反算速度与发布速度最大差2.22e-5m/s，接收时间反算最大差1.9002m/s。证据frame_timing_summary.json。
- 新增audit_frame_timing.py可复现审计；采集端--audit-frame-timing只附加同帧wall/monotonic时间，桥接条件记录frame_timing_audit.jsonl；没有共享模拟时钟、暂停或倍速。
- 新增默认关闭--publish-capture-timestamps，以同帧采集wall时间为RGB-D/pose/odom stamp。该模式下停止给旧pose重新打当前时间，按5Hz完整同帧输入；显式拒绝use_sim_time。原接收时间策略仍是默认。
- exploration_capture_stamp_smoke为30s接口验证，不属完整探索或覆盖比较；工具53323/25367，需核实存活和完成后审计。

- 采集stamp短时对照53323/25367已exit0，150帧四topic逐帧精确匹配，无受阻。ROS采集stamp反算最大速度1.973294m/s，发布最大1.973277m/s，速度最大残差2.2208e-5m/s；采集stamp消除已确认的接收抖动假象。仍有规划碰撞拒绝和局部时间下界日志，未视为成功或失败的唯一判断。
- 下一完整候选outputs/exploration_capture_stamp_full_trial：同一Oct12算法、600s正常实时/seed7/5Hz/同yaw/cap/collision；启用真实速度、2m/s实际位移限制和采集stamp。非最终选定，必须比较三层覆盖/停滞/墙观测并重新提墙及下游验证。

- 完整候选planner句柄2269已启动并验证四topic订阅；simulator启动句柄见本轮工具输出。继续前应先poll这些句柄，禁止因观察超时重启。
- simulator句柄96282已poll确认仍运行；planner2269。本轮未标记目标完成，完整候选尚无结果。

## 完整探索运行期间的独立下游审计

- 96282/2269本轮poll确认存活，不重启。新增audit_live_mapping_pose.py，直接比较MapServer实际发布interpolated_pose与精确stamp光学pose，60s窗口句柄45974；此窗口不是全程证明。
- Boxer原motion240数据不变，新增融合成员类别溯源；重融合14067正常exit0，全部79行几何/元数据与label_fixed逐字段完全相同，31簇包含多类别、7簇所选类支持比例<0.6。对应fusion_member_review_summary.json及fusion_class_review.png，不能当语义已修复。
- 复核图看到门窗重叠提案、浴室镜子/画的混淆、餐桌椅附近railing/stove误提案。图中同类提案只是上下文，并非逐实例对应证明；没有据此手改预测类别或造准确率。
- 官方Boxer论文/实现将单帧提升与多视角融合分开：https://arxiv.org/abs/2604.05212 和 https://github.com/facebookresearch/boxer 。结合本次真实混合簇证据，应重点核对跨物体关联与语义不确定性，不用简单类别ID一致替代语义验证。
- 提墙迁入可复跑scripts/extract_trial_walls.py：显式trial-dir/export/settings，记录点云及配置SHA256，使用与恢复基线同样AE设置；拒绝覆盖旧提取，低观测层输出空墙并标记不足。完整候选结束后方可在新观测上运行。

- 映射窗口45974已exit0：301 source stamps /191 mapped stamps，190精确匹配的position/光学orientation误差均为0；1个mapped缺source和111个source缺mapped仅属窗口差异，不能断言全部丢帧或全程同步。源码depthCallback在map_update_counter>concurrent_depth_input_max时直接return，当前阈值1、skip_pixel4、verbosefalse；尚无分支计数证明缺帧的具体原因，未改运行中的映射源码或提高并发。
- 完整试验identity保存候选源代码/配置/运行命令哈希、固定条件和接口差异，当前仍未选为最终。下一步poll96282/2269；结束后先stream和timing审计，再精确导出、分层观察对照、新提墙与房间检查。

## 候选运行期间房间显示与观察证据

- 本轮开始96282/2269两句柄均poll确认存活，上轮属于progress（映射窗口和融合复核实测证据），未启动重复探索。
- 新增render_trial_room_review.py，按当前指定白底/实际灰点/加粗红墙叠加现有候选labels，不修改任何分割。恢复基线U图measured_room_review_white.png已生成并查看，作为诊断，不是最终房间验证或视频。
- 候选约1987条实际帧状态时，zmax6.25，>=5.5相机帧42，发布速度最大1.97770m/s；短暂进入三层后返回二层。途中状态不能证明完整三层覆盖，必须等terminal bag再作固定采样对照。

- 完整候选96282/2269已exit0，超时600.013s/2997帧；四topic2997逐帧精确匹配，速度反算误差最大0.000438m/s，实际最大速度1.97770m/s，路径419.512m，zmax6.25。814受阻、1330个<1cm相邻区间，末段再次停留；不能替换0受阻的恢复基线，不因同步修复便称整体改进。
- baseline_comparison.json保存实际新旧指标；新精确点云导出57624已启动，结束后按同AE参数提墙。候选是失败/局限证据，不删除，原算法继续保留。

- 导出57624已exit0：2997帧全部精确同步，无需排除初始帧，1042751个4cm实际深度点，固定40帧保留。分层0.2m表面支持（基线→候选）L1 14220→13637、L2 23163→23309、L3 10151→9000，尚不能声称整体观测更好。末段实际静止156.551s，最近执行contact在[-4.112915,14.451726,6.130000]，距传感器约0.121m，需核对接触与planner clearance状态，不能宣称斜顶无风险。
- 新提墙1513已启动，使用tracked extract_trial_walls.py同配置，新点云独立提取；下轮先poll1513，不重复启动或叠加旧墙。
- 新提墙1513已exit0，geometry.npz与三层新墙均生成，manifest/provenance JSON解析正常。下一轮先查看新墙并复跑精确自由射线及房间审计，再判断是否保留该候选仅作同步接口证据。

## 候选新墙/房间与实际受阻的联合诊断

- 本轮前一turn属于progress：完整候选与新墙均实际完成，已核对文件而非重跑。候选自身自由射线55610正常exit0，1511帧精确pose；房间18253正常exit0，white复核图11531正常exit0并查看。
- 三层U候选region数7/10/6 vs基线6/11/7；独立topology无labeled wall overlap、无>0.5m²多组件，不能推导房间正确。轨迹落label比例L1从98.77%降到95.90%；候选三层北侧观测明显缺失，南侧多一些，不能用墙片数41>35宣布改善。
- 最终RGB显示床面附近的低高度视点；contact下方0.12m，最近采样深度距contact0.159m，不足以凭PCA确定屋顶/家具。final_contact_depth_support.json明确不作语义归因。
- 新增可复跑audit_stall_egress.py，离线同场景/同repair碰撞，仅重放已执行接触、0.18m guard/起点grace并强制6.25m cap，不注入航点。51366 exit0：26方向中3个满足0.3m swept-check+height+contact guard；实际ESDF/occupancy未保存，未证明规划地图可达。不宜仅归因contact把所有方向封死。
- 最末日志仍在持续规划，报告No finite frontier candidate or reachable HGrid relocation center；并非停在已完成FINISH。下一步需核对停滞时实际ESDF/占据及HGrid候选可达性，不继续盲调速度或高度。
- spatial_observation_comparison.json补充固定0.2m实测表面的逐层空间重合/遗漏，而非只比点数。候选不替换恢复基线，整体流水线仍未完成。

## 固定240帧识别器对照

- 前一turn属于progress：新数据房间审计与离线egress完成，未重启失败探索或改旧算法。当前无探索重型进程，GPU空闲15GB/磁盘35GB。
- 76359启动strict GDINO/Boxer motion240真实推理：与OWL240同一manifest哈希78b337...、同26类/0.20阈值/seed7/Boxer权重、无SAM2，只替换2D detector；evaluation_identity.json和run_inference.sh固定来源。
- 固定复核6帧（166/170浴室、95/99玻璃面板、14/63餐桌椅）在结果完成前写入fixed_review_cases.json，来源此前OWL混合簇复核；没有GT框，不编造AP/IoU。完成后需同帧比较2D类别及3D几何支持，不能以输出数量选择模型。

- strict GDINO240推理76359已exit0，240帧全部处理，2163raw 3D框。fusion92956、全范围深度41883均已exit0；2163行格式合法且全部精确匹配输入帧，40行无0.15m几何支持，不据此当准确率。
- 同帧6样例图fixed_detector_comparison.png已查看：餐桌椅提案更细，railing误提案仍在；浴室画166漏检/170误标window，门窗混淆仍在。两套模型均保留，未声称GDINO整体更好；comparison_findings.json记录类别映射与混合簇统计。
- OWL240同全范围深度审计已启动，句柄见本轮输出；必须同范围对照，不能与旧40帧或<=5m统计比较。
- OWL240同范围审计54074已exit0；same_range_support_comparison.json保存同240精确帧/同深度范围证据，不能把背景支持当真值或按框数改选模型。当前本轮任务均已结束，无重型实验存活。

## Boxer跨物体融合链已精确定位并做受控修复

- 前一turn属于progress：固定240帧两识别器真实推理/同范围支持/样例复核全部完成。本轮源码确认fuser按3D IoU图连通分量关联，传递链可以把同帧独立物体并成一个簇。
- 新增原CSV row索引溯源及精确2D/3D逐行stamp+name断言。63000重融合exit0；audit_fusion_association.py找到79簇中6簇同帧成员提案IoU<0.1。fusion_exact_member_review.png显示相邻窗扇、玻璃面板/柜门被并在中间，不再只用同类提案上下文猜关联。
- 新增默认关闭--separate-same-frame-proposals及fusion_frame_constraints.py：同帧低重叠提案不能在跨帧传递链最终同簇；按真实3D IoU降序确定性合并，不改原预测框、融合权重、置信门槛或min4。传递端点cannot-link、无约束连通和稀疏/稠密一致性验证通过。
- 初次对照76466 exit0：6簇拆分，35个graph merge拒绝；原柜子58被拆成不足4支持导致输出丢失，柜子16丢3成员，故未直接采用。
- 新增--preserve-under-supported-splits：有子簇不足原min4时保留原整簇并记录未解决歧义，不降低门槛。19289 exit0：只拆原12/23/37/74四个玻璃面板簇，所有原79簇的支持成员全部保留，75簇几何和prob逐字段差0；新9子簇有4–8检测/2–5实际pose bins。结果84框不能当准确率改善依据，仍保留2个柜子关联问题。
- frame_constraints_preserved_validation.json保存成员/几何不变性，fusion_frame_constraints_preserved_comparison.png展示原绿框/候选橙框；候选未选为最终，接下来检查拆分后的3D多视角投影与深度支撑、其他楼层/对象，再决定是否进入全流水线。

## 融合拆分后的逐成员多视角验证

- 前一turn属于progress：精确关联定因及受控融合/保留策略均真实运行；本轮读取现有结果，未重复推理或探索。新增audit_fusion_multiview_support.py在全部45个变化成员同帧RGB-D与精确2D提案上比较原父框和新子框。
- 中心投影落在对应原提案内由24/45→45/45，中位归一化中心偏差0.210458→0.032551；两版0个提案范围深度支持为零。仅为提案对应/深度证据，不是GT IoU或准确率。
- 提案范围近框深度点35视角增加、5减少、5相同；减少包含child13/frame70 3169→2959、25/frame80 4001→3887、40/frame99 2629→1993、41/frame95 1175→1172、78/frame190 2137→1995。frame_constraints_remaining_review.json保留全部5视角，不能只报告增加。玻璃背景可能影响这项指标。
- frame_constraints_multiview_review.png已生成并检查9个子簇各首末视角：相邻窗扇和玻璃面板分开后对应位置更稳定，原中间框问题改善；柜子16/58仍未解决，名称及完整3D尺寸/朝向准确率仍未证明。候选保留用于后续联合评估，不宣布整个Boxer或流水线完成。
- 探索的实际ESDF/occupancy停滞快照仍缺失，原save_map_pcd硬编码/home/eason/map.pcd且只导占据，testESDFCallback硬编码z1，不能用这两接口伪证三层停滞可达性。后续应作默认关闭的原始规划状态诊断，不再盲调高度或执行速度。

## 恢复算法的实际规划状态诊断

- 前一turn属于progress：45支持视角/9子簇多视角几何审计完成。本轮核对容器探索进程均已停止，源码SHA256 8625cc52...与恢复基线一致。
- 原testESDF强制z1/save_map硬编码外部路径不适用三层诊断。新增stall_planning_state_diagnostic.patch及runtime/falcon_oct12_diagnostic_checkpoint：仅在no-finite-frontier/relocation失败、参数stall_diagnostic_path非空时记录，每10s最多一次；记录实际pos/vel、FREE/UNKNOWN枚举、260个局部occupancy/ESDF采样、全部frontier候选及temporary block标记，不进行路线查询/目标注入/地图更新。原算法/安全阈值保留。
- 初次共享runtime路径传入失败，已核对docker实际仅挂载outputs/scripts等，改用docker cp传入，未改变权限或删除文件。22346编译当前在运行，须poll确认terminal后才能启动新实验。
- 新目录exploration_oct12_stall_state_trial沿用原恢复基线600s/seed7/5Hz/yaw/cap/collision，同原直接执行/旧速度和接收stamp策略；只启用规划状态诊断，避免把此前失败执行候选混入算法定位。

- 22346编译已exit0。新诊断试验planner12663/模拟器40941已启动并确认四topic录制订阅。当前只跑这一重型任务。原Oct12快照未覆盖，容器运行其只增加诊断的派生源码。
- 新增summarize_stall_planning_state.py，以实际枚举与ESDF值报告离散0.3m局部可行方向和候选封禁/占据分布，明确不包括完整AStar/HGrid路线或contact约束；不把局部采样通过当完整可达证明。下一轮先poll40941/12663；若仍运行继续同句柄，不因超时重启。

## 中断后真实进程核查

- 上轮属于progress：实测深度/相机above证据推断三个楼面候选z=-0.15477/2.59949/5.59510m，面积71.325/89.2575/46.98m²，infer_observed_floor_planes.py27590已exit0；非scene truth，不证明楼面语义全正确。
- 本轮40941/12663句柄缺失，host模拟器确实不存在；docker自己的roslaunch142232及录制142267仍存活，不能直接当整个试验terminal。模拟记录约1310帧停止，无result与stall state。仅SIGINT自己的142232保全bag，旧11312进程未碰。interruption_report.json明确不完整，不宣称600s完成，也不在同目录重启。
- 中断bag已正常落盘607MB，interrupted_stream_audit完成；采集跨度262.89868521700004s，depth帧1314。此数据只能用于中断前运动/同步核对，缺少停滞失败快照，尚未定位最终不可达根因。

## 中断诊断的独立复跑

- 前一turn属于progress：关闭自己的残留launch、保全并审计中断bag。本轮核实无exploration_node/record存活，但旧监控142190仍存活，已仅TERM这个旧监控。容器源码SHA2569541da85...与诊断快照完全一致。
- 新目录exploration_oct12_stall_state_retry，旧中断数据未覆盖；原恢复算法配置/seed/yaw/cap/collision/正常实时600s不变，仅可选状态记录。planner35342与simulator92116已启动并验证四topic录制订阅，trial_manifest保存哈希和句柄。继续时先poll这两个句柄及实际进程，不因观察超时重启。

## 实测楼面高度的房间分割消融

- 本轮开始92116/35342 poll确认运行，无重复探索；前一turn属于progress（新目录恢复诊断运行并清理已终止试验的旧监控）。
- iterate_room_geometry新增默认关闭--observed-floor-planes，只接受三个有限/相距>1m实测大面候选，将ray楼层划分阈值设为测量floor height；仅离线分割，不输入在线planner。默认历史配置不变。input_manifest记录证据和实际阈值。
- 2559新消融exit0，rooms_observed_floor_trial保存独立输出。U三层6/11/7标签与原结果逐格完全相同，domain L2变化1格，旧label未损失；独立topology无墙重叠/大组件断裂。floor_evidence_ablation.json保留逐区域转换，不将一致性当房间正确证明。
- 实测楼面z=-0.15477/2.59949/5.59510m与原ground近似，但ray阈值原L2/L3为2.75/5.55m，候选直接用实测ground，当前未改变labels。楼梯/跨层接口仍需局部3D证据检查，不能按平面面积等值通过判全部正确。

## 墙面来源与倾斜几何独立核查

- 本轮92116/35342 poll均确认仍运行，当前约540帧且无failure snapshot，不重复启动。前一turn属于progress（实测楼面分割消融完成）。
- 新增audit_trial_wall_geometry.py，用每层实际观测点及法向/ceiling集合独立检查输出墙面来源。恢复基线及完整执行候选两组真实墙审计85401已运行：所有墙点均精确来自各自深度几何，来源距离0；无abs(normal_z)>0.35点，无墙/ceiling完全重复点。
- 这证明未生成无源墙点且选中法向符合竖直条件，不证明墙全对或家具已排净；距roof<0.1m点单独报告，因为可能为真实墙顶连接，不机械删除。两个试验independent_wall_geometry_audit.json保留逐层结果。停滞根因仍需本轮planner snapshot。
- 继续poll92116/35342：约1090帧，ROS参数/fast_planner/stall_diagnostic_path已由rosparam确认实际加载，快照仍0；诊断分支尚未触发。无改变配置。
- 继续poll92116/35342：约2200帧，位置在[-2.65,~5.0,3.85]附近往返，run.log持续Total time too long，stall_planning_state仍0；实际停滞是轨迹/重规划循环，未进入候选空集分支。继续保留复跑。

## 规划超时循环诊断复跑完成

- 诊断复跑92116/35342正常结束，600.036s/2997帧；同步审计通过：RGB/depth/pose/odom各2997，零缺精确stamp，z0.223–6.25。stream_audit.json已保存。
- 诊断快照0，但不是插桩失效：参数实际加载，run.log明确候选持续非空；错误模式为反复Total time too long与Short path preserving waypoint/optimizer skipped，而非No finite frontier。
- 末段轨迹最终[-2.65,5.55,3.85]附近循环，实际路径430.477m，1208个<1cm间隔；最终execution诊断标记nonempty-frontier/replanning-timeout。发布速度仍旧legacy zero，接收stamp策略不变，故本试验只诊断恢复基线，不把速度审计混入。
- collision_blocked_frames=0、timeout而非FINISH；不能宣称探索完成。最高stamp反算速度9.99996m/s来自接收时pose stamp抖动，已由之前采集stamp实验定因，不作动力学结论。
- 当前最有证据的探索失败模式已从“候选可能为空”收窄为“非空候选+短路径保留+轨迹总时长超限的重规划循环”。下一步应针对该规划/轨迹接口做小范围源码审计或离线日志统计，不能继续盲调高度。

## Short path保留逻辑的源码定位

- 本轮完成源码审计而非重复探索：planner_manager.cpp第约212行在三点A*路径总长<1m时直接跳过B-spline优化并保留角点；planTrajToView仍要求yaw time_lb，日志中Short path与Total time too long相邻出现。
- 新增未激活runtime/falcon_oct12_optional_shortpath_checkpoint/planner_manager.cpp及patches/falcon/optional_shortpath_optimizer.patch：参数/fast_planner/preserve_short_waypoint默认true，完全保持基线；只有显式false才允许短路径进入优化。manifest标记需运行消融、未选最终。恢复基线源已恢复原样，未污染。
- 当前下一项应是独立短时/低资源消融比较轨迹超时、短路径循环和碰撞，不能只因跳过优化便宣称修复；高度/速度配置保持不变。

## 短路径优化开关60秒消融

- 可选源码正确复制并编译，默认基线源未修改。60s消融将/fast_planner/preserve_short_waypoint=false，其余Oct12 seed7/5Hz/yaw/cap/collision/接收stamp完全不变。
- 消融35670/21281正常结束60.1s/300帧，RGB-D-pose-odom精确同步，路径60.628m，1次模拟器几何受阻，未作完整探索结论。run.log中Short path/Total time too long联合次数41；600s恢复诊断基线对应500次，时长不同且不能直接归一化，初步显示开关可能减少反复但需同预算更长对照。
- stream_audit.json记录高度与同步；该候选未选最终，不能因短时错误数减少宣称修复。下一步应在独立60/120s相同配置再复核末段停滞、轨迹连续性和碰撞，再决定是否进入完整600s。

## 短路径优化120秒复核

- 120s消融编译和运行完成；同配置仅preserve_short_waypoint=false。600帧精确同步、路径129.639m、低位移间隔54、0碰撞受阻，zmax1.961m，尚未到三层；timeout不是探索完成。
- 120s run.log短路径保留0次、Total time too long 65次；60s消融为短路径41次、总时长日志需结合相同窗口，不能以日志减少单独宣称修复。120s没有末段停滞到三层证据，候选不进入600s前仍需覆盖/层切换风险检查。
- ablation_summary.json保存可复现统计。默认基线和原恢复算法未替换，短路径优化仍是未选消融。

## 短路径优化600秒候选的联合几何复核

- 完整消融5045已结束，审计通过：2997帧精确同步，路径527.963m，known volume1074.425m³，低位移759，8受阻，zmax6.25，timeout；Short path日志0、Total time too long367。
- 分层表面支持（恢复基线→候选）：L1 14220→13399，L2 23163→23767，L3 10151→12750；候选三层观测显著增加但一层减少、路径更长且有8受阻。候选不是自动最佳，per_floor_comparison.json保存折中。
- 候选导出44458已完成：2997精确帧、1145129点；新墙提取87961完成，manifest记录每层source和设置；独立墙审计8006进行中/结果写入。候选墙不能仅凭patch数量选择，必须检查受阻点/斜顶和房间。
- 由于候选改善三层表面但损害一层并引入受阻，暂不替换恢复基线；后续需对候选新墙和房间做同一U分割审计，若三层任务权重明确再考虑保留为“覆盖优先”候选。

- 候选完整消融导出/墙审计完成：episode 40帧，墙L1 94271/39、L2 159526/74、L3 46016/45；独立审计显示墙点均精确来自候选深度、竖直法向筛选通过，质量仍未证明。候选free rays 1515帧。
- 候选房间脚本当前固定读取trial_dir/synchronized_export_matched，候选导出使用final目录；本轮未复制约GB数据或改旧目录，room_joint_audit未运行，避免产生错误对照。候选已明确保留为覆盖优先实验，不冒充完整房间联合验证。

## 候选短路径优化的房间分割审计

- iterate_room_geometry新增--export-name，默认synchronized_export_matched保持兼容；候选final导出可显式复用同一算法。7717正常exit0，U候选区域数L1/L2/L3=7/10/9，轨迹标记比例0.9790/0.99745/1.0，墙命中比例0.00525/0.00153/0。
- 相比恢复基线U=6/11/7、label比例约0.9877/0.9872/0.9984，候选有跨层区域数量变化且二层更完整、三层多分区，但不证明真房间；独立topology无墙重叠/大组件断裂。baseline_comparison.json保存对照。
- 候选现在具备同步点云/新墙/自由射线/房间候选全套离线证据，但仍timeout、8受阻、墙和房间语义未验证，暂不替换恢复基线。

- 候选短路径600s新增斜顶离线审计：从候选L1–L3新墙manifest提取4个10–70°斜面，L3轨迹601帧中47帧有局部XY支持、554帧无支持；有支持帧最小垂直间隙0.8505m、法向间隙0.5533m、0帧高于测量斜面。该结果仅证明有支持局部安全参考，不能把554无支持帧当安全，也不证明全建筑 containment。

## 联合流水线证据报告

- 新增outputs/PIPELINE_JOINT_EVIDENCE_REPORT.json，逐项列出同步、探索完成、高度/屋顶、墙、房间、Boxer、可视化/复现证据及状态。明确selected baseline、coverage-priority candidate和四项必需未完成工作；full_pipeline_complete=false。

- 候选完整消融固定40帧已运行同OWL/Boxer（88796）并完成全深度支持审计：345行3D框，格式合法/输入stamp全匹配，8行无0.15m深度支持。与恢复基线40帧不能做性能比较，因为两者选取帧manifest不同；comparison_summary明确禁止按框数或支持数宣称更好。候选下游链路可运行但仍非最终。

- 最终证据一致性检查发现联合报告中的OWL深度审计路径多写了episode一级；已修正为outputs/boxer/oct12_owl_motion240_20261014/depth_support_full_range_audit.json。修正后报告引用文件全部存在，git diff --check通过；磁盘剩余约31GB，未启动新重型任务。

## 选定基线三视角诊断视频

- 新增scripts/render_selected_three_view_video.py，使用选定Oct12固定同步40帧、实际灰色点云/红色新墙/蓝色轨迹生成outputs/exploration_oct12_restored_trial/selected_three_view_diagnostic.mp4。该视频是可复现诊断展示，不改变算法或宣称覆盖完成。
- 联合报告visuals_and_reproducibility状态已更新；最终语义/房间验证仍未完成。


## 用户要求清理无用改动（2026-10-14）
- 删除未启用且已明确拒绝的八月基线工程副本 runtime/falcon_success_baseline/（873 个 Git 未跟踪文件）。
- 删除淘汰的 height_aware_viewpoint_candidates.patch 和空的 optional_shortpath_optimizer.patch；上文相关路径为历史记录，文件已清理。
- 保留 Oct12/13 成功基线、实验输出、原有 bridge_real_pcd 用户数据，以及现有诊断与候选源码以便复现。此次仅清理文件，不代表运行中算法已回退。


## 用户指定基线更正与重跑（2026-10-14）
- 唯一指定基线为 exploration_recovery_local_trial/round6_extended（原始约720秒，视频缓存preview_trimmed）。277fcf7为后续保存提交，不是录制前固定版本号；规划yaw第一视角是离线重渲染，点云来自原始观测。此前带6.25m cap的round5恢复版本不能视为该基线。
- 恢复manager_stable、astar_contacts、planner_contacts、frontier_before及对应配置；采集脚本和桥接恢复277fcf7，使用历史默认朝向。原启动命令未保存，该朝向是历史脚本和离线渲染来源推断，并非逐参数复现证明。
- outputs/exploration_round6_extended_rerun_20261014已完成720.12秒实时重跑、3597模拟器帧，148防护拦截，timeout；录包3574帧、715.30秒，全部录到的深度精确匹配RGB/pose/odom，23模拟器帧不在bag内。已知体积1127.852m³，原记录1115.290m³；同口径路径见assessment.json。仍有位置跳变、零速度反馈等历史限制；不宣称完整探索或稳定飞行。
- 600秒outputs/exploration_oct12_baseline_rerun_20261014明确标为非用户指定基线，不混入round6对照。


## 默认视频形式恢复（2026-10-14）
- 用户要求默认使用前天原版。render_exploration_three_view.py保留导入工具函数，main改为调用current_video_snapshot原版：无字、左大俯视、右上全局、右下第一视角、扇形视野、三层渐变、最近15秒轨迹，默认40秒30fps。
- 删除render_selected_three_view_video.py及本次通用预览派生脚本。首次提供的左RGB布局视频不作为默认。
- 输入必须使用本次cache、geometry和离线yaw素材，缺失时报错，不借用历史观测。


## 跨层代价单参数候选（2026-10-14）
- 用户满意round6重跑总体轨迹，目标为探索效率。保留满意基线，新增exploration_round6_cross_floor_cost30_trial，仅floor_crossing_penalty18→30；源码、执行方式、场景、起点、720秒条件保持一致。参数差异已检查，候选未运行，未宣称更优。
- 二楼约603–607秒短暂停留日志显示路径搜索回退、规划耗时1.279–2.520秒，再恢复EXEC_TRAJ；与末段111秒持续失败不同。
- 跨层惩罚仅影响存在可竞争本层可达候选时的选择，没有本层可达候选仍会跨层，不保证避免原513秒下楼。


## 跨层代价30试跑完成
- exploration_round6_cross_floor_cost30_trial完成720.12秒，3597帧全部RGB/depth/pose/odom精确匹配，timeout。七个运行源码哈希与满意基线一致，仅跨层代价18→30。
- 已知体积1042.970m³（基线1127.852），路径376.030m（基线516.912）；跨层轨迹1→2→3，没有回二楼，但三楼长时间受阻，拦截1617帧（基线148）。
- 不采用代价30，保留代价18的满意基线。单次实时运行不足以将差异全部归因于参数；少跨层和短路径不能独立证明效率改善。相同覆盖量耗时、覆盖增长与静止区间见baseline_comparison.json。


## 受阻目标20秒避让优化完成
- 候选exploration_round6_blocked_goal_recovery_trial基于满意代价18基线，仅FSM实际受阻回调新增默认关闭的block_scene_failed_viewpoint参数；启用时blockViewpoint执行目标20秒、清除目标有效标志，再按原逻辑停止/重规划。正常路径、净空和碰撞检查未修改。补丁scene_blocked_viewpoint_cooldown.patch，源码及哈希保存在候选目录。
- 编译通过，720.13秒实时3597帧完整录包同步通过，避让实际触发10次，timeout。已知体积1164.973（基线1127.852，+3.29%），路径456.279（基线516.912，-11.73%），1→2→3无回二楼。防护拦截179帧（基线148），仍有位置跳变等历史限制。
- 保留满意基线；新候选作为下一轮评估对象。单次实时运行是初步证据，未证明稳定收益/完整房间覆盖或FINISH。覆盖目标耗时与停滞详见baseline_comparison.json。


## 原始采集yaw一致性核验
- 使用原始video_cache/camera_pose.npy（非离线yaw_matched）、同采集时刻最近pos_cmd（age≤.25s）进行800样本审计。满意基线797有效样本：误差中位87.37°，67.38%>45°；受阻避让候选797样本：中位89.47°，73.27%>45°，前15秒14有效样本全部>45°。明细acquisition_yaw_consistency.json。
- 原runner未启用use-planner-yaw，移动时朝实际位移方向，停止不执行规划yaw，idle_scan_rate默认0。数学坐标变换验证planner yaw0对应FALCON+X，不存在固定90°坐标偏移；大偏差符合两套朝向策略而非仅坐标错误。
- 这些是采样指令对齐证据，不单独证明出生角落漏观测的完整因果链；离线yaw重渲染不生成原始深度观测，不能用于证明探索时看过目标。应优先单独验证真实采集与规划yaw一致，再判断前沿生命周期策略；当前地图依赖候选未选用。


## 实际采集规划yaw一致性修复与重测
- exploration_yaw_consistent_trial沿用受阻避让候选，唯一实验变化为实际采集启用use-planner-yaw。720.14秒正常实时完成3597帧，全部RGB/depth/pose/odom精确对齐。timeout，不是探索完成。
- 797有效采样：yaw误差中位0.105°、P95约7.958°、>45°为0，前15秒中位1.785°。残余偏差包含指令与bridge receipt时间差，不能全部解释为执行误差。
- 出生左下离线评估ROI x[-1.8,0],y[-1.5,0],z[-.25,2.5]：原始深度5%采样的4cm体素3999（前候选同区域约36点），不向在线算法提供房间坐标。证明该区域实际观测显著增加，不证明所有房间完整。
- 已知体积1203.292m³（前候选1164.973），路径548.280m（前456.279），拦截4帧（前179）；更多覆盖伴随更长路程，不能声称每米效率提高。
- run_habitat_falcon.py默认实际执行planner yaw，提供显式legacy-motion-heading复现旧模式；给4个旧试验启动脚本加legacy标志。记录yaw_policy及executed_command_yaw，避免再混淆原始采集与离线yaw重渲染。CLI及默认参数静态检查通过；重测使用同一已验证use-planner-yaw执行分支。


## 用户请求提交当前有效版本与裁剪展示
- 保存实际运行8个源码及头文件、config与启动脚本到runtime/falcon_selected_yaw_checkpoint，哈希逐项对应本次试跑。仅提交探索修复、渲染入口和运行/审计工具；Boxer/房间未完成实验不混入。
- 当前runner保留HEAD原有depth-only及可选max-flight-z支持，仅增加默认规划yaw、显式legacy模式与采集朝向元数据；桥接恢复HEAD以保留原能力，普通RGB-D路径等价。
- 用户指出视频约28–29秒后超出可见建筑结构，展示截取前28秒，完整记录保留且覆盖数字仍是全720秒。轨迹在配置box内（0越界），不据此宣称建筑内安全；后段真实建筑包含关系仍需分析。
