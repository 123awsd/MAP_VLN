# Active experiment handoff

## Final status — 2026-10-16

Independent suite finished. Authoritative report: `docs/CONSISTENCY_VALIDATION_RESULTS_20261016.md`; machine report/CSV and final_identity_checks under `outputs/consistency_validation_20261015`. All six repeated performance bags finalized, source manifests match runtime and snapshots, clearance0.07/visible4/gain64/crossfloor18/defaultAstar0.001s0.5m verified.

R0 light two504:1184.024/1007.338m³. A:660.831/1108.146m³. B:820.027/992.267m³. A introduced duration rejection incompatibility, first full run stalled252s; do not promote. B11realC++ checks passed but full nonregression not established. C no proven lost reachable observation, no algorithm modification. Combination NOT RUN because independent system gate failed; do not automatically merge or launch AB. No new baseline promoted. Dedicated container has no active ROS tasks, only idle PID1 plus defunct children. Prior resume sessions completed or failed and are no longer active. Original accepted entrypoint preserved.

Below is historical execution context, not current running state.

User authorized the entire R0/A/B/C/combination suite and wants the final report after completion. Do not stop after partial results. Do not touch accepted selected_yaw entrypoint or historical source_snapshot. All heavy runs sequential in `consistency-audit-20261015` container, host GPU Habitat, /data output symlinks.

Current workers:

- `run_consistency_stage.py controls`: two formal 504s R0 diagnostic-v2 runs, r2 then r3. Unified exec session2308.
- `continue_consistency_validation.py`: waits for r3 audit, runs latest A final90 fault gate, compiled A server tests, two A504 runs, syncs B, adds EXCLUDE_FROM_ALL real C++ search fixture target, runs B fixture, then two B504 runs. Unified exec session25437; log `outputs/consistency_validation_20261015/continuation.log`. Stops on error. Inspect and fix actual failures; do not silently count them as passed.
- After independent runs complete, assess results before preparing/running AB. C is diagnostic only unless a real loss of valid observation is established; do not modify thresholds to force a room entry.

Completed:

- 61 historical executable/config file identity checks match except unarchived package.xml metadata. Isolated container image recorded in environment_identity.json.
- Historical bathroom raw data independently confirms949 visibility events,780fail,169pass,9 bathroom68frames223.2–236.6s, room2 nearest0.7175058m. Saved historical_bathroom_verification.json.
- R0 pilot diagnosticv1 504:992.594m3,479.944m,25blocked. Not a matched diagnosticv2 formal control.
- R0 formal r2:885.653m3,303.877m,760blocked;133.343s final adjacent-subcm pause. Formal r3 currently running.
- A1 feedback30: real bag150capture stamps,919repeat messages retain stamps,velocity transport error0; passed.
- A server_v5: all7 integration assertions passed. Earlier failed harness versions lacked robot FoV config; not implementation acceptance evidence.
- A_gate90: all7 fault-contract assertions passed, one injected safe-old-plan failure preserved active trajectory, both collision entrances exercised,15fresh recoveries,73held frames andzeroheld displacement. A has since tightened finite-derived-state validation, endpoint stopped derivatives, snapshot-local bridge pose reads and true acquisition timestamp; latest queued final90/server tests revalidate final source before full runs.

Implementation overview:

- A only changes5 files vsR0: simulator true capture/velocity/explicit hold primitive; bridge stable snapshot stamps+hold; FSM retained active LocalTrajData, scheduled candidate/accepted/switched protocol, both collision entrances and fresh-state recovery, adaptive future lead; server queued validated trajectories and ack/active topics, emergency actual hold. No Frontier/clearance/search-rule changes.
- B changes astar.cpp/h, path_cost evaluator.cpp, manager.cpp/h. All search return sites typed; validates endpoints using existing rules; records mode/start/goal/map stamps. Finite route estimates preserved for integer solvers, unknown reachability tracked separately perdecision, computationally incomplete/invalid-start outcomes do not cool goals. Real C++ fixture tests natural0budget timeout,2node resource exhaustion, bounded free component exhaustion, success, invalidpoints, actual manager cooldown andcost propagation. B has not yet been compiled/tested.
- Common diagnostic-v2 gives stored stable Viewpoint IDs, generated/geometry rejection exact reasons, frontier signature/map stamp, visibility/classes/gain/quota/dedup/cooldown/scoring. Candidate ID0 means added routing/pending fallback, not a generated viewpoint ID. Room coordinates are offline audit only.

Entrypoints:

- `scripts/run_consistency_case.py VARIANT CASE --duration N [--faults]`: build sync, effective params/full runtime identity, capture bag/simulator/volume logs; fresh outputs only. Needs escalated execution for Docker/GPU and /data writes.
- `scripts/check_consistency_execution_gate.py TRIAL`: A contract gate; `scripts/audit_consistency_feedback_ros.py` runs inside sourced ROS container to inspect real bag.
- `scripts/test_consistency_server.py OUT`: run inside container with ROS_MASTER_URI uniqueport and ROS_HOSTNAME127.0.0.1; loads robot/perception config, direct binary, handles own process groups.
- `scripts/audit_consistency_case.py TRIAL`: actual acquisition-time metrics and lifecycle counts; host needs escalation because outputs symlink /data. Reference3-floor areas and bathrooms offline only.
- `scripts/run_consistency_stage.py {controls,A,B,AB}`: repeats each full condition twice, audits each before next.

Formal source changes now frozen. Do not keep adding unrelated checks or alter thresholds. Review patches in `review` were generated before last minor A/B refinements; regenerate at end. Some preparation scripts were incremental; final per-run source snapshots and generated review diffs are the authoritative reproducible evidence.


## 2026-10-16 定点故障阶段（完成）

用户授权先处理 A 真实失败轨迹，审计 B 停滞和 C 已选中卫生间目标，避免再扩大完整矩阵。已完成：
- 独立 A2_yaw_domain，仅改原 A 的 traj_server.cpp/exploration_fsm.cpp；短 yaw 合法，结束后角度保持、导数置零。原 A/B/R0 不修改。
- 真实轨迹137（2.09025s/1.58485s）服务端9项回归全通过，原7项接管契约全通过；90s仿真故障注入7项、反馈4项全通过。未运行504s，不晋升/合并。
- B r1：188.53s内最大漂移6.6mm；停滞后281次规划失败/14次发布，覆盖仅增0.006m³；5303抑制事件/41个目标位置，未对应同轮同位置发布；独立日志微测完成，不能据此推断在线覆盖损失。
- C r3 plan172候选2858472→轨迹177→服务端接收→实际有限移动→场景受阻→20s冷却→换178。发布晚0.7675s，轨迹1.4201s；此轨迹yaw与位置等长，与A短yaw故障不同。候选规则不改。
- 详情 docs/TARGETED_FAILURE_VALIDATION_20261016.md；证据 outputs/consistency_validation_20261015 与 outputs/consistency_A2_yaw_gate_90_20261016。


## 2026-10-16 迟到接管与B受阻初期（定点阶段结束）

- 用户限定两项核查，不跑504、不组合A2+B。A2未改动，哈希与A2_yaw_domain_validation.json一致。
- 177首命令按t=0.781574s求值，与176前命令相差0.834527m/1.745195m/s；176原4.129798s期限被kind0截到2.150671s，提前到期后0.465904s才发布177，事件链明确。不能直接断言此跳变造成碰撞。
- 独立冻结R0服务器复现迟到中段接管；A2拒绝迟到177、拒绝只改时间戳的同控制点178、保留旧安全轨迹，全部回归通过。原137回归保持之前证据。服务器仍未使用实测twist做完整接管复核，不宣称所有接管已安全。
- 原B315.918s地图缺失；26方向离线场景/接触检查有5个通过，但不代表原在线地图可达。
- 独立B_recovery_probe仅FSM加只读局部图/独立A*诊断；120s无受阻；补采最长360s、读到受阻诊断后5s结束，实际206.931s结束（首次原始受阻后8.212s，记录消费/收尾有延迟）。两次现场FREE、ESDF约0.47m；16/11方向通过在线采样+A*+离线执行几何，但未优化/命令。新轮后来移动1.406m，未复现历史188s停滞。
- B的积分stamp字段始终0，不能将same_stamp当地图版本证据；原B只声明未赋值。诊断回调15/52ms。无全局原子地图快照，不得冒充同状态重放或宣称B恢复修好。
- 详情 docs/LATE_HANDOVER_RECOVERY_AUDIT_20261016.md。容器当前同步到独立B_recovery_probe；正常run脚本会按指定checkpoint重新同步，不能把当前容器二进制当冻结R0。

## 2026-10-16 实测切换复核与一致快照（本阶段完成）

- 用户要求先补齐A2切换时实测位置/速度/yaw/时间检查、短测；独立完善地图诊断，不动B/C，不跑504。
- 新候选A3_switch_review只改A2的traj_server.cpp。接收阶段检查保留；实际切换按旧轨迹把实测残差换到共同时间，验证位置/速度/yaw、反馈新鲜度、候选剩余时域。候选失败保留健康旧轨迹，实际跟踪失效才保持。紧急停止取消已认可未切换候选；非法反馈保留有限缓存。
- 最终A3：切换故障14项、原服务端7项、真实137的9项、迟到177的4项均通过；最终90秒执行7项、反馈4项通过。39次切换确认，57.420m/62.674s有效平移，最长小位移段6.002s后恢复。10次接收连续性拒绝、1次切换过期反馈拒绝。只作为候选，不据单次短测认定效率不退化。
- A2_map_snapshot独立副本只改map_server.{cpp,h}/exploration_fsm.{cpp,h}：完成深度/点云TSDF+占据+ESDF积分后，在同一锁内更新源stamp和提交版本；同锁保存局部完整crop+halo并执行26个受限A*探测，记录固定实际状态、目标、接触参数；接触变化则中止。
- 地图副本首构建因误引用B接口失败，未仿真；r2手动快照35s通过；最终35s自动受阻信号触发也通过。版本4/9/14/22，每份15625体素，独立二进制复算104个在线占据/插值ESDF查询全通过，误差<5e-7。注入受阻信号不是物理碰撞证明。
- 快照只支持局部地图查询，非全局TSDF/Frontier/FSM重启检查点，不能声称重现历史B188s故障。源积分/机器人实际state分别保留采集时间，自动事件差约0.205s如实保存。锁内诊断约20–50ms，不视为无开销。
- R0/B/A2与原完整实验/冻结清单哈希核验未变；新候选源码与最终各自实验快照哈希完全匹配。
- 报告 docs/SWITCH_REVIEW_AND_MAP_SNAPSHOT_20261016.md；候选冻结清单 engineering_candidates_20261016.json；review/{A3_switch_review,A2_map_snapshot}.patch。当前容器编译的是A2_map_snapshot；正常启动需用指定checkpoint同步，不能冒充正式R0或A3。独立A3服务器/tmp/A3_switch_review_traj_server用于全部最终服务端回归。
- 没有运行504秒或A2+B，没有修改B/C算法。后续是否做完整A3验收需另行安排；本轮不追加实验。

## 2026-10-16 纯A3首次504秒整机验收（完成，性能不通过）

用户授权纯A3完整验收，改善后才重复；45秒持续无位移/无覆盖且规划失败或受阻可提前判负。未启用快照/故障注入/B/C。
- consistency_A3_acceptance_504_r1_20261016正常完成503.908s，未触发提前失败。源码匹配最终A3冻结哈希，config与R0逐字一致。
- 已知824.591m³，路程350.422m，实际平移360.737s，最长小位移5.607s，受阻3帧。三楼固定区未进入；2号旁0帧，9号185帧（418.224–463.669s）。未证明卫生间完整观测。
- 200接收认可/194切换/194交接确认；无正常replan提前截断，切换命令最大相邻步0.094959m、速度差0.476867m/s；未复现历史0.835m跳变。真实反馈2518唯一采集/15145重复消息，速度误差0。
- 278次规划全部返回成功，非历史B长失败链。47停止：32candidate_late（全部旧轨迹已过原期限）、11server_rejected、2SceneGeometryFeedback、2candidate_unsafe；另38active_preserved。最早16.457s server_rejected旧轨迹已过期0.204s；56.786s首次candidate_late旧轨迹已过期0.913s。下一关注点是成功规划未能及时形成有效接管，不是新增Frontier规则。
- 相对R0两次平均已知低约24.74%，未进三楼，性能验收不通过；保持R0正式基线，A3不晋升。不自动重复第二轮，不开地图快照再跑，不开发A4，不动B/C。报告 docs/A3_FULL_ACCEPTANCE_20261016.md。
- 当前容器已同步纯A3，运行退出并关闭该master。所有正式状态以具体run源码快照/身份文件为准。


## 2026-10-16 截止时间唯一修复与跨层只读核查（本阶段完成，未晋升）

- 用户附件要求不重复504、不扩诊断矩阵：先用已有数据分类32次candidate_late，实施唯一时间调度修复+一次有界测试，另核对三楼去向。
- 分类按首次预算不足边界：17次规划启动已过期，14次旧轨迹剩余短于承诺提前量，1次计算越过原余量。原A3到期触发仅50ms；全278次求解p95约0.928s。原32次求解中位0.666s，启动剩余中位-0.134s。
- 原A3三楼高度视点发布176/177：176接管但样条局部终点仅z4.212；177能到z5.953却因continuity拒绝，178改回二楼。不能说从未调度三楼，也不证实所有跨层失败根因。
- 独立A3_deadline_schedule仅FSMcpp/h变化，server逐字与A3相同：实测32样本p95求解+交付预算提前触发、过期起点须停止后新鲜反馈、完成已迟到则丢弃不重盖戳；不提前记录目标已完成。生产预算类6项确定性测试通过。
- 唯一120s运行consistency_A3_deadline_120_20261016完成120.860s，100.980m/99.222s平移，最长小位移2.802s，0受阻帧，已知382.798m³。实际反馈4项通过。
- 同<=120s历史窗口：切换56→54、拒绝5→19、停止4→7。新标签candidate_late为0仍有1planning_deadline_missed+2active_expired_before_plan，不能宣称解决迟到。连续性拒绝11，id_or_pending5。短测未晋升、不跑504，不追加第二套补丁。保留修复失败证据。
- 报告docs/DEADLINE_SCHEDULING_REPAIR_20261016.md，现有数据逐事件A3_deadline_audit.json/上层目标A3_upper_target_segments.json；补丁review/A3_deadline_schedule.patch、冻结deadline_schedule_manifest.json。下一断点提前计算候选的起点/交接生命周期，禁止放宽服务器接受过期轨迹。
- 当前容器为独立A3_deadline_schedule，实验已结束；R0/原A3/B/C保持原样。

## 2026-10-16 首条continuity/id拒绝与最终载荷冻结（完成，运行未晋升）

- 用户要求仅现有数据先还原两条首次拒绝，按证据改唯一交接边界，确定性回归后仅一次120s，不跑504、不调阈值。
- 首continuity plan24/candidate24/base22，pending无未完成accept，handover时间一致；旧22 wire速度约[-.0917,.7083,-.0163]，候选回退安全前缀初速度近0，差.714327，与服务端记录吻合，安全拒绝合理。FSM旧22速度/时长缓存未随发布前lengthenTime刷新：wire duration6.08453而active cache10.043163。
- 首id_or_pending plan34发33，服务端active33，无pending；全5次均重复active ID。PLAN提前分配issued_id，PUB安全前缀内部位置规划再次++local traj_id，分配器没跟进，后续复用wire ID。不是pending不清除的已证实缺陷。
- 独立A3_payload_contract基于deadline副本，仅FSMcpp/h：回退/时间修改后刷新全派生状态和duration；最终安全/时效检查后单点分配wireID；冻结pending_data/baseid，用冻结pending确认安装active。server逐字不变，未新建跨进程base消息，保留零速前缀超速差拒绝。
- 历史真实样条编译生产冻结语句5项回归通过。唯一120s运行consistency_A3_payload_contract_120_20261016，110.672m/91.865s平移，0几何受阻，反馈4项通过。完整54切换，统一<=120s窗口53；对前一版窗口54，未增加。拒绝19→11但停止7→13。wire ID严格唯一递增，全部54安装duration与FSM冻结误差0。id_or_pending=0，但continuity4/switch_tracking5/switch_continuity2保留。
- 未达到“切换增加、停止不增加”晋升条件；不跑504、不追加第二套补丁，冻结R0/原A3/之前deadline副本未变。报告docs/FIRST_REJECTION_PAYLOAD_CONTRACT_20261016.md；first_rejection_audit/fixtures/payload_finalization_test/payload_contract_bounded_result；review/A3_payload_contract.patch及payload_contract_manifest.json。
- 当前容器为独立A3_payload_contract，实验master已关闭，正常启动须指定checkpoint，不把当前容器当正式基线。


## 2026-10-16 运动前缀修复后直接504（完成，整机未通过）

用户要求不再120短测，修前缀移动边界后回归→504。独立A3_motion_prefix保留server及所有阈值；helper使用实际约定vel/acc/yaw，无法兼容仍拒绝，停止后真实状态再规划。历史边界回归速度误差1.82512e-5（合成FREE地图，不是历史地图），然后直接完整504，未追加120。

完整结果 {"known_m3": 829.126, "duration_s": 503.9108338356018, "path_m": 364.6367796936711, "translation_s": 340.5411524772644, "longest_small_step_s": 62.0625479221344, "third_floor_entry": null, "bathroom2_frames": 94, "bathroom9_frames": 0, "stop_reasons": {"finalized_candidate_late": 1, "candidate_late": 2, "planning_deadline_missed": 10, "server_rejected": 39, "moving_prefix_unavailable": 4, "safetyCallback": 5, "SceneGeometryFeedback": 5, "active_expired_before_plan": 2, "plan_failed": 8}, "rejections": {"continuity": 10, "switch_tracking": 8, "time": 31, "switch_continuity": 2}}

三楼未进、体积未恢复，版本不晋升、不重复。2号进入94帧，9号未进入。docs/MOTION_PREFIX_FULL_VALIDATION_20261016.md。用户中断等待后核实实验已正常结束，master关闭；当前容器A3_motion_prefix。


## 2026-10-16 62秒与31次time窄审计（完成；无新探索）

- 用户禁止叠补丁/新仿真，先查152.752–214.814s与全部31time；若同链只修确定边界并历史回归。本轮只读历史bag+服务端回归，无120/504、无新地图。
- 窗口实际路程/净位移/max偏移均2.882mm，0几何拦截；311采集310零速、279显式hold，全为旧88；bag6207命令6188零速。旧88在152.942s到期，无新切换。覆盖仅+0.018m³。
- 前54s为8次FAIL各6.7s（plan97–104）；失败也计入solve p95，预算升到6.9s。随后成功只约0.1s，但31条time全部接收时提前6.731–6.819s，超过server3s上限，非候选过期。首89发布→拒绝约8ms，起点提前6.798s。31次只在207.753–214.603s，不解释前54sFAIL。
- 原早停未触发因要求45s>=15FAIL或100blocked，此处8慢FAIL且31server_reject不计，位置/覆盖条件不是主要阻碍。未改监控。
- 唯一A3_admission_window只改FSMcpp/h：不可变候选过早则等待原窗口，进入后检查基准/新鲜实际/完整轨迹安全并再次查时间，只提交一次；stop取消，过期拒绝，不重盖戳/ID/缩放。等窗口的意图时间不进入交付开销反馈。server及其阈值逐字未变。
- 生产predicate重放31全部WAIT/边界检查通过；真实89原样条同绝对时刻，提前6.8s发送重现原server拒绝，进入原窗口后同payload认可并实际切换；仅理想反馈ROS服务器回归，不是完整FSM/历史地图重放。
- 原移动前缀冻结哈希保持不变。docs/STALL_62S_TIME_WINDOW_AUDIT_20261016.md；motion_prefix_62s_and_time_audit.json/motion_prefix_62s_bag_audit.json/fixture89；review/A3_admission_window.patch、A3_admission_window_manifest.json。原8次FAIL仍未结案，未晋升/未追加整轮。当前容器同步A3_admission_window，所有regression master已退出。

## 2026-10-16 plan97–104 existing-log audit

Confirmed all eight FAILs return from planTrajToView path search before optimization: primary + 12 identical ordered recovery goals each cycle, 104 fine failures; 52.0069 of 53.7466 seconds in fine-call intervals. Historical termination reason/map version absent; no search/selection patch or 504 run. Added reproducible audit and separate experimental command-hold stop-loss branch, saved-data branch regression passed. Actual FSM wait–submit integration remains unverified. See docs/PLAN97_104_FAILURE_AUDIT_20261016.md.

## 2026-10-16 shared search deadline and retry scheduling

Implemented independent A3_search_round: FSM-derived shared search deadline, accurate A* terminal metadata, finite context/version-aware deferral and fair full-queue retry scheduling, integration epochs. Server/config/scoring unchanged. Actual A* regression passed. First bounded run FAILED due to introduced fixed whole-manager 0.5s cap: 104s pause; preserved and corrected deadline wiring. Required 90s retest: path82.122m, longestpause3.201s, 11 budget exits all followed by motion+coverage within10s, 44 switches, one safe continuity rejection. Live candidate searches all succeeded; historical failure batch NOT reproduced. No504/baseline promotion. Report docs/SEARCH_ROUND_REPAIR_20261016.md, freeze A3_search_round_manifest.json.

## 2026-10-16 A3_search_round full504 acceptance

One full run completed503.913s, no early stop. Known848.333m3, path388.510m, translation373.953s, longestsubcm6.208s, blocked4frames. Third-floor fixed entry401.007s/16frames; room2 entry379.385–386.993s/39frames; room9zero. Planning235SUCCESS/36PENDING, all pending followed by movement+coverage in10s; all235 local candidates succeeded (historical failed batch not reproduced). 150–220s maxcompute.708s; full max2.127s in preprocessing. IDsunique, normalreplan no truncation, all200switches accepted first; no time/id rejects. Feedback passed. Coverage below both R0 references: NOT promoted, no automatic repeat/newfix. Full report docs/SEARCH_ROUND_FULL_ACCEPTANCE_20261016.md and machine search_round_504_acceptance_summary.json.

## 2026-10-16 read-only183→184 decision audit

Recovered plan222/223 and actual wire curves. At400.410s,183 still had1.157s; predicted endpoint0.323m from goal, actualfeedback2.159m. ObservationOutcome400/0 wrongly settles failed observation using predicted pos and clears static pending. Floor priority explicitly replaces viable z6.26 reference withz3.17; final rescoring chooses lowgoal184 utility13.478 over high11.784. High candidates remain (18complete rows).184 localcurve endsz3.640, actual descent confirmed. HGrid/SOP indices available, final candidate stableID/ownership missing. Final observation winner does not synchronize pending_floor; exact pending value unlogged. These three code blocks identicalR0; no claim of single-cause volume explanation or replay. Frozen source unchanged, no simulation/algorithm edits. Report docs/TARGET183_184_DECISION_AUDIT_20261016.md.

## 2026-10-16 explicit observation lifecycle repair and full acceptance

Independent A3_observation_lifecycle modifies managercpp/h+FSMcpp only. Task selection/accept/execution distinct; fresh actual pose gates arrival, later stable integrated depth gates settlement; staticpending removed. Final executing target owns floorintent; actual pose owns floorarrival. Actual manager regression passes historical183 boundary, integration waiting, cancel/reject and same-ID settled-terminal continuation. First started run engineering-aborted on review for terminal idempotence, preserved, not504. Final run503.912s: known990.700m3, path323.957m, translation306.872s, longestpause7.807s, blocked2frames. L3fixed248frames; continuous454.865–503.912s/49.047s; wholemapgain159.589m3, eight L3nonemptysettlements. Room2 86frames, room9zero. Lifecycle39settles:38nonempty success+1empty,0failure, no identity/integration/arrival/duplicate errors. Real timeout batch89–94 rotates10timeouts then95success; no54s stall. 206switches,12safe rejects,no time/ID rejects; feedback passed. Below bothR0 coverage: NOT promoted, no repeat/newpatch. Report docs/OBSERVATION_LIFECYCLE_REPAIR_20261016.md.

## 2026-10-16 room9-only readonly audit and projection correction

Old topdown used-X/Y, newdense used+X/Y; regenerated display-minusX with unchanged rawcloud. Latest nearest190.591s/.224m is real geometryblocked98 frame. Internalgoal task82 winsplan128, publishes98/accepted/actualswitch189.514, plan129stillretains82; SceneGeometryFeedback190.920 cancels+20s cooldown. Priorplan127PENDINGcancelleddooroutside80; sameinternalpositionretry128succeeded. Aftercooldownplans142–148room9scoreslosecompetitors; late423.819separateFREEcomponentcooldown, no online snapshot. ROIunknown1.064m3 is bbox not GT usefulinterior; R0r2 previouslyFREE.157m3 there. HistoricalA3entry185framesfirst418.224 atdifferentlocation/map. No planner/sourcechanges, no simulation. Report docs/ROOM9_TARGET_CHAIN_AUDIT_20261016.md.
