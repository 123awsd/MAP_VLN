# Lab room departure verification

Final preview: `lab_room_v2/check_cup_basket_trashcan_07`.
Source mission: `check_cup_basket_trashcan_04`. Observation poses, start, task
order and three-second dwell times are preserved. No NX synchronization or
flight was performed.

The original phase 3 reversed height by 0.218484 m. Full-cloud verification
also found its continuous clearance lower bound was only 0.218111 m; the
earlier 0.312 m claim applied to the filtered collision cloud, not full PCD.

The revised ascending guide is optimized against the full PCD with a 0.33 m
sampled clearance constraint, then independently checked at 4 mm spacing
(required continuous lower bound 0.32 m). Its lower bound is 0.345379 m.
CIRI uses a contiguous seed chain for ascending phases, avoiding the pruning
that had broken overlaps after adding tracking planes. Each corridor gets
a 0.05 m lateral tube and a 0.01 m vertical envelope. The optimizer includes
a negative-vertical-velocity penalty. These penalties are not algebraic hard
constraints: the final polynomial is independently accepted or rejected.

For derived tasks carrying `departure_guide_report.json`, generation runs
`verify_departure_geometry.py` before sealing and hashes its report. This
validator checks phase 3 monotonicity (minimum vz >= -0.0001 m/s, maximum
height reversal <= 0.001 m), polynomial speed/acceleration/jerk/snap extrema,
yaw rate, piece PVAJ continuity and all-phase full-cloud clearance. It uses
2 ms samples minus the certified polynomial peak speed times 1 ms to bound
continuous distance to the point set. This does not establish safety against
unobserved objects or map changes.

Final results:

- Phase 3 height reversal: 0.0000005322 m, numerical residual.
- Phase 3 full-cloud continuous clearance lower bound: 0.336791 m.
- All-phase full-cloud lower bound: 0.296554 m.
- At y=5.13: position approximately (15.802, 5.130, 0.455), full-cloud
  clearance 0.464025 m versus 0.334558 m in the baseline.
- Maximum speed: 0.953065 m/s under the requested 1.0 m/s cap.
- Total time including dwell: 37.433699 s.
- Eight regression tests pass, artifact hashes verify, mission hash and
  unchanged observation poses checked independently.

Reproduce under a NEW task ID (existing artifacts must not be overwritten):

```bash
.envs/habitat/bin/python real_fly/stage2_offline/scripts/prepare_monotone_departure.py \
  --source real_fly/stage2_offline/data/lab_room_v2/tasks/check_cup_basket_trashcan_04 \
  --dest real_fly/stage2_offline/data/lab_room_v2/tasks/NEW_TASK_ID \
  --map real_fly/stage2_offline/data/lab_room_v2/fastlio_complete/handheld_map_lab_room_v2_complete.pcd
./real_fly/stage2_offline/scripts/generate_final_minco.sh lab_room_v2 NEW_TASK_ID 0.25 clearance_optimized 1.0 1.5
```

Reports and comparison plot:
`real_fly/stage2_offline/data/lab_room_v2/tasks/check_cup_basket_trashcan_07/departure_comparison.{json,png}`.
Final coefficients and sealed full-cloud report:
`real_fly/stage2_runtime/missions/lab_room_v2/check_cup_basket_trashcan_07/`.

The first two phases retain their prior height variations; this fix targets
the departure from observation 2 to observation 3. RViz review remains manual.
