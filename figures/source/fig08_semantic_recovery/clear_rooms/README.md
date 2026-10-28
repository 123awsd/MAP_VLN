# Fig. 8: household verification and semantic recovery

The user approved `fig8_household_check_preview.png` and authorized replacing Fig. 8 in the manuscript on 2026-10-24. `fig8_household_check_en.pdf` is the publication asset; `.svg` and 600-dpi `.png` are retained alongside it. The earlier `fig8_clear_rooms_preview.png` is superseded: its color-based sofa task does not match the system's capabilities.

`export_publication.py` preserves the accepted composition and removes the redundant preview footer. The concise publication caption in `caption.tex` identifies the figure as an illustration and describes the inspection and recovery sequence. The full source provenance remains below. The installed figure is `PRE_MAP_VLN/论文/TASE_英文初稿/figures/semantic_recovery_en.pdf`; the corresponding caption and introductory reference are in `root.tex`. The previous manuscript files are retained in `manuscript_backup_20261024/`.

The composition studies BSC-Nav Fig. 4c: an instruction band, a large scene panel, subgoal cues, a shared flag timeline, and aligned top-down and first-person views. The underlying scene and task differ from that reference.

## Scene and views

- HM3D scene: `00337-CFVBbU9Rsyb`, second floor.
- Background: the existing official floor sheet at `PRE_MAP_VLN/docs/assets/system_overview/official_scene/topdown_floors.png`, cropped to `(0, 1266, 1280, 2382)` and displayed at 78% opacity over white.
- First-person views: authored camera poses rendered with CPU Blender Cycles from the original decoded HM3D mesh and RGB textures. These are not native Habitat sensor outputs or recorded rollout frames. No generative image retouching, new room geometry, or furniture changes were used.
- Navigability thumbnails: `habitat_sim.PathFinder.get_topdown_view` on the provided navmesh at the second-floor elevation, with 0.04 m resolution. They show static navigability with cumulative path overlays, not online occupancy-map reconstruction.
- Pixel registration and camera provenance: `assets/registration.json` and `view_manifest.json`.

## Authored mechanism

The user-approved household task is: guests are arriving; check the kettle on the kitchen counter, look for a cushion on the dining chairs and continue searching if it is not seen there, then verify a bedside lamp in the twin bedroom. The local recheck views face the dining chairs. Semantic recovery proposes their nearby sofa, where a cushion is visible. The agent then crosses the hall into the unvisited western twin bedroom to verify the lamp. No color-based target selection is used. The candidate text and verification outcomes are illustrative, not a recorded Qwen response or detector outputs. Cue thumbnails are scene crops, not generated subgoal predictions.

`minco.py` transcribes the fixed-time S4NU equations from `PRE_MAP_VLN/real_fly/vendor/dls_ws_src/super_planner/src/utils/minco.cpp`. It constructs minimum-snap polynomial segments with zero velocity, acceleration and jerk at each observation stop. It does not execute the production corridor and time-allocation optimizer. `minco_coefficients.json` records coefficients, durations and numerical residuals.

`episode_geometry.json` records the route length (approximately 22.03 m), camera positions, dense curve samples and checks. Every sampled ground projection is inside the native navigability mask; the maximum sample spacing is approximately 0.0241 m. This does not certify aerial body clearance or flight feasibility. The final pose is approximately 11.18 m from all route samples before the final leg, and its bedroom has not been traversed earlier.

## Regeneration

From the workspace root:

```sh
PRE_MAP_VLN/.envs/habitat/bin/python 画图/语义恢复与地图更新/habitat_reference_draft/clear_rooms/prepare_episode.py
blender -b -t 8 --python 画图/语义恢复与地图更新/habitat_reference_draft/clear_rooms/render_views.py
/usr/bin/python3 画图/语义恢复与地图更新/habitat_reference_draft/clear_rooms/compose_bsc.py
/usr/bin/python3 画图/语义恢复与地图更新/habitat_reference_draft/clear_rooms/export_publication.py
```

The renderer requires `/tmp/fig8_00337_original.blend`. To rebuild it, decode the original basis GLB using `PRE_MAP_VLN/scripts/decode_hm3d_basis_glb.py` into `/tmp/fig8_00337_decoded`, then run `prepare_scene.py` in Blender. That script saves the complete original scene before making the temporary clipped calibration render. The calibration render is not used as the figure background.

The alternative-scene download attempted during scene selection was not executed because automatic approval review could not run due to workspace credits. This preview was completed with existing local assets.
