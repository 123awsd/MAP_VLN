# Reviewed real-data panels

All panels in this directory use the real 00337 Stage-1/Stage-2 outputs. They
use floors 1--3 only, a pure-white background, no coordinate axes, and no
generated scene content. Each panel is provided as a high-resolution PNG and a
paper-facing PDF.

| Stem | What it communicates | Recommended role |
|---|---|---|
| `stage1_multifloor_categorical` | Complete 1--3 floor occupancy, one color per floor | Main Stage-1 map evidence |
| `stage1_floor_triptych_isometric` | Each floor in isolation | Floor-wise supplementary panel |
| `stage1_floor_triptych_open_cutaway` | Ceiling/front-shell removal exposes internal geometry | Main architectural cutaway |
| `stage1_multifloor_exploded` | Vertical separation between floors | Compact multi-floor overview |
| `stage1_pointcloud_with_3d_boxes` | Occupancy plus semantically grouped 3D OBBs | Object-map representation |
| `floor2_pointcloud_with_selected_3d_boxes` | Uncluttered local object example | Method inset / visual explanation |
| `rooms_with_object_box_footprints` | Room polygons plus object footprints on L1--L3 | Room segmentation evidence |
| `semantic_rooms_and_3d_boxes_stack` | Stacked semantic rooms and 3D object boxes | Semantic-map representation |
| `multiview_candidate_frustums_local_geometry` | Executable viewpoints around a support region | Observation-pose generation |
| `stage2_multifloor_execution_with_observation_frustums` | Short real multi-floor task path and observations | Stage-2 execution evidence |

Additional color and clipping experiments are kept separately under
`assets/candidates/clean_real_data/`; they are intentionally excluded here so
the reviewed asset directory stays clean.

