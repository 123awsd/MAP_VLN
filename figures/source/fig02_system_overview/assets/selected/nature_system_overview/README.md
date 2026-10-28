# Scene-led System Overview

Recommended paper exports:

- `nature_system_overview_en.pdf`
- `nature_system_overview_zh.pdf`

The composition uses one continuous visual hierarchy rather than equal-sized
method cards:

1. the official HM3D 00337 dollhouse gives physical-scene context;
2. an overlapping lens shows the recorded L1--L3 semantic map and execution;
3. a nested lens exposes one real spatial-grounding/viewpoint decision; and
4. recorded Habitat RGB plus OWLv2 detections provide terminal evidence.

The route, 16 feasible viewpoints, four selected observations, 48.1 m path,
height profile, object boxes, and detections come from
`outputs/stage2_3d/tasks/00337_all_found_showcase_v1_20260830` and its Stage-1
map inputs. This run found all four targets and did not execute active
recovery. The official dollhouse and FALCON map are linked representations of
the same scene; no pixel-level registration is claimed.

Official scene page:
<https://aihabitat.org/datasets/hm3d/00337-CFVBbU9Rsyb/index.html>
