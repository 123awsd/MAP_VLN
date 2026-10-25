# Task-conditioned viewpoint generation

This directory contains the current English-only paper figure, its renderer and the data used to audit its mechanism. The adjacent `../画图参考/` directory contains the original reference images.

## Files

- `viewpoint_generation_en.pdf`: paper figure, 184 × 84 mm, vector artwork with embedded fonts.
- `viewpoint_generation_en.svg`: editable vector artwork and text.
- `viewpoint_generation_en.png`: 500 dpi export.
- `viewpoint_generation_en_preview.png`: 220 dpi preview.
- `render_figure.py`: the complete renderer and mechanism validation.
- `example_inputs.json`: the illustrative scene and expected candidate decisions.
- `mechanism_evidence.json`: regenerated validation results, geometry and production-source hash.
- `README.md`: reproduction instructions, caption, scope and references.

## Regenerate

```bash
/home/uav/miniconda3/envs/env_isaaclab/bin/python /home/uav/map_VLN/画图/候选视角生成_新版/render_figure.py
```

The renderer validates the example against `PRE_MAP_VLN/stage2/candidate_poses.py`, then writes the four figure exports and `mechanism_evidence.json` directly into this directory. It uses Matplotlib and NumPy and does not create backup directories or require a separate publishing script.

## Figure conventions

The mint, blue, yellow and coral panels read from left to right. Their a–d labels establish the order without connecting arrows. Labels are English and omit numerical sampling settings, counts, ratios and budget values. Green marks the region, blue marks proposals and passing examples, raspberry marks rejection, and coral marks retained views. The artwork is original vector geometry; no reference image is embedded.

The height inset uses a side-view drone with an integrated camera, open FoV boundaries and a height-range bracket. The displayed pose illustrates a FoV upper bound. The collision row uses matched drone–obstacle side views to show a visible gap and an occupied position. Dimensions of the drone glyphs and display depths of cones are illustrative.

## Suggested paper caption

**Task-conditioned candidate viewpoint generation.** (a) The mapped anchor and spatial relation define the observation region. (b) Candidate positions are sampled over distances and azimuths, with yaw directed toward the region. At fixed pitch, the requirement that all region samples lie within the vertical FoV constrains camera height. The side view illustrates the upper bound: the lower FoV edge touches the limiting sample, while the other samples remain inside. (c) Passing/failing pairs illustrate FoV inclusion, obstacle clearance and visibility; a view must observe some region samples. (d) The best view at each azimuth is chosen by visible sample count and terminal cost, followed by angular diversity within the viewpoint budget. Green denotes region samples and coral denotes retained views. Sampling density, subset size, drone dimensions and cone display depths are illustrative.

## Internal mechanism audit and depiction details

The numerical values below document the reproducible analytic scene used to verify the illustration. They are not labels or prescribed settings in the abstract figure.

- Source: `PRE_MAP_VLN/stage2/candidate_poses.py`, specifically the `below_region` branch with automatic distance/height, normal detail, one mapped anchor and budget eight. Other region types, explicit height constraints, fine-detail sampling and no-candidate fallback policies are outside this example.
- The mapped table is the search anchor. No backpack location is supplied to the generator or drawn as known geometry.
- The three offsets are 0.35, 0.60 and 1.40 m beyond the table box's horizontal circumscribed radius. The displayed circles and retained poses use their actual coordinates.
- All 72 horizontal proposals appear in step 2, including positions that will not produce an admissible height. All valid example heights are 0.65 m.
- Small position dots and pale 90-degree FoV sectors depict all 72 proposals. Each sector opens along the computed yaw toward the region, with no separate arrow or camera pictogram. Sectors are illustrative views at the sampled XY positions, not assertions of feasible height or unobstructed visibility before filtering. The step-2 plan has no thick radial emphasis line or azimuth-spacing wedge.
- The height construction holds the outer-radius zero-azimuth proposal's horizontal position and yaw fixed. For each sample, the FoV-compatible height interval is its height plus/minus its horizontal distance times the tangent of the vertical half-FoV. The upper endpoint of their common interval is the minimum of the sample-wise upper bounds. Its limiting sample is highlighted in the drawing, and the lower camera-FoV edge passes through it exactly. All other samples satisfy the vertical angular test at that bound. In this same-elevation region, the nearest sample is limiting; this need not hold for samples with varying heights. The drawing shows the three central-section samples with undistorted metric aspect, translated and reflected horizontally to place the camera on the left. This coordinate transform preserves horizontal distances, elevations and viewing angles. The left bracket spans only the above-ground part of the FoV-compatible interval. Its lower endpoint is the ground/display boundary, not a room setting or a depicted FoV lower bound. This figure omits the below-ground part of the geometric interval. The camera at the upper bound is an illustrative limiting pose, not the production-selected candidate height. Drone dimensions and local cone display depth are illustrative.
- The implementation still intersects the common FoV height interval with configured room sensor-height bounds and clamps the mean region-sample height to that intersection for automatic normal-detail height selection. This implementation detail is retained only in the audit. In this under-table example, the region is below the minimum permitted sensor height, so the resulting height is the lower bound; this does not imply the algorithm always chooses the minimum. An empty intersection yields no height candidate. Explicit height constraints and additional fine-detail heights are outside this figure's scope. The nominal height remains checked against `_height_candidates_for_view`. Separately, the displayed FoV upper bound is checked against the sample-wise angle tests and a production call constrained entirely above that bound returns no candidate height. The numerical construction and limiting sample are recorded in `mechanism_evidence.json`.
- The FoV comparison uses the outer-radius and inner-radius zero-azimuth proposals at the room's minimum camera height, 0.65 m. Sample bearing offsets and elevations are divided by the horizontal half-FoV (45 degrees) and vertical half-FoV (35 degrees), respectively. These exact normalized coordinates are placed on a common illustrative face of two projected frusta. Thus the rectangular boundaries preserve the code's independent angle tests, while cone depth and perspective are schematic rather than metric scene geometry or a rendered RGB image. All nine samples are inside for the far proposal. Three fall below the near proposal's lower boundary; since the camera is already at its minimum height, that proposal has no admissible height. Occlusion is checked separately.
- The clearance comparison uses the outer-radius 60-degree proposal in free space and the outer-radius 45-degree proposal inside the sofa back. Two matched side views reflect world Y about the sofa’s rear face and preserve actual camera height and the sofa back/base extents. The blue pose has a 0.2734 m gap, exceeding the 0.12 m clearance requirement; a short dimension mark shows that gap. The raspberry pose is inside the back, so its obstacle distance is zero. The occupied area around this rejected pose is highlighted. These distances and their outcomes are checked against the same analytic 3D solids as the generator audit and saved as `clearance_sections`. The drone glyph identifies position and does not replace the code’s positional validity test with mesh collision; its size and depicted heading are illustrative. Upholstery seams are decorative.
- The occlusion comparison uses the outer-radius and middle-radius 90-degree proposals in the vertical section at world x = 0. Both have level pitch and a 70-degree vertical FoV. A dense angular sweep, supplemented by exact bearings to obstacle corners, is intersected with the same analytic 3D solids. Colored cone sections terminate at the first hit; gray hatched areas behind those hits are occluded. The chair seat/back and the visible part of the tabletop are drawn at their exact section coordinates. The far cone includes the region-center sample below the chair seat; the near cone stops at the chair back and leaves that sample in shadow. The section passes between the chair legs. The original near proposal sees none of the nine region samples. Display clipping at ground level, the upper plot edge and the far edge keeps both cone sections compact and does not change candidate checks or impose camera range limits.
- Furniture seams and rounded outlines are visual identifiers. Geometric checks use the explicit analytic 3D solids in the evidence file. This scene implements the same `is_state_valid` and `line_of_sight` interfaces consumed by the production candidate generator; it does not instantiate the production voxel backend, which additionally handles observed/unknown occupancy, inflation and endpoint allowances. The counts therefore apply to this analytic example only.
- The pruning detail schematically aligns two actual views at 60 degrees. The nearer one sees two of nine samples, and the farther one sees three; the latter is therefore chosen for that azimuth and marked with a check. The mini-grids use the exact visible sample IDs, not arbitrary shaded dots.
- Per-azimuth ranking compares negative visible sample count, terminal cost and ID lexicographically. If over budget, the first retained view has lowest terminal cost; subsequent choices maximize the minimum angular separation to already retained views, with visible sample count and cost resolving ties. This is not a global top-eight score selection.
- The final plan shows all 23 representatives as faint dots and all eight selected poses in coral. Positions and yaw are exact; each pale sector is a 90-degree horizontal FoV section. Its short display radius is not a maximum camera sensing range. The former dashed radial guides have been removed. Path reachability is evaluated downstream.
- Before rendering, the script reruns the production generator and checks its automatic distance profile, every proposal's XY coordinate and target-facing yaw, height/collision/visibility status and visible sample IDs, all 23 per-azimuth representatives and all eight complete final pose records against the recorded example. It additionally checks all three displayed pass/fail pairs, the limiting height and highlighted sample against the vertical angular tests, and that the computed occlusion-cone polygons include/exclude the region-center point consistently with the exact ray intersections. The evidence file includes these pairs, the code locations of the functions called and verifies the production source hash is unchanged during the audit. The 36 rejected positions comprise 32 with no admissible height, three clearance failures and one occluded view.

## Visual references

For the viewpoint-symbol revision, actual figures in these papers were inspected:

- [FUEL: Fast UAV Exploration Using Incremental Frontier Structure and Hierarchical Planning](https://arxiv.org/pdf/2010.11561), Fig. 4 and Fig. 5: candidate samples use points; sensor FoV uses sectors. The revised marks follow this position-plus-viewing-region convention, with our own vector geometry, palette, scale and exact example coordinates.
- [Surface Edge Explorer (SEE): Planning Next Best Views Directly from 3D Observations](https://www.robots.ox.ac.uk/~mobile/Papers/2018ICRA_border.pdf), Fig. 4: a simplified sensor and FoV boundary distinguish position, orientation and viewing extent. Reviewed as an alternative; its central direction arrow and sensor body are not used here.
- [FC-Planner: A Skeleton-guided Planning Framework for Fast Aerial Coverage of Complex 3D Scenes](https://arxiv.org/pdf/2309.13882), Fig. 3(b): small wireframe frusta depict viewpoints. Reviewed for the full-frustum convention; the dense plan uses simpler sectors to remain legible at publication scale.

These are visual references only. Their sampling, visibility and selection algorithms are not substituted for the project's candidate generator. No paper artwork is embedded in the outputs.

The user's local references were re-examined for their visual organization:

- `画图/画图参考/9312204f8ec958c1515d5d0005b97ae8.png`: explicit input-to-output transitions and adjacent visual evidence for each operation.
- `画图/画图参考/90401ad4053d2cce1bcd6c3e3d050abe.jpg`: direct spatial marks, concise labels, and warm scene geometry paired with cool method overlays.
- `画图/画图参考/d0d57a2159d6df08c011a0ce4d97b5fe.jpg`: compact mechanism details and restrained separators.

No figures, method claims or data are copied from these references. Complete publication metadata was not supplied with these images, so no paper identity is inferred from filenames.
