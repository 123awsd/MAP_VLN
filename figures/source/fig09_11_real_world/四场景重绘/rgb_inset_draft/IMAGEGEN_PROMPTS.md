# Restoration preview prompts

## Latest segmentation-only pass

Tool: built-in image_gen, transparent_background=true. Seven cutout outputs are retained as assets/segmentation_ROW_COL.png; source paths are recorded in segmentation_sources.json. Only their alpha channels are used. All RGB in the final locally brightened insets comes from the pre-brightness source assets listed in local_brightness_manifest.json. Generated RGB from this segmentation pass is discarded.

Exact shared prompt, with TARGET replaced by each target description below:

Use case: background-extraction. Produce a segmentation cutout of TARGET. VERY IMPORTANT: preserve the full original 4:3 canvas, exact object position, size, silhouette, perspective and framing; do not center, move, zoom, straighten, reconstruct, sharpen or relight the object. Make every non-target pixel fully transparent, retaining only the target in its original location. No shadows or background remnants. Keep existing object pixels and appearance unchanged. This is only an intermediate segmentation mask for local brightness editing; original photographic pixels will be used in the final composite.

- 0_1: the single central white kitchen cooker, including lid and base, excluding the other container at far left
- 0_2: the white mug including its handle on the refrigerator, excluding refrigerator and wall
- 0_5: all four stacked horizontal transparent water bottles, including caps, excluding the wall, floor and table leg
- 1_1: the central white thermos cup including its metal cap, excluding the tabletop and background equipment
- 1_4: the basketball only, excluding its white circular support, chair and floor
- 1_5: the entire gray trash can and its black liner bag at the rim, excluding fan and other equipment
- 2_5: the black backpack body, handle and straps, excluding the hanging toy and background pool table

Tool: built-in image_gen, edit mode, three separate calls. Inputs are retained original crops `assets/inset_0_1.jpg`, `assets/inset_0_2.jpg`, `assets/inset_0_5.jpg`; output copies are `assets/enhanced_0_1.png`, `assets/enhanced_0_2.png`, `assets/enhanced_0_5.png`. No enhanced pixels replace the originals.

Exact common prompt, with TARGET replaced by `pot`, `cup`, or `water` respectively:

Use case: precise-object-edit. Edit target: supplied low-resolution experimental photograph crop. Perform CONSERVATIVE image restoration only: reduce noise and compression artifacts, modestly deblur existing edges, improve local contrast. Preserve exact 4:3 framing, camera pose, object silhouettes, geometry, positions, sizes, counts, occlusions, colors and background structures. Do not rearrange, add, remove or redesign anything. Do not invent readable text, logos, controls, bottle labels, liquid levels, seams or fine details; leave uncertain details soft rather than guessing. No new objects, no stylization, no new lighting, no arrows, boxes or text overlays. Keep the result visually registered to the source, only clearer. Target: TARGET. This is an AI-restored presentation preview, not new evidence. Output the single restored crop at 4:3.

The prompt expresses desired preservation constraints; it does not certify pixel-level structure preservation. AI restoration can infer fine details. The enhanced variant is a presentation draft, not a substitute for original measurements or evidence.

## Brightness balance pass

Built-in image_gen, five individual edits. Inputs: display_0_2_enhanced.png, enhanced_0_5.png, inset_1_1.jpg, inset_1_4.jpg, inset_1_5.jpg. Outputs retained as brightened_0_2.png, brightened_0_5.png, brightened_1_1.png, brightened_1_4.png, brightened_1_5.png. Exact shared prompt:

Use case: lighting-weather. Edit ONLY the brightness/exposure of this existing photograph crop. Lift dark midtones and shadows moderately (roughly +0.5 to +0.7 EV), with gentle highlight rolloff so white surfaces are not clipped. Aim for a naturally bright, evenly exposed indoor RGB thumbnail that fits alongside bright third-person indoor photos. Keep original colors and contrast natural. STRICT invariants: identical 4:3 camera framing, perspective, exact scene content, object positions, shapes, proportions, quantity, all existing texture, blur and occlusions. Do NOT sharpen or reconstruct details. Do NOT invent text, labels or new scene pixels. Do not change or remove any objects, recompose, zoom or relight directionally. Output ONLY the same photograph with a moderate global tonal lift; no annotations, no text, no frames.
