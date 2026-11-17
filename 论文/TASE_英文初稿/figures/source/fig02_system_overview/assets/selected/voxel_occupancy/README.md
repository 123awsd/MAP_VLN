# Stage-1 3D voxel occupancy assets

These panels use the real FALCON voxel map for floors L1-L3. Whole-building panels render OCCUPIED voxels because full FREE/UNKNOWN volumes occlude the building. Local-state panels explicitly show OCCUPIED, FREE, and UNKNOWN.

- `occupancy_3d_whole_ocean`: primary blue/green/orange floor palette.
- `occupancy_3d_whole_berry`: cooler alternative floor palette.
- `occupancy_3d_open_cutaway_ocean`: roof/front cutaway exposing interiors.
- `occupancy_3d_floor_triptych_ocean`: isolated L1-L3 comparison.
- `occupancy_3d_corner_notch_height`: cuboid corner cut with continuous z color.
- `occupancy_3d_corner_notch_floor_gradient`: same cut with per-floor gradients.
- `occupancy_3d_quarter_cut_height`: one full vertical quadrant removed, z gradient.
- `occupancy_3d_quarter_cut_floor_gradient`: quarter cut with per-floor gradients.
- `occupancy_3d_quarter_cut_no_slabs_height`: quarter cut without floors/ceilings/roof.
- `occupancy_3d_quarter_cut_no_slabs_floor_gradient`: open per-floor-gradient variant.
- `occupancy_3d_local_states_cyan`: dark OCCUPIED, cyan FREE, gray UNKNOWN.
- `occupancy_3d_local_states_warm`: dark OCCUPIED, amber FREE, gray UNKNOWN.

The source grid resolution is 0.10 m. Whole-building rendering aggregates voxels to 0.30 m, and detailed/cutaway panels to 0.20-0.25 m, solely for legibility and manageable paper file size. No z coordinate is flattened.
