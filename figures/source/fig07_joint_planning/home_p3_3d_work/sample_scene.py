"""Sample GLB mesh surfaces in three dimensions for experimental occupancy construction.

This is an approximate point cloud, not a watertight or certified collision map.
The GLB length unit is unknown; 0.0254 m per unit is an unverified working assumption.
"""
from pathlib import Path
import json
import numpy as np
import trimesh
HERE=Path(__file__).resolve().parent
SRC=Path('/home/uav/下载/home_-_p3.glb')
np.random.seed(7)
scene=trimesh.load(str(SRC),force='scene')
mesh=scene.to_geometry()
# glTF Y-up -> Blender Z-up, the same transform used by Blender's GLB importer.
points,_=trimesh.sample.sample_surface(mesh,2_000_000)
xyz=np.column_stack((points[:,0],-points[:,2],points[:,1])).astype(np.float32)
# Restrict to the visible main interior, leaving a margin around the walls.
mask=(xyz[:,0]>=-405)&(xyz[:,0]<=15)&(xyz[:,1]>=-440)&(xyz[:,1]<=-40)&(xyz[:,2]>=-5)&(xyz[:,2]<=95)
xyz=xyz[mask]
np.savez_compressed(HERE/'home_p3_surface_sample_draft.npz',points_glb_xyz=xyz)
(HERE/'surface_sample_info.json').write_text(json.dumps({'source':str(SRC),'samples':len(xyz),'seed':7,'axis_conversion':'Blender xyz = glTF (x,-z,y)','unit_scale_m_per_glb_unit':0.0254,'scale_status':'unverified assumption','collision_status':'approximate surface sample, not validated, no free-space observations'},indent=2),encoding='utf8')
print('sample points:',len(xyz),'bounds:',xyz.min(axis=0),xyz.max(axis=0))
