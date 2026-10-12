from pathlib import Path
import numpy as np,json,habitat_sim
p=Path(__file__).resolve().parents[2]
c=habitat_sim.SimulatorConfiguration();c.scene_id=str(p/'data/versioned_data/hm3d-0.2/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb');c.scene_dataset_config_file=str(Path(c.scene_id).parent.parent/'hm3d_annotated_basis.scene_dataset_config.json');c.enable_physics=True
ag=habitat_sim.agent.AgentConfiguration();ag.sensor_specifications=[]
with habitat_sim.Simulator(habitat_sim.Configuration(c,[ag]))as sim:
 pf=sim.pathfinder
 print('methods',[m for m in dir(pf)if 'navmesh' in m or 'vert' in m])
 v=np.asarray(pf.build_navmesh_vertices());i=np.asarray(pf.build_navmesh_vertex_indices()).reshape(-1,3)
 np.savez(Path(__file__).parent/'navmesh.npz',vertices_habitat=v,triangles=i)
 tri=v[i];cent=tri.mean(1);area=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)/2
 levels={}
 for h,a in zip(np.round(cent[:,1],1),area):levels[str(h)]=levels.get(str(h),0)+float(a)
 print('bounds',v.min(0),v.max(0));print('largest height bins',sorted(levels.items(),key=lambda x:-x[1])[:18])
