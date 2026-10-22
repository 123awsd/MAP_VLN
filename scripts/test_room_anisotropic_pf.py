"""Evaluate public APF reproduction on accepted acquisition, floors separated offline."""
from pathlib import Path
import sys,json,time,hashlib,argparse
import numpy as np
from scipy.spatial import cKDTree
from PIL import Image,ImageDraw
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=Path('outputs/room_apf_latest_trial'));p.add_argument('--vis-targets',type=int,default=512);p.add_argument('--variant',default='reference');a=p.parse_args();out=a.output
sys.path.insert(0,str((out/('reference' if a.variant=='reference' else 'corrected')).resolve()))
from roomseg.pipeline import segment
palette=np.array([[130,190,225],[244,174,139],[179,213,153],[200,171,219],[238,216,125],[144,211,193],[225,159,186],[179,190,230],[212,187,155],[170,216,215],[229,197,208],[180,222,170]])
results=[]
for f in [1,2,3]:
 points=np.load(out/f'L{f}_input.npy');path=out/f'L{f}_{a.variant}';path.mkdir(exist_ok=True);t=time.monotonic();normals=np.empty_like(points);tree=cKDTree(points)
 for start in range(0,len(points),5000):
  _,ix=tree.query(points[start:start+5000],k=20,workers=2);near=points[ix];near-=near.mean(1,keepdims=True);_,v=np.linalg.eigh(np.einsum('nki,nkj->nij',near,near));normals[start:start+len(ix)]=v[:,:,0]
 print('floor',f,'normals ready',flush=True)
 labels,art=segment(points,normals,voxel=.18,vis_targets=a.vis_targets or None)
 np.savez_compressed(path/'segmentation.npz',points=points,point_labels=labels,interior=art['interior'],potential_field=art['pf2d'],cells=art['cells'],cell_labels=art['cell_labels'],grid_origin=art['grid'].origin,voxel=.18)
 unique,count=np.unique(labels,return_counts=True);record={'floor':f,'variant':a.variant,'input_points':len(points),'room_clusters':art['n_rooms'],'point_labels':dict(zip(map(str,unique.tolist()),count.tolist())),'unassigned_point_fraction':float((labels<0).mean()),'interior_voxels':int(art['interior'].sum()),'pf_cells':len(art['cells']),'pf_thresholds':list(map(float,art['thresholds'])),'stage_benchmarks':art['bench'],'elapsed_s':time.monotonic()-t,'voxel_m':.18,'visibility_targets':a.vis_targets,'ground_truth_used':False,'quality_validated':False};results.append(record);(path/'metrics.json').write_text(json.dumps(record,indent=2));print(record,flush=True)
 cells=art['cells'];cl=art['cell_labels'];nx,ny=art['pf2d'].shape;canvas=np.full((ny,nx,3),245,np.uint8);colors=np.full((len(cl),3),180,np.uint8);good=cl>=0;colors[good]=palette[cl[good]%len(palette)];canvas[ny-1-cells[:,1],cells[:,0]]=colors
 occupied=art['grid'].busy.any(2);# Do not project floor/ceiling occupancy as black walls.
 occupied=art['grid'].busy[:,:,2:-2].mean(2)>.65
 canvas[np.flip(occupied.T,0)]=(40,40,40)
 im=Image.fromarray(canvas).resize((nx*6,ny*6),Image.Resampling.NEAREST);im.save(path/'rooms_top.png')
 pf=art['pf2d'];scaled=np.clip(pf/max(float(pf.max()),1e-9),0,1);rgb=np.stack([255*scaled,150*np.sin(np.pi*scaled),255*(1-scaled)],axis=-1).astype(np.uint8);rgb[~art['interior'].any(2)]=245;Image.fromarray(np.flip(rgb.transpose(1,0,2),0)).resize((nx*6,ny*6),Image.Resampling.NEAREST).save(path/'potential_field.png')
 with (path/'labeled_points.ply').open('w') as fh:
  fh.write(f'ply\nformat ascii 1.0\nelement vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nproperty int label\nend_header\n');np.savetxt(fh,np.c_[points,labels],fmt=['%.4f','%.4f','%.4f','%d'])
(out/f'{a.variant}_summary.json').write_text(json.dumps(results,indent=2))
