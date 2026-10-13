"""Export the inspected pure-geometry trial with the established video display."""
from pathlib import Path
import json
import shutil
import hashlib
import numpy as np
from scipy.spatial import cKDTree
from refine_measured_walls import ROOT,OUT,GROUNDS,render

def main():
    source=OUT/'round11';selected='AE_稀疏顶面邻接'
    dest=OUT/'selected_geometry';dest.mkdir(exist_ok=True)
    vis=ROOT.parent/'视频制作/墙体提取_斜顶几何迭代';vis.mkdir(exist_ok=True)
    geo=np.load(ROOT/'outputs/room_quality_ceiling/geometry.npz')
    crossings=np.load(OUT/'depth_crossing_counts.npz')
    configs=json.loads((source/'metrics.json').read_text())
    summaries=[]
    for f,ground in enumerate(GROUNDS):
        inputfile=source/f'L{f+1}_{selected}.npz'
        data=np.load(inputfile);points=data['wall_points'];p=geo[f'p{f}']
        dist,ids=cKDTree(p).query(points)
        assert np.all(dist<1e-6),'Export must consist exclusively of measured source points'
        counts=crossings[f'counts{f}'][ids]
        rejected=int((counts>=5).sum())
        points=points[counts<5];counts=counts[counts<5]
        origin=np.floor((p[:,:2].min(0)-.2)/.05)*.05
        size=np.ceil((p[:,:2].max(0)+.2-origin)/.05).astype(int)+1
        wall=np.zeros(tuple(size[::-1]),np.uint8)
        xy=np.floor((points[:,:2]-origin)/.05).astype(int);wall[xy[:,1],xy[:,0]]=1
        np.save(dest/f'L{f+1}_wall_candidates.npy',wall)
        np.savez_compressed(dest/f'L{f+1}_wall_points.npz',wall_points=points,ceiling_points=data['ceiling_points'])
        for kind in ['patches','roof_planes']:
            shutil.copyfile(source/f'L{f+1}_{selected}_{kind}.json',dest/f'L{f+1}_{kind}.json')
        old=np.load(ROOT/'outputs/wall_patch_context'/f'L{f+1}_局部高度宽松.npz')['wall_points']
        render(p,[old,points],ground,vis/f'L{f+1}_新旧对照.png',[f'{f+1}层 · 原墙体候选',f'{f+1}层 · 斜顶几何墙体候选'])
        shutil.copyfile(vis/f'L{f+1}_新旧对照_panel1.png',vis/f'L{f+1}_墙体结果.png')
        cfg=next(x['config']for x in configs if x['floor']==f+1 and x['method']==selected)
        meta=dict(format='pre_map_vln.measured_wall_geometry.v1',floor=f+1,ground_z_m=ground,origin_xy_m=origin.tolist(),resolution_m=.05,shape=list(wall.shape),point_count=len(points),source_trial=str(inputfile.relative_to(ROOT)),source_sha256=hashlib.sha256(inputfile.read_bytes()).hexdigest(),uses_semantics=False,uses_rgb_for_extraction=False,uses_gpb=False,synthesized_wall_points=0,depth_rejected_points=rejected,depth_crossed_3_frames=int((counts>=3).sum()),depth_crossed_5_frames=int((counts>=5).sum()),config=cfg,status='diagnostic_wall_candidates')
        (dest/f'L{f+1}_wall_candidates.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2))
        summaries.append(meta)
        print('exported',f+1,len(points),'depth_conflicts_5',meta['depth_crossed_5_frames'],flush=True)
    (dest/'manifest.json').write_text(json.dumps(dict(selected_variant=selected,floors=summaries,formal_room_outputs_modified=False),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
