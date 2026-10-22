"""Explicit user-authorized local output correction; not an automatic algorithm."""
from pathlib import Path
import json, shutil
import numpy as np
R=Path(__file__).resolve().parents[1]
source=R/'outputs/coverage700_wall_room_preview/wall_structural_rooms'
out=R/'outputs/coverage700_wall_room_preview/manual_room305_boundary'
out.mkdir(exist_ok=True)
for floor in (1,2,3):
    with np.load(source/f'L{floor}_constrained.npz') as data:
        arrays={k:data[k].copy() for k in data.files}
    labels=arrays['labels']; before=labels.copy()
    mask=np.zeros(labels.shape,bool)
    if floor==3:
        x=arrays['origin_xy'][0]+(np.arange(labels.shape[1])+.5)*float(arrays['resolution'])
        mask=(labels==5)&(x[None,:]<-4.5)
        labels[mask]=0
    arrays['manual_exclusion_mask']=mask
    np.savez_compressed(out/f'L{floor}_constrained.npz',**arrays)
    if floor==3:
        assert np.array_equal(before==4,labels==4)
        assert np.array_equal(before[~mask],labels[~mask])
        report={'kind':'manual_output_boundary_correction','automatic_algorithm_improvement':False,'source':str(source),'rule':'L3 label 5 and original X < -4.5 m becomes unassigned','removed_cells':int(mask.sum()),'removed_area_m2':float(mask.sum()*float(arrays['resolution'])**2),'room304_unchanged':True,'scene_geometry_modified':False,'raw_depth_and_map_modified':False}
(out/'manual_correction.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
code=(R/'scripts/render_wall_structural_rooms_final.py').read_text().replace("b=Path('outputs/coverage700_wall_room_preview/wall_structural_rooms')",f'b=Path({str(out)!r})').replace('新墙提取 → 独立观测支持 → 房间分割｜实测700秒数据','原版分割＋305人工边界修正｜非自动算法结果')
exec(compile(code,'render_manual_final','exec'))
print(json.dumps(report,ensure_ascii=False))
