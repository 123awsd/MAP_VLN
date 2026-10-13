"""Ceiling-seeded wall-face components. Diagnostic measured points, no invented lines."""
from pathlib import Path
import json
import argparse
import numpy as np
import cv2
from scipy.ndimage import maximum_filter, distance_transform_edt
from scipy.spatial import cKDTree
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs/wall_refinement'
OUT.mkdir(exist_ok=True)
FONT = ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 22)
GROUNDS = [-.155, 2.595, 5.595]

def runs(mask):
    d = np.diff(np.r_[False, mask, False].astype(int))
    return zip(np.where(d == 1)[0], np.where(d == -1)[0])

def render(p, sets, ground, path, labels):
    # Use the actual video renderer's top projection, not the older diagnostic
    # renderer (which mirrored X and selected the lowest rather than top point).
    f = GROUNDS.index(ground)
    cache=ROOT/'outputs/exploration_recovery_local_trial/round6_extended/preview_trimmed'
    inc=np.load(cache/'increments.npz')
    raw=np.concatenate([inc[k] for k in inc.files if k.startswith('new_')])
    base=raw[(raw[:,2]>[-.2,2.6,5.4][f]+.2)&(raw[:,2]<[2.05,4.85,7.05][f])]
    refs=np.load(cache/'timeline.npz')['cloud_100']
    refs=refs[(refs[:,2]>[-.2,2.6,5.4][f]+.2)&(refs[:,2]<[2.05,4.85,7.05][f])]
    rotation=np.array([[-1.,-np.sin(np.pi),0],[np.sin(np.pi),-1.,0],[0,0,1]],np.float32)
    bounds=refs@rotation.T
    center=(bounds.min(0)+bounds.max(0))/2;extent=bounds.max(0)-bounds.min(0)
    W, H = 1290, 850
    scale = min((W-100)/extent[0],(H-90)/extent[1])
    def uv(q):
        v=q@rotation.T
        return np.column_stack([W/2+(v[:,0]-center[0])*scale,H/2-(v[:,1]-center[1])*scale]).astype(int)
    bg = np.full((H, W, 3), 255, np.uint8)
    a=uv(base);ok=(a[:,0]>=0)&(a[:,0]<W)&(a[:,1]>=0)&(a[:,1]<H)
    a,base=a[ok],base[ok];addr=a[:,1]*W+a[:,0];order=np.lexsort((base[:,2],addr))
    ids=order[np.r_[addr[order][:-1]!=addr[order][1:],True]];a=a[ids]
    height=np.clip((base[ids,2]-ground)/2.05,0,1)
    shade=(150+40*height).astype('uint8');bg[a[:,1],a[:,0]]=shade[:,None]
    board = Image.new('RGB', (W*len(sets), 970), 'white')
    for j, (q, label) in enumerate(zip(sets, labels)):
        im = bg.copy()
        a=uv(q);ok=(a[:,0]>=0)&(a[:,0]<W)&(a[:,1]>=0)&(a[:,1]<H);a=a[ok]
        mask = np.zeros((H, W), np.uint8); mask[a[:, 1], a[:, 0]] = 1
        mask = cv2.dilate(mask,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)))
        im[mask > 0] = (155, 20, 30)
        panel=Image.new('RGB',(W,970),'white');panel.paste(Image.fromarray(im),(0,110));d = ImageDraw.Draw(panel)
        d.text((20, 16), label, font=FONT, fill=(20,20,20))
        d.text((20, 52), '灰：视频显示范围；深红：全高度墙面候选', font=FONT, fill=(60,60,60))
        panel.save(path.with_name(path.stem+f'_panel{j}.png'))
        board.paste(panel, (j*W, 0))
    board.save(path)

def extract(p, n, ground, cfg):
    res = .08
    org = p[:, :2].min(0)-.4
    size = np.ceil((p[:, :2].max(0)+.4-org)/res).astype(int)+1
    shape = tuple(size[::-1])
    xy = np.floor((p[:, :2]-org)/res).astype(int)
    z = p[:, 2]-ground
    # Upper envelope in a bounded spatial neighborhood, rather than nearest 8 points.
    upper = np.full(shape, -np.inf)
    top = (np.abs(n[:, 2]) > .4) & (z > 1.35)
    np.maximum.at(upper, (xy[top,1], xy[top,0]), z[top])
    envelope = maximum_filter(upper, size=cfg['envelope_cells'])
    roofpoints = top & (z >= envelope[xy[:,1],xy[:,0]]-.15)
    roofmask = np.zeros(shape, np.uint8); roofmask[xy[roofpoints,1],xy[roofpoints,0]] = 1
    filled = cv2.morphologyEx(roofmask, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8))
    num, lab, stats, _ = cv2.connectedComponentsWithStats(filled, 8)
    large = np.zeros(shape, bool)
    for k in range(1,num):
        if stats[k,cv2.CC_STAT_AREA]*res**2 >= cfg['roof_area']:
            large[lab == k] = True
    roofpoints &= large[xy[:,1],xy[:,0]]
    roof = p[roofpoints]
    roof_records=[];roof_models=[];roof_metadata=[]
    horizontal=(np.abs(n[:,2])>.9)&(z>1.5)
    hh,edges=np.histogram(z[horizontal],bins=np.arange(1.5,3.11,.1))
    dominant_ceiling=float((edges[np.argmax(hh)]+edges[np.argmax(hh)+1])/2)
    if cfg.get('roof_planes'):
        # Extract real plane patches independently: an overhead repair plane must
        # not erase a lower slanted ceiling. Normals are unoriented.
        candidates=np.flatnonzero((np.abs(n[:,2])>.38) & (z>cfg['roof_min_z']))
        rng=np.random.default_rng(20261013)
        while len(candidates)>400:
            trials=rng.choice(candidates,min(100,len(candidates)),replace=False)
            best=None; best_count=0
            sample=candidates[::max(1,len(candidates)//16000)]
            for seed in trials:
                normal=n[seed]; offset=p[seed]@normal
                mask=(np.abs(p[sample]@normal-offset)<.055)&(np.abs(n[sample]@normal)>.94)
                count=mask.sum()
                if count>best_count: best=(normal,offset); best_count=count
            if best_count<60: break
            normal,offset=best
            ids=candidates[(np.abs(p[candidates]@normal-offset)<.065)&(np.abs(n[candidates]@normal)>.92)]
            if len(ids)<350: break
            # Refine from the measured patch rather than retain the seed normal.
            centered=p[ids]-p[ids].mean(0)
            _,v=np.linalg.eigh(centered.T@centered); normal=v[:,0];offset=p[ids].mean(0)@normal
            ids=candidates[(np.abs(p[candidates]@normal-offset)<.055)&(np.abs(n[candidates]@normal)>.92)]
            patchmask=np.zeros(shape,np.uint8);patchmask[xy[ids,1],xy[ids,0]]=1
            patchmask=cv2.morphologyEx(patchmask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
            num,lab,stats,_=cv2.connectedComponentsWithStats(patchmask,8)
            for k in range(1,num):
                component=ids[lab[xy[ids,1],xy[ids,0]]==k]
                area=stats[k,cv2.CC_STAT_AREA]*res**2
                if cfg.get('flat_ceiling_only') and abs(normal[2])<.94:
                    continue
                if cfg.get('reject_low_horizontal') and abs(normal[2])>.94:
                    if np.median(z[component])<dominant_ceiling-cfg['horizontal_drop']:
                        continue
                if area>=cfg['roof_area'] and len(component)>200:
                    roof_records.append(component)
                    roof_models.append((normal,offset,cKDTree(p[component,:2])))
                    roof_metadata.append(dict(normal=normal.tolist(),offset=float(offset),inclination_degrees=float(np.degrees(np.arccos(np.clip(abs(normal[2]),0,1)))),area_xy_m2=float(area),points=len(component),z_above_floor_range=[float(z[component].min()),float(z[component].max())]))
            candidates=candidates[~np.isin(candidates,ids)]
            if len(roof_records)>45: break
        roofpoints[:]=False
        if roof_records: roofpoints[np.concatenate(roof_records)]=True
        roof=p[roofpoints]
    roof_tree=cKDTree(roof) if len(roof) else None
    ceiling = np.full(shape, -np.inf)
    np.maximum.at(ceiling, (xy[roofpoints,1],xy[roofpoints,0]), z[roofpoints])
    ceiling = maximum_filter(ceiling, size=cfg['envelope_cells'])
    vertical = (np.abs(n[:,2]) < .35) & (z > .15)
    source = np.flatnonzero(vertical); q = p[source]; nn=n[source,:2].copy()
    predicted_top=np.full(len(q),np.inf)
    if cfg.get('sloped_envelope'):
        for normal,offset,patchtree in roof_models:
            if abs(normal[2])<.35:continue
            dist,_=patchtree.query(q[:,:2],workers=2)
            predicted=(offset-q[:,:2]@normal[:2])/normal[2]
            good=(dist<cfg['roof_xy_gap'])&(predicted>=q[:,2]-.12)
            predicted_top[good]=np.minimum(predicted_top[good],predicted[good])
    nn /= np.maximum(np.linalg.norm(nn,axis=1,keepdims=True),1e-8)
    dominant = np.angle(np.mean(np.exp(4j*np.arctan2(nn[:,1],nn[:,0]))))/4
    angles = [dominant, dominant+np.pi/2]
    if cfg['extra_angles']:
        hist, edges=np.histogram(np.mod(np.arctan2(nn[:,1],nn[:,0]),np.pi),bins=90,range=(0,np.pi))
        for idx in np.argsort(hist)[::-1]:
            if hist[idx]<hist.max()*.08:break
            theta=(edges[idx]+edges[idx+1])/2
            if all(abs(np.sin(theta-a)) > .18 for a in angles): angles.append(theta)
            if len(angles)>=5: break
    selected = np.zeros(len(p),bool); records=[]
    for theta in angles:
        normal=np.array([np.cos(theta),np.sin(theta)]); tangent=np.array([-normal[1],normal[0]])
        align = np.abs(nn@normal) > .965
        offsets=q[:,:2]@normal
        keys, counts=np.unique(np.round(offsets[align]/.04).astype(int),return_counts=True)
        accepted=[];accepted_fits=[]
        for key in keys[np.argsort(counts)[::-1]]:
            normal=np.array([np.cos(theta),np.sin(theta)]); tangent=np.array([-normal[1],normal[0]])
            offset=key*.04
            if any(abs(offset-a)<.085 for a in accepted): continue
            ids=np.flatnonzero(align & (abs(offsets-offset)<.055))
            if len(ids)<80: continue
            if cfg.get('refine_wall_plane'):
                # Global direction is only a proposal. Fit each observed wall
                # independently so a small angular error does not fragment a
                # long wall or thicken its footprint.
                local=q[ids,:2]
                for _ in range(2):
                    centered=local-local.mean(0)
                    val,v=np.linalg.eigh(centered.T@centered)
                    if val[1]<.2:break
                    candidate=v[:,0]
                    if candidate@normal<0:candidate=-candidate
                    if abs(candidate@normal)<.98:break
                    normal=candidate;offset=float(np.median(local@normal))
                    residual=abs(local@normal-offset)
                    local=local[residual<.035]
                    if len(local)<50:break
                if any(abs(normal@oldnormal)>.999 and abs(offset-oldoffset)<.065 for oldnormal,oldoffset in accepted_fits):continue
                tangent=np.array([-normal[1],normal[0]])
                ids=np.flatnonzero((abs(nn@normal)>.965)&(abs(q[:,:2]@normal-offset)<cfg['wall_tolerance']))
                if len(ids)<80:continue
            pts=q[ids]; zz=pts[:,2]-ground; tt=pts[:,:2]@tangent
            lo=np.floor(tt.min()/.1)*.1; ti=np.floor((tt-lo)/.1).astype(int)
            zi=np.floor(zz/.1).astype(int); nt=ti.max()+1; nz=max(zi.max()+1,30)
            face=np.zeros((nz,nt),np.uint8); face[zi,ti]=1
            joined=cv2.morphologyEx(face,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
            num, lab, stats, _=cv2.connectedComponentsWithStats(joined,8)
            pos=np.floor((pts[:,:2]-org)/res).astype(int)
            tops=ceiling[pos[:,1],pos[:,0]]
            touching=np.isfinite(tops) & (zz>=tops-cfg['top_gap'])
            if cfg.get('roof_planes'):
                if roof_tree is None:
                    touching=np.zeros(len(pts),bool)
                else:
                    near,ri=roof_tree.query(pts,k=1)
                    touching=(near<cfg['top_gap']) & (pts[:,2]<=roof[ri,2]+.12)
            if cfg.get('sloped_envelope'):
                tops=predicted_top[ids]-ground
                touching=np.isfinite(tops)&(zz>=tops-cfg['top_gap'])&(zz<=tops+.12)
            keep=np.zeros(len(ids),bool)
            for k in range(1,num):
                member=lab[zi,ti]==k
                if member.sum()<50: continue
                columns=np.unique(ti[member]); seedcols=np.unique(ti[member & touching])
                if len(seedcols)*.1 < cfg['seed_length']: continue
                if np.ptp(zz[member]) < cfg['height']: continue
                if len(columns)*.1 < .5: continue
                density=float(stats[k,cv2.CC_STAT_AREA]/max(1,stats[k,cv2.CC_STAT_WIDTH]*stats[k,cv2.CC_STAT_HEIGHT]))
                if density<cfg.get('face_density',0):continue
                # Near-top attachment must have breadth, not a single accidental point.
                if len(seedcols)/len(columns) < cfg['seed_fraction']: continue
                pointkeep=member.copy()
                if cfg.get('column_filter'):
                    # Component attachment alone leaks into co-planar furniture.
                    # Retain columns with a continuous vertical run linked to
                    # the roof, allowing only 10 cm sampling holes.
                    comp=(lab==k).astype(np.uint8)
                    allowed=np.zeros(nt,bool)
                    for col in columns:
                        colhit=cv2.dilate(comp[:,max(0,col-1):min(nt,col+2)],np.ones((1,3),np.uint8)).any(1)
                        longest=max((b-a for a,b in runs(colhit)),default=0)*.1
                        allowed[col]=longest>=cfg['column_height']
                        if cfg.get('relative_column'):
                            local=member&(abs(ti-col)<=1)&np.isfinite(tops)
                            localheight=float(np.median(tops[local])) if local.any() else 2.5
                            # A sloping ceiling changes the required wall span
                            # along the wall. Tall sparse posts do not count as
                            # a continuously observed floor-to-roof wall.
                            occupied=np.flatnonzero(colhit)
                            lower=(occupied.min()*.1 if len(occupied) else 9)
                            allowed[col]&=(longest>=max(.25,(localheight-.2)*cfg['relative_column']))
                            allowed[col]&=lower<cfg['lower_limit']
                    allowed=cv2.morphologyEx(allowed.astype(np.uint8)[None,:],cv2.MORPH_CLOSE,np.ones((1,3),np.uint8))[0]>0
                    if cfg.get('local_top_attachment'):
                        attached=np.zeros(nt,bool)
                        for col in columns:
                            local=member&(abs(ti-col)<=1)&np.isfinite(tops)
                            gap=tops[local]-zz[local]
                            attached[col]=bool(((gap>=-.12)&(gap<=cfg['local_top_attachment'])).any())
                        attached=cv2.morphologyEx(attached.astype(np.uint8)[None,:],cv2.MORPH_CLOSE,np.ones((1,3),np.uint8))[0]>0
                        allowed &= attached
                    keep |= pointkeep & allowed[ti]
                else:
                    keep |= pointkeep
                records.append(dict(angle=float(np.arctan2(normal[1],normal[0])),offset=float(offset),along_start=float(lo+columns.min()*.1),along_end=float(lo+(columns.max()+1)*.1),length=float(len(columns)*.1),height=float(np.ptp(zz[member])),seed_fraction=float(len(seedcols)/len(columns)),face_density=density,points=int(pointkeep.sum())))
            if keep.any():
                selected[source[ids[keep]]] = True
                accepted.append(offset)
                accepted_fits.append((normal.copy(),offset))
    return p[selected], roof, records,roof_metadata

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--round', type=int, default=11,choices=[1,2,5,6,7,8,9,10,11]); args=parser.parse_args()
    geo=np.load(ROOT/'outputs/room_quality_ceiling/geometry.npz')
    variants = [
        dict(name='A_顶面连续',envelope_cells=11,roof_area=1.2,extra_angles=False,top_gap=.3,seed_length=.3,height=.7,seed_fraction=.12),
        dict(name='B_严格顶部',envelope_cells=15,roof_area=2.,extra_angles=True,top_gap=.22,seed_length=.4,height=.8,seed_fraction=.2),
        dict(name='C_矮墙兼容',envelope_cells=11,roof_area=1.2,extra_angles=True,top_gap=.35,seed_length=.25,height=.5,seed_fraction=.15),
    ]
    if args.round>=2:
        variants=[
            dict(name='D_三维顶面邻接',envelope_cells=11,roof_area=1.5,extra_angles=False,top_gap=.27,seed_length=.3,height=.5,seed_fraction=.12,roof_planes=True,roof_min_z=1.35),
            dict(name='E_逐列连续',envelope_cells=11,roof_area=1.5,extra_angles=False,top_gap=.3,seed_length=.3,height=.5,seed_fraction=.12,roof_planes=True,roof_min_z=1.35,column_filter=True,column_height=.6),
            dict(name='F_保守顶面',envelope_cells=11,roof_area=2.5,extra_angles=False,top_gap=.24,seed_length=.4,height=.65,seed_fraction=.2,roof_planes=True,roof_min_z=1.65,column_filter=True,column_height=.8),
        ]
    if args.round>=5:
        variants=[
            dict(name='M_斜顶局部交线',envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.3,seed_length=.3,height=.3,seed_fraction=.15,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.45),
            dict(name='N_斜顶相对墙高',envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.3,seed_length=.3,height=.3,seed_fraction=.15,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.45,relative_column=.45,lower_limit=1.),
            dict(name='O_严格连续墙面',envelope_cells=11,roof_area=3.,extra_angles=False,top_gap=.25,seed_length=.4,height=.4,seed_fraction=.2,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.4,sloped_envelope=True,roof_xy_gap=.35,relative_column=.55,lower_limit=.75),
        ]
    if args.round>=6:
        variants=[
            dict(name='P_面片密度约束',envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.3,seed_length=.3,height=.3,seed_fraction=.15,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.45,face_density=.35),
            dict(name='Q_面片密度宽松',envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.3,seed_length=.3,height=.3,seed_fraction=.15,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.45,face_density=.25),
            dict(name='R_顶部支持加严',envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.25,seed_length=.4,height=.3,seed_fraction=.25,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.35,face_density=.3),
        ]
    if args.round>=7:
        variants=[
            dict(name='S_斜顶与高位水平顶',envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.3,seed_length=.3,height=.3,seed_fraction=.15,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.45,face_density=.25,reject_low_horizontal=True,horizontal_drop=.45),
            dict(name='T_顶部邻接保守',envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.25,seed_length=.4,height=.3,seed_fraction=.2,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.35,face_density=.3,reject_low_horizontal=True,horizontal_drop=.45),
            dict(name='U_顶面片段扩大',envelope_cells=11,roof_area=3.,extra_angles=False,top_gap=.3,seed_length=.3,height=.3,seed_fraction=.15,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.45,face_density=.25,reject_low_horizontal=True,horizontal_drop=.35),
        ]
    if args.round>=8:
        variants=[
            dict(name='V_逐墙平面拟合',envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.3,seed_length=.3,height=.3,seed_fraction=.15,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.45,face_density=.25,reject_low_horizontal=True,horizontal_drop=.45,refine_wall_plane=True,wall_tolerance=.04),
            dict(name='W_平面范围收紧',envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.3,seed_length=.3,height=.3,seed_fraction=.15,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.45,face_density=.25,reject_low_horizontal=True,horizontal_drop=.45,refine_wall_plane=True,wall_tolerance=.028),
            dict(name='X_拟合与邻接加强',envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.25,seed_length=.4,height=.3,seed_fraction=.2,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.35,face_density=.3,reject_low_horizontal=True,horizontal_drop=.45,refine_wall_plane=True,wall_tolerance=.04),
        ]
    if args.round>=9:
        variants=[
            dict(name='Y_按楼层顶面结构',envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.3,seed_length=.3,height=.3,seed_fraction=.15,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.45,face_density=.25,reject_low_horizontal=True,horizontal_drop=.45,refine_wall_plane=True,wall_tolerance=.04),
            dict(name='Z_按楼层顶面保守',envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.25,seed_length=.4,height=.3,seed_fraction=.2,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.35,face_density=.3,reject_low_horizontal=True,horizontal_drop=.45,refine_wall_plane=True,wall_tolerance=.04),
        ]
    if args.round>=10:
        base=dict(envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.3,seed_length=.3,height=.3,seed_fraction=.15,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.45,face_density=.25,reject_low_horizontal=True,horizontal_drop=.45,refine_wall_plane=True,wall_tolerance=.04)
        variants=[dict(base,name='AA_逐列墙顶邻接',local_top_attachment=.4),dict(base,name='AB_逐列墙顶严格',local_top_attachment=.25),dict(base,name='AC_逐列墙顶宽松',local_top_attachment=.55)]
    if args.round>=11:
        variants=[dict(base,name='AD_多方向墙面',local_top_attachment=.4,extra_angles=True),dict(base,name='AE_稀疏顶面邻接',local_top_attachment=.4,roof_xy_gap=.6),dict(base,name='AF_顶面广泛支持',seed_fraction=.3,face_density=.35)]
    dest=OUT/f'round{args.round}'; dest.mkdir(exist_ok=True); metrics=[]
    for f,ground in enumerate(GROUNDS):
        p,n=geo[f'p{f}'],geo[f'n{f}']
        old=np.load(ROOT/'outputs/wall_patch_context'/f'L{f+1}_局部高度宽松.npz')['wall_points']
        sets=[old]; labels=[f'{f+1}层 · 原结果']
        for cfg in variants:
            cfg=cfg.copy()
            if args.round>=9:cfg['flat_ceiling_only']=f<2
            walls,roof,records,roof_metadata=extract(p,n,ground,cfg)
            np.savez_compressed(dest/f'L{f+1}_{cfg["name"]}.npz',wall_points=walls,ceiling_points=roof)
            (dest/f'L{f+1}_{cfg["name"]}_patches.json').write_text(json.dumps(records,indent=2))
            (dest/f'L{f+1}_{cfg["name"]}_roof_planes.json').write_text(json.dumps(roof_metadata,indent=2))
            metric=dict(floor=f+1,method=cfg['name'],points=len(walls),patches=len(records),roof_points=len(roof),config=cfg)
            metrics.append(metric); print(metric,flush=True)
            sets.append(walls); labels.append(f'{f+1}层 · '+cfg['name'])
        render(p,sets,ground,dest/f'L{f+1}_comparison.png',labels)
    (dest/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2))

if __name__=='__main__': main()
