"""Independent measured-wall trial. Strong full-height plane support may replace missing roof attachment.
No filling of missing points, no ROI/room-specific rule. Baseline extractor unchanged.
"""
import numpy as np
import cv2
from scipy.ndimage import maximum_filter
from scipy.spatial import cKDTree
from refine_measured_walls import runs
def extract_with_evidence(p, n, ground, cfg):
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
    audit = cfg.get("audit", {})
    def mark(reason, indices):
        if reason not in audit: audit[reason]=np.zeros(len(p),bool)
        audit[reason][indices]=True
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
                structural = (np.ptp(zz[member]) >= 1.8 and np.quantile(zz[member],.1)<.6
                              and np.quantile(zz[member],.9)>1.8 and len(columns)*.1>=1.)
                alternative = cfg.get('structural_support',False) and structural
                if len(seedcols)*.1 < cfg['seed_length'] and not alternative:
                    mark('component_top_seed',source[ids[member]]);continue
                if np.ptp(zz[member]) < cfg['height']: continue
                if len(columns)*.1 < .5: continue
                density=float(stats[k,cv2.CC_STAT_AREA]/max(1,stats[k,cv2.CC_STAT_WIDTH]*stats[k,cv2.CC_STAT_HEIGHT]))
                if density<cfg.get('face_density',0):continue
                # Near-top attachment must have breadth, not a single accidental point.
                if len(seedcols)/len(columns) < cfg['seed_fraction'] and not alternative:
                    mark('component_top_fraction',source[ids[member]]);continue
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
                        mark('local_top_attachment', source[ids[pointkeep & allowed[ti] & ~attached[ti]]])
                        if alternative:
                            # Require actual observations at low and high elevations in the same column.
                            fullheight=np.zeros(nt,bool)
                            for col in columns:
                                zcol=zz[member&(abs(ti-col)<=1)]
                                fullheight[col]=len(zcol)>=6 and np.quantile(zcol,.1)<.6 and np.quantile(zcol,.9)>1.8
                            attached |= fullheight
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
