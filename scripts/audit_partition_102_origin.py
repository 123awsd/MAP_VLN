"""Reproduce exact current L1 partition and audit origin of region 1-02; read-only inputs."""
from pathlib import Path
import ast,heapq,json
import numpy as np,cv2
from scipy import ndimage as nd
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';src=R/'scripts/segment_rooms_wall_guided.py'
a=np.load(B/'surface_supported/measured/L1_rooms.npz');saved=np.load(B/'wall_guided/measured/L1_rooms.npz');res=float(a['resolution']);org=a['origin_xy'];wall=a['wall_grid'];raw=cv2.morphologyEx(wall,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))>0;closed=cv2.morphologyEx(wall,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))>0;enclosed=nd.binary_fill_holes(closed)&~closed;domain=(a['domain']|enclosed)&~raw;dist=nd.distance_transform_edt(domain)*res;cores,n=nd.label(dist>.45);seed=np.zeros_like(a['labels']);count=0;seed_records=[]
for k in range(1,n+1):
 m=cores==k;area=m.sum()*res**2
 if area>=.04:count+=1;seed[m]=count;seed_records.append({'seed_id':count,'source':'distance_core','core_area_m2':float(area)})
cc,n=nd.label(domain)
for k in range(1,n+1):
 m=cc==k
 if not seed[m].any() and m.sum()*res**2>=.3:count+=1;seed[m&(dist>=dist[m].max()-.05)]=count;seed_records.append({'seed_id':count,'source':'unseeded_component_fallback'})
module=ast.parse(src.read_text());fn=next(v for v in module.body if isinstance(v,ast.FunctionDef)and v.name=='watershed');ns={'np':np,'heapq':heapq};exec(compile(ast.Module(body=[fn],type_ignores=[]),str(src),'exec'),ns);labels=ns['watershed'](dist,seed,domain);merged=[]
for _ in range(3):
 changed=False
 for rid in np.unique(labels):
  if not rid:continue
  region=labels==rid;ring=nd.binary_dilation(region)&~region;neighbor=np.unique(labels[ring]);neighbor=neighbor[(neighbor>0)&(neighbor!=rid)];wallfrac=float(raw[ring].mean())if ring.any()else 0;touches=bool(np.any(ring&~domain&~raw))
  if len(neighbor)==1 and wallfrac<.2 and not touches:
   target=int(neighbor[0]);merged.append({'from':int(rid),'to':target});labels[region]=target;changed=True
 if not changed:break
new=np.zeros_like(labels);mapping={}
for k,rid in enumerate(np.unique(labels[labels>0]),1):new[labels==rid]=k;mapping[k]=int(rid)
assert np.array_equal(new,saved['labels']),'Must reproduce actual current mask exactly'
m=new==2;ring=nd.binary_dilation(m)&~m;ys,xs=np.where(m);original=mapping[2];ownseed=seed==original
geo=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz');p,n=geo['p0'],geo['n0'];ij=np.floor((p[:,:2]-org)/res).astype(int);valid=(ij[:,0]>=0)&(ij[:,0]<m.shape[1])&(ij[:,1]>=0)&(ij[:,1]<m.shape[0]);p,n,ij=p[valid],n[valid],ij[valid];q=p[m[ij[:,1],ij[:,0]]];nn=n[m[ij[:,1],ij[:,0]]]
near_floor=(abs(q[:,2]+.155)<.12)&(abs(nn[:,2])>.75)
report={'exact_reproduction':True,'display_id':'1-02','source_seed_id':original,'source_seed':next(r for r in seed_records if r['seed_id']==original),'area_m2':round(m.sum()*res**2,4),'max_clearance_to_obstacle_m':float(dist[m].max()),'seed_core_count':int(nd.label(ownseed)[1]),'bbox_raw_xy_m':[float(org[0]+xs.min()*res),float(org[0]+(xs.max()+1)*res),float(org[1]+ys.min()*res),float(org[1]+(ys.max()+1)*res)],'neighbor_labels_touching_without_wall':[int(k)for k in np.unique(new[ring])if k>0],'boundary_wall_cell_fraction':round(float(raw[ring].mean()),4),'area_added_by_wall_enclosure_fill_m2':round((m&~a['domain']).sum()*res**2,4),'measured_points_in_projection':len(q),'near_floor_horizontal_points_in_projection':int(near_floor.sum()),'merges':merged,'algorithm_has_room_corridor_semantic_classifier':False,'algorithm_has_portal_detector':False,'classification':'untyped geometric partition, not verified room or corridor','causal_chain':['non-wall supported domain','distance greater than .45m yields connected core','core area threshold .04m2 admits seed','masked watershed assigns surrounding cells','all positive labels are exported and numbered without semantic room validation'],'limitation':'shape and adjacency alone do not establish architectural room/corridor meaning; no manual semantic annotation used'}
(B/'partition_102_origin_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False,indent=2))
