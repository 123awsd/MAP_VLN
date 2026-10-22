"""Complete offline measured room pipeline on newly reconstructed surface/rays.
Not historical TSDF replay; exports explicitly omit stale FREE protection.
"""
from pathlib import Path
import numpy as np,json,cv2
from scipy.spatial import cKDTree
from scipy import ndimage as nd
from extract_walls_structural_evidence import extract_with_evidence
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'shell_filtered_final';raw=np.load(O/'reconstructed_evidence.npz');points=raw['points'];data={};walls={};domains={}
cfg=dict(envelope_cells=11,roof_area=2.,extra_angles=False,top_gap=.3,seed_length=.3,height=.3,seed_fraction=.15,roof_planes=True,roof_min_z=.85,column_filter=True,column_height=.3,sloped_envelope=True,roof_xy_gap=.6,face_density=.25,reject_low_horizontal=True,horizontal_drop=.45,refine_wall_plane=True,wall_tolerance=.04,local_top_attachment=.4,structural_support=True)
def raster(p):
 a=np.zeros((420,380),np.uint8);ij=np.floor((p[:,:2]-[-8.5,-2.5])/.05).astype(int);v=(ij[:,0]>=0)&(ij[:,0]<380)&(ij[:,1]>=0)&(ij[:,1]<420);ij=ij[v];a[ij[:,1],ij[:,0]]=1;return a
for fi,(g,lo,hi) in enumerate(zip([-.155,2.595,5.595],[-.25,2.5,5.5],[2.5,5.5,8.05])):
 p=points[(points[:,2]>lo)&(points[:,2]<hi)];tree=cKDTree(p);n=np.empty_like(p)
 for start in range(0,len(p),5000):
  _,ix=tree.query(p[start:start+5000],k=20,workers=2);near=p[ix];near=near-near.mean(1,keepdims=True);_,v=np.linalg.eigh(np.einsum('nki,nkj->nij',near,near));n[start:start+len(ix)]=v[:,:,0]
 data[f'p{fi}']=p;data[f'n{fi}']=n
 w,_,_,_=extract_with_evidence(p,n,g,{**cfg,'flat_ceiling_only':fi<2});walls[fi+1]=w
 support=(abs(n[:,2])>=.35)&(p[:,2]>=g-.15)&(p[:,2]<=g+2.85);domain=cv2.morphologyEx(raster(p[support]),cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9)))>0
 # Fresh surface footprint, same bounded gap fill as the existing surface stage.
 holes=nd.binary_fill_holes(domain)&~domain;cc,num=nd.label(holes)
 for k in range(1,num+1):
  m=cc==k
  if m.sum()*.05**2<=1:domain[m]=1
 wall=raster(w);enclosed=nd.binary_fill_holes(cv2.morphologyEx(wall,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))>0)
 domains[fi+1]=domain|enclosed;print('extracted',fi+1,len(p),len(w),flush=True)
np.savez_compressed(O/'geometry.npz',**data)
# Reuse production segmentation tail and renderer; replace inputs explicitly.
s=(R/'scripts/run_wall_structural_rooms.py').read_text();s=s.replace("O=B/'wall_structural_rooms'","O=B/'shell_filtered_final'").replace("G=np.load(run/'original_style_video/geometry.npz')","G=np.load(O/'geometry.npz')")
s=s.replace("B/'independent_room_support'/f'L{f}_constrained.npz'","B/'wall_structural_rooms'/f'L{f}_constrained.npz'")
s=s.replace("nw=np.load(B/'wall_structural_evidence'/f'L{f}_walls.npz')['wall_points']","nw=rebuilt_walls[f]")
s=s.replace('domain=(domain|oldbar)&~bar','domain=rebuilt_domains[f]&~bar')
s=s.replace("free_tree=cKDTree(np.load(run/'observed_grid_final.npz')['free']);", "")
s=s.replace("free_layers=(free_tree.query(probes.reshape(-1,3),workers=2)[0].reshape(-1,4)<=.09).sum(1);", "")
s=s.replace("horizontal surface or multi-height FREE or measured rays or trajectory", "fresh horizontal surface or freshly filtered sampled rays or original trajectory")
s=s.replace("新墙输入＋同一分割流程", "过滤深度全记录离线重建")
s=s.replace("多高度FREE、多帧深度射线", "新有效多帧深度射线")
s=s.replace('protected_free=(free_layers>=2).reshape(shape)','protected_free=np.zeros(shape,bool) # no stale FREE from original map')
s=s.replace("ray_votes=previous['ray_frame_votes']","ray_votes=rebuilt_rays[f-1]")
s=s.replace("'new墙'","'new墙'")
exec(compile(s,str(R/'scripts/run_wall_structural_rooms.py'),'exec'),{'__file__':str(R/'scripts/run_wall_structural_rooms.py'),'rebuilt_walls':walls,'rebuilt_domains':domains,'rebuilt_rays':raw['ray_votes']})
s=(R/'scripts/render_wall_structural_rooms_final.py').read_text().replace('wall_structural_rooms','shell_filtered_final').replace('新墙提取 → 独立观测支持 → 房间分割','安全壳深度过滤 → 全记录离线重建 → 房间分割');exec(compile(s,'render_final','exec'))
meta=json.loads((O/'reconstruction.json').read_text());meta.update({'online_FREE_reused':False,'online_TSDF_replayed':False,'surface_domain_rebuilt':True,'wall_and_normals_recomputed':True,'comparison_caveat':'offline sampled-ray reconstruction differs from original online FREE evidence; changes cannot all be attributed solely to shell filtering'});(O/'final_manifest.json').write_text(json.dumps(meta,indent=2))
