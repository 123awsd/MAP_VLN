#!/usr/bin/env python3
"""Official HM3D surface samples vs latest actual RGB-D surface reconstruction."""
import hashlib,json
from pathlib import Path
import habitat_sim  # registers Magnum import plugins
import magnum.trade as trade
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from scipy.spatial import cKDTree
root=Path(__file__).resolve().parents[1];out=root/'outputs/consistency_validation_20261015/latest_official_comparison';out.mkdir(parents=True,exist_ok=True)
scene=root/'data/scene_datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb';repair_path=root/'scene_repairs/00337_three_floor_shell/repair.json';repair=json.loads(repair_path.read_text());source_hash=hashlib.sha256(scene.read_bytes()).hexdigest();assert source_hash==repair['source_glb_sha256']
origin=np.array(repair['reference_initial_sensor_habitat_xyz']);transform=np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]]);falcon_origin=np.array([0.,0.,1.]);grounds=[-.155,2.595,5.595];extent=(-10.5,8.5,-2.5,18.5);rng=np.random.default_rng(337);density=1600.
def to_falcon(h):return ((h-origin)@transform.T+falcon_origin).astype(np.float32)
def sample(vertices,triangles,density):
 tri=vertices[triangles];area=.5*np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1);expected=area*density;count=np.floor(expected).astype(int)+(rng.random(len(area))<expected%1);idx=np.repeat(np.arange(len(tri)),count);u=np.sqrt(rng.random(len(idx)));v=rng.random(len(idx));w=np.column_stack((1-u,u*(1-v),u*v));return np.einsum('ni,nij->nj',w,tri[idx]).astype(np.float32)
cache=out/'official_sampled_surfaces.npz'
if cache.exists():reference=np.load(cache)['points']
else:
 manager=trade.ImporterManager();imp=manager.load_and_instantiate('AnySceneImporter');imp.open_file(str(scene));parts=[]
 for i in range(imp.mesh_count):
  mesh=imp.mesh(i)
  if not mesh.has_attribute(trade.MeshAttribute.POSITION):continue
  vert=np.asarray(mesh.attribute(trade.MeshAttribute.POSITION),dtype=np.float64);idx=np.asarray(mesh.indices).reshape(-1)if mesh.is_indexed else np.arange(len(vert));idx=idx[:len(idx)//3*3].reshape(-1,3)
  # Sample only triangles whose FALCON bounding boxes intersect shown crop/bands.
  fv=to_falcon(np.column_stack((vert[:,0],vert[:,2],-vert[:,1])));tri=fv[idx];lo=tri.min(axis=1);hi=tri.max(axis=1);mask=(hi[:,0]>=-8.5)&(lo[:,0]<=10.5)&(hi[:,1]>=-2.5)&(lo[:,1]<=18.5)&(hi[:,2]>=-.005)&(lo[:,2]<=7.795)
  parts.append(sample(fv,idx[mask],density));print('sampled mesh',i,len(parts[-1]),flush=True)
 reference=np.concatenate(parts);np.savez_compressed(cache,points=reference)
actual=np.load(root/'outputs/consistency_validation_20261015/latest_dense_floor_topdown/dense_observed_surfaces.npz')
font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';font_manager.fontManager.addfont(font);plt.rcParams['font.family']=font_manager.FontProperties(fname=font).get_name();plt.rcParams['axes.unicode_minus']=False
floor_data=[]
for i,g in enumerate(grounds):
 keep=(reference[:,2]>=g+.15)&(reference[:,2]<g+2.2)&(reference[:,0]>=-8.5)&(reference[:,0]<10.5)&(reference[:,1]>=-2.5)&(reference[:,1]<18.5);ref=reference[keep];obs=actual[f'floor{i+1}'];dist,_=cKDTree(obs).query(ref,k=1,workers=2);matched=dist<=.08;floor_data.append((ref,obs,matched));print('floor',i+1,'reference',len(ref),'actual',len(obs),'match',matched.mean(),flush=True)
def repair_lines(ax,i):
 for item in repair['objects']:
  lo=np.array(item['falcon_min']);hi=np.array(item['falcon_max']);g=grounds[i]
  if hi[2]<g+.15 or lo[2]>=g+2.2:continue
  # Exact projected AABB documented by repair manifest, not official geometry.
  x=[-lo[0],-hi[0],-hi[0],-lo[0],-lo[0]];y=[lo[1],lo[1],hi[1],hi[1],lo[1]];ax.plot(x,y,color='#a651ac',lw=.65,ls='--',alpha=.8)
def panel(ax,i,col):
 ref,obs,matched=floor_data[i]
 if col==0:ax.scatter(-ref[:,0],ref[:,1],c='#4b5865',s=.34,marker='.',lw=0,rasterized=True)
 elif col==1:ax.scatter(-obs[:,0],obs[:,1],c='#123f5a',s=.45,marker='.',lw=0,rasterized=True)
 else:
  ax.scatter(-ref[~matched,0],ref[~matched,1],c='#b7bcc1',s=.55,marker='.',lw=0,rasterized=True)
  ax.scatter(-ref[matched,0],ref[matched,1],c='#078da7',s=.4,marker='.',lw=0,rasterized=True)
 if col in [0,2]:repair_lines(ax,i)
 ax.set_xlim(extent[:2]);ax.set_ylim(extent[2:]);ax.set_aspect('equal');ax.set_xlabel('−X / m');ax.set_ylabel('Y / m');ax.tick_params(labelsize=8)
 if i==0:ax.set_title(['官方HM3D网格表面采样','最新运行实测稠密点云','官方表面匹配对比'][col],fontsize=13,pad=12)
 if col==0:ax.text(.01,.97,f'{i+1}楼',transform=ax.transAxes,va='top',fontsize=12,bbox={'fc':'white','ec':'none','alpha':.9})
fig,axes=plt.subplots(3,3,figsize=(15,16.4),facecolor='white')
for i in range(3):
 for j in range(3):panel(axes[i,j],i,j)
fig.suptitle('官方场景参考 vs 最新504秒建图｜分层俯视',fontsize=18,y=.985);handles=[Line2D([0],[0],marker='.',color='#078da7',lw=0,label='官方表面距实测点≤8 cm'),Line2D([0],[0],marker='.',color='#b7bcc1',lw=0,label='官方表面未匹配'),Line2D([0],[0],color='#a651ac',ls='--',label='实验修复几何投影边界（非官方）')];fig.legend(handles=handles,loc='lower center',bbox_to_anchor=(.5,.035),ncol=3,frameon=False,fontsize=10);fig.text(.5,.014,'统一−X/Y朝向和高度切片；官方采样仅作离线参考。表面距离匹配≠自由空间覆盖率，未匹配点也不一定可观测。',ha='center',fontsize=9,color='#555555');fig.subplots_adjust(left=.045,right=.985,bottom=.075,top=.945,wspace=.12,hspace=.16);fig.savefig(out/'official_vs_latest_three_floors.png',dpi=180,facecolor='white');plt.close(fig)
for i in range(3):
 fig,axes=plt.subplots(1,3,figsize=(15,6.5),facecolor='white')
 for j,ax in enumerate(axes):panel(ax,i,j);ax.set_title(['官方网格采样','最新实测稠密点云','表面匹配对比'][j],fontsize=13)
 fig.suptitle(f'{i+1}楼｜官方参考与最新建图',fontsize=16,y=.99);fig.legend(handles=handles,loc='lower center',bbox_to_anchor=(.5,.025),ncol=3,frameon=False,fontsize=9);fig.subplots_adjust(left=.045,right=.985,bottom=.13,top=.9,wspace=.14);fig.savefig(out/f'floor{i+1}_official_comparison.png',dpi=200,facecolor='white');plt.close(fig)
meta={'official_mesh':str(scene),'official_mesh_sha256':source_hash,'reference_is_sampled_from_mesh_not_provided_official_pointcloud':True,'surface_density_points_per_m2':density,'transform_habitat_to_falcon':transform.tolist(),'initial_sensor_habitat':origin.tolist(),'display_projection':'-X/Y','repair_overlay_manifest':str(repair_path),'repair_overlay_is_not_official_geometry':True,'surface_match_radius_m':.08,'floor_slices':[{'floor':i+1,'z_band_m':[g+.15,g+2.2],'reference_points':len(floor_data[i][0]),'observed_points':len(floor_data[i][1]),'reference_surface_distance_match_fraction':float(floor_data[i][2].mean())}for i,g in enumerate(grounds)],'limits':['Not an official standalone GT pointcloud: reference sampled directly from original GLB.','Raw official scene differs from experiment repair overlays; dashed repair bounds identify that distinction.','3D surface nearest-neighbor threshold is visualization correspondence, not volumetric exploration completeness or navigable-room coverage.','No ICP or fitted alignment: fixed recorded coordinate transform only.']};(out/'comparison_metadata.json').write_text(json.dumps(meta,indent=2));print(out/'official_vs_latest_three_floors.png',flush=True)
