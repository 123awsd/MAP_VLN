"""Same-run boundary ablation and observation audit of current 1-02; no relabeling."""
from pathlib import Path
import json,numpy as np,cv2
from scipy import ndimage as nd
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'partition_102_test';O.mkdir(exist_ok=True);run=R/'outputs/coverage_lifecycle_700_r1_20261016'
a=np.load(B/'wall_guided/measured/L1_rooms.npz');base=np.load(B/'surface_supported/measured/L1_rooms.npz');labels=a['labels'];m=labels==2;org=a['origin_xy'];res=float(a['resolution']);wall=a['wall_grid']>0;current=a['wall_barrier'];ring=nd.binary_dilation(m)&~m;distance=nd.distance_transform_edt(m);sy,sx=np.unravel_index(distance.argmax(),distance.shape);seed=(sy,sx)
cc,n=nd.label(m);parts=[{'cells':int((cc==i).sum()),'area_m2':round((cc==i).sum()*res**2,4)}for i in range(1,n+1)];parts.sort(key=lambda x:-x['cells']);variations=[]
for k in [1,3,7]:
 barrier=wall if k==1 else cv2.morphologyEx(wall.astype('uint8'),cv2.MORPH_CLOSE,np.ones((k,k),np.uint8))>0
 domain=base['domain']&~barrier;components,num=nd.label(domain);sid=components[seed];component=(components==sid)if sid else np.zeros_like(m)
 neighbor_ids=[int(i)for i in np.unique(labels[component])if i>0]
 variations.append({'close_kernel_cells':k,'component_area_m2':round(component.sum()*res**2,4),'existing_partition_ids_in_component':neighbor_ids,'target_cells_retained':int((component&m).sum()),'boundary_closed_cells':int((barrier&~wall&ring).sum())})
traj=np.load(run/'video_cache/trajectory.npy');t=traj[(traj[:,3]>.195)&(traj[:,3]<2.045)];ij=np.floor((t[:,1:3]-org)/res).astype(int);ok=(ij[:,0]>=0)&(ij[:,0]<m.shape[1])&(ij[:,1]>=0)&(ij[:,1]<m.shape[0]);t,ij=t[ok],ij[ok];hits=m[ij[:,1],ij[:,0]];y,x=np.where(m);xy=org+np.column_stack([x+.5,y+.5])*res;nearest=cKDTree(xy).query(t[:,1:3])[0]
mapdata=np.load(run/'observed_grid_final.npz');free=mapdata['free'];occ=mapdata['occupied'];evidence={};freeprojection=np.zeros_like(m)
for name,points in [('free',free),('occupied',occ)]:
 q=points[(points[:,2]>.195)&(points[:,2]<1.445)];ij=np.floor((q[:,:2]-org)/res).astype(int);valid=(ij[:,0]>=0)&(ij[:,0]<m.shape[1])&(ij[:,1]>=0)&(ij[:,1]<m.shape[0]);ij=ij[valid];hit=m[ij[:,1],ij[:,0]];evidence[name+'_voxel_count_in_height_band']=int(hit.sum());covered=np.zeros_like(m);covered[ij[hit,1],ij[hit,0]]=True;evidence[name+'_xy_covered_area_m2']=round(covered.sum()*res**2,4)
 if name=='free':freeprojection=covered
r=np.load(B/'evidence_test_results/L1_boundary_evidence.npz');votes=r['free_ray_frame_votes'];nearwall=nd.binary_dilation(m,iterations=2)&~m
report={'target':'1-02','mask_connected_components':parts,'raw_boundary_wall_cells':int((ring&wall).sum()),'postclose_boundary_wall_cells':int((ring&current).sum()),'boundary_ring_cells':int(ring.sum()),'boundary_close_only_cells':int((ring&current&~wall).sum()),'component_ablation':variations,'actual_trajectory_samples_in_partition':int(hits.sum()),'minimum_actual_trajectory_xy_distance_to_partition_m':float(nearest.min()),'actual_inside_times_s':t[hits,0].tolist(),'final_map_height_band_relative_to_floor_m':[.35,1.6],**evidence,'neighbor_wall_cells_crossed_at_least3_frames_same_height_band':int((nearwall&wall&(votes>=3)).sum()),'classification':'not inferred; no independent semantic room annotation','limitations':['final observed voxels are final-state evidence, not time-local topology','no depth crossing does not establish impassable wall','component ablation changes processing only, not physical geometry','a room/corridor/outdoor label cannot be proven from compactness alone']}
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(1,3,figsize=(15,7));geo=np.load(run/'original_style_video/geometry.npz');p=geo['p0'];p=p[(p[:,2]>.0)&(p[:,2]<2.045)]
for j,ax in enumerate(axs):
 ax.scatter(-p[:,0],p[:,1],s=.25,c='#b7c3cd',lw=0)
 rgba=np.zeros((*m.shape,4));rgba[m]=[1,.55,.1,.5];ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5]);ax.set(xlim=(-3.6,1.8),ylim=(3.3,9.6),aspect='equal',xlabel='−X / m',ylabel='Y / m')
 if j==0:
  ax.scatter(-t[:,1],t[:,2],s=3,c='#13649b');ax.set_title('1-02位置与本轮实际轨迹')
 if j==1:
  y,x=np.where(wall);ax.scatter(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,s=2,c='#b02336');y,x=np.where(current&~wall);ax.scatter(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,s=4,c='#1463dc');ax.set_title('红：原墙点；蓝：闭运算新增')
 if j==2:
  y,x=np.where(freeprojection);ax.scatter(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,s=3,c='#008d73');ax.set_title('绿色：最终地图中的FREE投影')
fig.suptitle('1-02边界与实际观测测试：橙色为该分区，非预设房间类型',fontsize=14);fig.tight_layout(rect=[0,.06,1,.94]);fig.text(.5,.02,'同一700秒运行；FREE投影和轨迹仅用于核查，不是语义标签，也不是完整三维通行证明。',ha='center',fontsize=10);fig.savefig(O/'boundary_and_observation_test.png',dpi=170);(O/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False,indent=2))
