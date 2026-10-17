"""Same-data preliminary benchmark; no semantic room GT or accuracy claims."""
from pathlib import Path
import sys,os,time,json,argparse
os.environ['MPLCONFIGDIR']='/tmp/room_benchmark_mpl';sys.path.insert(0,'/tmp/rose2_deps')
import numpy as np,cv2
from scipy import ndimage as nd
from skimage.segmentation import watershed
ap=argparse.ArgumentParser();ap.add_argument('--radius',type=float,default=.45);args=ap.parse_args();radius=args.radius
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'algorithm_comparison' if radius==.45 else B/'algorithm_comparison'/f'radius_{radius:.2f}';O.mkdir(exist_ok=True);geo=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz');rows=[]
for f,g in enumerate([-.155,2.595,5.595],1):
 a=np.load(B/'wall_guided/measured'/f'L{f}_rooms.npz');p=geo[f'p{f-1}'];org=a['origin_xy'];res=float(a['resolution']);shape=a['labels'].shape
 def raster(q):
  m=np.zeros(shape,np.uint8);ij=np.floor((q[:,:2]-org)/res).astype(int);ok=(ij[:,0]>=0)&(ij[:,0]<shape[1])&(ij[:,1]>=0)&(ij[:,1]<shape[0]);ij=ij[ok];m[ij[:,1],ij[:,0]]=1;return m
 footprint=cv2.morphologyEx(raster(p),cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))>0;common=nd.binary_fill_holes(footprint)
 raw=cv2.morphologyEx(raster(p[(p[:,2]>=g+.8)&(p[:,2]<g+1.2)]),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))>0
 wall=cv2.morphologyEx(a['wall_grid'],cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))>0
 np.savez_compressed(O/f'L{f}_input.npz',domain=common,raw_occupancy=raw,extracted_wall=wall,origin_xy=org,resolution=res)
 for method,barrier in [('raw_distance',raw),('raw_morphological',raw),('extracted_wall_distance',wall)]:
  start=time.perf_counter();free=common&~barrier;distance=nd.distance_transform_edt(free)*res
  cores=distance>radius if method!='raw_morphological' else nd.binary_opening(free,structure=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*round(radius/res)+1,2*round(radius/res)+1))>0)
  cc,num=nd.label(cores);seeds=np.zeros(shape,np.int32);count=0
  for k in range(1,num+1):
   m=cc==k
   if m.sum()*res**2>=.04:count+=1;seeds[m]=count
  cc,num=nd.label(free)
  for k in range(1,num+1):
   m=cc==k
   if not seeds[m].any() and m.sum()*res**2>=.3:
    count+=1;y,x=np.where(m);idx=distance[y,x].argmax();seeds[y[idx],x[idx]]=count
  labels=watershed(-distance,seeds,mask=free,connectivity=1).astype('int32');elapsed=time.perf_counter()-start
  np.savez_compressed(O/f'L{f}_{method}.npz',labels=labels,domain=free,barrier=barrier,origin_xy=org,resolution=res)
  areas=[(labels==k).sum()*res**2 for k in np.unique(labels)if k>0];rows.append({'floor':f,'method':method,'regions':len(areas),'small_regions_below1m2':sum(x<1 for x in areas),'largest_region_m2':round(max(areas,default=0),3),'labeled_area_m2':round(sum(areas),3),'segmentation_seconds':round(elapsed,5),'note':'excludes 3D cloud/wall preparation; counts are not accuracy'})
(O/'baseline_metrics.json').write_text(json.dumps({'common_domain':'full-slab measured point projection, 35cm close then enclosed-hole fill; identical across methods, no GT','occupancy_slab_relative_m':[.8,1.2],'distance_radius_m':radius,'minimum_seed_area_m2':.04,'methods':'local conventional baselines, not author implementations of specific papers; extracted_wall_distance is controlled component comparison not exact previous preview','results':rows},indent=2,default=lambda x:x.item()));print(json.dumps(rows,default=lambda x:x.item()))
