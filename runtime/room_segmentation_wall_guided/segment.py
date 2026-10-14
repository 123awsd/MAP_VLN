from pathlib import Path
import sys,json
sys.path.insert(0,'/tmp/rose2_deps')
import numpy as np,cv2
from scipy.ndimage import distance_transform_edt,binary_closing,binary_fill_holes,label
from skimage.segmentation import watershed
root=Path(__file__).resolve().parents[2];out=root/'outputs/room_wall_guided_trial';rows=[]
for f in [1,2,3]:
 d=np.load(root/'outputs/room_rose2_latest_trial'/f'L{f}_occupancy.npz');grid=d['grid'];origin=d['origin'];res=float(d['resolution']);walls=np.load(root/'outputs/wall_structure_latest_trial/automatic_doors_hough_clean'/f'L{f}_walls.npz')['wall_points'];xy=np.floor((walls[:,:2]-origin)/res).astype(int);valid=(xy[:,0]>=0)&(xy[:,0]<grid.shape[1])&(xy[:,1]>=0)&(xy[:,1]<grid.shape[0]);xy=xy[valid];barrier=np.zeros(grid.shape,bool);barrier[xy[:,1],xy[:,0]]=True
 # Only bounded smoothing of wall sampling, no guessed doorway closures or unknown filling.
 barrier=binary_closing(barrier,structure=np.ones((3,3)))
 observed=(grid==0)|(grid>=50);domain=observed&~barrier
 distance=distance_transform_edt(domain)*res
 for radius in [.30,.45,.60]:
  cores=distance>=radius;markers,num=label(cores);nextid=0;clean=np.zeros_like(markers)
  for k in range(1,num+1):
   m=markers==k
   if m.sum()*res**2<1.:continue
   nextid+=1;clean[m]=nextid
  # Preserve separately observed regions that lack a wide kernel if their area is meaningful.
  components,num=label(domain)
  for k in range(1,num+1):
   m=components==k
   if not np.any(clean[m]) and m.sum()*res**2>=1.:
    yy,xx=np.where(m);best=np.argmax(distance[yy,xx]);nextid+=1;clean[yy[best],xx[best]]=nextid
  labels=watershed(-distance,clean,mask=domain);labels=labels.astype(np.int32)-1;labels[~domain]=-1
  dest=out/f'r{radius:.2f}'/f'L{f}';dest.mkdir(parents=True,exist_ok=True);np.savez_compressed(dest/'segmentation.npz',grid=grid,labels=labels,origin=origin,resolution=res,wall_barrier=barrier)
  areas=[float(np.sum(labels==i)*res**2) for i in np.unique(labels[labels>=0])];rows.append({'floor':f,'core_clearance_m':radius,'regions':len(areas),'areas_m2':areas,'labeled_observed_area_m2':float(np.sum(labels>=0)*res**2),'unknown_colored_cells':int(np.sum((labels>=0)&(grid==-1)))})
(out/'metrics.json').write_text(json.dumps(rows,indent=2));print([(x['floor'],x['core_clearance_m'],x['regions']) for x in rows])
