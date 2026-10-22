"""Offline adapter around author's ROSE/ROSE2 core, no ROS algorithm replacement."""
from pathlib import Path
import sys,types,json,time,traceback,os,argparse
os.environ['MPLCONFIGDIR']='/tmp/rose2_mpl';os.environ['MPLBACKEND']='Agg'
sys.path.insert(0,'/tmp/rose2_deps');root=Path('outputs/room_rose2_latest_trial');src=root/'reference/ROSE2-main/src';sys.path.insert(0,str(src.resolve()))
ros=types.ModuleType('rospy');ros.loginfo=lambda msg,*a:print(msg%a if a else msg,flush=True);ros.logwarn=ros.loginfo;ros.logerr=ros.loginfo;ros.get_param=lambda key,default=None:default;sys.modules['rospy']=ros
import numpy as np,cv2
from rose_v1_repo.fft_structure_extraction import FFTStructureExtraction
from rose_v2_repo import minibatch,parameters
parser=argparse.ArgumentParser();parser.add_argument('--voronoi',action='store_true');args=parser.parse_args()
summary=[]
for f in [1,2,3]:
 folder=root/(f'L{f}_voronoi' if args.voronoi else f'L{f}');folder.mkdir(exist_ok=True);t=time.monotonic()
 try:
  d=np.load(root/f'L{f}_occupancy.npz');grid=d['grid'];original=np.full(grid.shape,200,np.uint8);original[grid==0]=255;original[grid==100]=0
  original=np.flip(original,0).copy()
  rose=FFTStructureExtraction(original,peak_height=.2,par=50);rose.process_map();rose.simple_filter_map(.18);rose.generate_initial_hypothesis_simple()
  try:rose.find_walls_flood_filing()
  except ValueError as e:print('Author optional ROSE wall-hull visualization fails:',e,flush=True)
  clean=rose.analysed_map[:grid.shape[0],:grid.shape[1]].astype(np.uint8);cv2.imwrite(str(folder/'structure_clean.png'),255-clean*255)
  obj=parameters.ParameterObj();obj.comp=list(rose.main_directions);model=minibatch.Minibatch();model.start_main(parameters,obj,clean,original,str(folder.resolve())+'/',args.voronoi)
  rooms=model.rooms_th1;labels=np.full(grid.shape,-1,np.int32);polygons=[]
  for i,poly in enumerate(rooms or []):
   corners=np.array(poly.exterior.coords);corners[:,1]-=1;polygons.append(corners.tolist());mask=np.zeros(grid.shape,np.uint8);cv2.fillPoly(mask,[np.rint(corners).astype(np.int32)],1);labels[(mask>0)&(grid==0)]=i
  np.savez_compressed(folder/'segmentation.npz',labels=labels,grid=grid,origin=d['origin'],resolution=d['resolution'],clean_structure=clean)
  record={'floor':f,'rooms':len(rooms or []),'elapsed_s':time.monotonic()-t,'dominant_directions':list(map(float,rose.main_directions)),'free_cells':int((grid==0).sum()),'labeled_free_cells':int((labels>=0).sum()),'voronoi':args.voronoi,'status':'completed','polygons_pixel_xy':polygons};(folder/'metrics.json').write_text(json.dumps(record,indent=2));summary.append(record)
 except Exception as e:
  traceback.print_exc();summary.append({'floor':f,'status':'failed','error':str(e)})
(root/('voronoi_summary.json' if args.voronoi else 'summary.json')).write_text(json.dumps(summary,indent=2));print('RESULT',[(r['floor'],r['status'],r.get('rooms')) for r in summary],flush=True)
