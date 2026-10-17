"""Current-run adapter to local author ROSE2; compatibility shims only, report failures."""
from pathlib import Path
import os,sys,types,json,time,traceback
os.environ['MPLCONFIGDIR']='/tmp/room_benchmark_mpl';os.environ['MPLBACKEND']='Agg'
sys.path[:0]=['/tmp/room_benchmark_optional','/tmp/rose2_deps','/home/uav/miniconda3/envs/airsim/lib/python3.10/site-packages','/home/uav/miniconda3/envs/vlm_nav/lib/python3.10/site-packages']
import numpy as np,cv2
import shapely.ops
if not hasattr(shapely.ops,'cascaded_union'):shapely.ops.cascaded_union=shapely.ops.unary_union
import png
ros=types.ModuleType('rospy');ros.loginfo=lambda *a:None;ros.logwarn=ros.loginfo;ros.logerr=ros.loginfo;sys.modules['rospy']=ros
# Non-Voronoi author mode never calls this optional module; fail explicitly if it does.
vmod=types.ModuleType('util.voronoi')
def unavailable(*args,**kwargs):raise RuntimeError('Optional Voronoi branch unavailable; no result may be claimed')
vmod.compute_voronoi_graph=unavailable;vmod.voronoi_segmentation=unavailable;sys.modules['util.voronoi']=vmod
R=Path(__file__).resolve().parents[1];O=R/'outputs/coverage700_wall_room_preview/algorithm_comparison';src=R/'outputs/room_rose2_latest_trial/reference/ROSE2-main/src';sys.path.insert(0,str(src))
f=int(sys.argv[1]);folder=O/f'author_rose2_L{f}';folder.mkdir(exist_ok=True);start=time.perf_counter();record={'floor':f,'method':'author_rose2','source':str(src),'compatibility':['rospy logging stub','shapely cascaded_union alias','optional Voronoi module unavailable; selected author rooms_voronoi=False and fail if called','pypng/descartes loaded from isolated /tmp dependencies'],'orientation':'Cartesian array flipped to image before author processing; author height-y output minus one restores Cartesian grid'}
try:
 from rose_v1_repo.fft_structure_extraction import FFTStructureExtraction
 from rose_v2_repo import minibatch,parameters
 a=np.load(O/f'L{f}_input.npz');domain=a['domain'];raw=a['raw_occupancy'];original=np.full(raw.shape,200,np.uint8);original[domain&~raw]=255;original[raw]=0
 original=np.flipud(original).copy() # author flip_lines uses height-y; polygons below subtract one pixel
 rose=FFTStructureExtraction(original,peak_height=.2,par=50);rose.process_map();rose.simple_filter_map(.18);rose.generate_initial_hypothesis_simple()
 clean=rose.analysed_map[:raw.shape[0],:raw.shape[1]].astype(np.uint8)
 obj=parameters.ParameterObj();obj.comp=list(rose.main_directions);model=minibatch.Minibatch();model.start_main(parameters,obj,clean,original,str(folder.resolve())+'/',False)
 rooms=model.rooms_th1;labels=np.zeros(raw.shape,np.int32)
 for i,poly in enumerate(rooms or [],1):
  corners=np.array(poly.exterior.coords);corners[:,1]-=1;mask=np.zeros(raw.shape,np.uint8);cv2.fillPoly(mask,[np.rint(corners).astype('int32')],1);labels[(mask>0)&domain&~raw]=i
 np.savez_compressed(O/f'L{f}_author_rose2.npz',labels=labels,domain=domain&~raw,barrier=raw,origin_xy=a['origin_xy'],resolution=a['resolution']);record.update(status='completed',rooms=len(rooms or []))
except Exception as e:record.update(status='failed',error=f'{type(e).__name__}: {e}',traceback=traceback.format_exc());traceback.print_exc()
record['elapsed_s']=time.perf_counter()-start;(folder/'status.json').write_text(json.dumps(record,indent=2));print('RESULT',json.dumps(record),flush=True)
