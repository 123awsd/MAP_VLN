from pathlib import Path
import sys,types,json,copy
sys.path[:0]=['/tmp/room-research-deps','/tmp/room-research-rose2/src'];sys.modules['rospy']=types.SimpleNamespace(loginfo=lambda *a:None)
import numpy as np
from rose_v1_repo.fft_structure_extraction import FFTStructureExtraction
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'outputs/room_method_research';out.mkdir(exist_ok=True)
for f in [1,2,3]:
 d=np.load(ROOT/f'outputs/room_quality_fullheight/T_删除前薄墙_L{f}.npz');wall=d['wall'];im=np.where(wall,0,255).astype('uint8');base=FFTStructureExtraction(im,peak_height=.2,par=50);base.process_map();print('directions',f,base.main_directions,flush=True)
 for threshold in [.10,.18,.30]:
  try:
   rose=copy.deepcopy(base);rose.simple_filter_map(threshold);rose.generate_initial_hypothesis_simple();rose.find_walls_flood_filing();a=rose.analysed_map[:wall.shape[0],:wall.shape[1]].astype(bool);np.savez_compressed(out/f'ROSE_{threshold}_L{f}.npz',wall=a,origin=d['origin'],resolution=d['resolution'],domain=d['domain']);print('done',f,threshold,int(a.sum()),flush=True)
  except Exception as e:print('failed',f,threshold,type(e).__name__,str(e),flush=True)
