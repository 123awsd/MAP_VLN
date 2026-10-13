from pathlib import Path
import sys,types,json
sys.path[:0]=['/tmp/room-research-deps','/tmp/room-research-rose2/src'];sys.modules['rospy']=types.SimpleNamespace(loginfo=lambda *a:print(*a,flush=True))
# Disable optional ROS, plotting and Voronoi integration; structural algorithm unchanged.
sys.modules['util.voronoi']=types.ModuleType('util.voronoi');sys.modules['descartes']=types.SimpleNamespace(PolygonPatch=lambda *a,**k:None)
import shapely.ops
shapely.ops.cascaded_union=shapely.ops.unary_union
import numpy as np,cv2
from rose_v2_repo import minibatch,parameters
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'outputs/room_method_research';font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',30)
for f in [1,2,3]:
 d=np.load(ROOT/f'outputs/room_quality_fullheight/T_删除前薄墙_L{f}.npz');clean=np.load(out/f'ROSE_0.1_L{f}.npz')['wall'];original=np.where(d['wall'],0,255).astype('uint8');original[~(d['domain']|d['wall'])]=0
 original=cv2.cvtColor(original,cv2.COLOR_GRAY2RGB);par=parameters.ParameterObj();par.comp=[np.pi/2,-np.pi/2,0,np.pi];model=minibatch.Minibatch()
 try:
  model.start_main(parameters,par,clean.astype('uint8'),original,str(out)+'/',False)
  rooms=model.rooms_th1 or [];records=[];mask=np.zeros(d['wall'].shape,np.uint8)
  for room in rooms:
   coords=np.array(room.exterior.coords);records.append(coords.tolist());cv2.polylines(mask,[coords.astype('int32')],True,1,1)
  np.savez_compressed(out/f'ROSE2_correct_dirs_L{f}.npz',wall=mask.astype(bool),origin=d['origin'],resolution=d['resolution']);(out/f'ROSE2_correct_dirs_L{f}_rooms.json').write_text(json.dumps(records));print('ROOMS',f,len(rooms),flush=True)
 except Exception as e:
  import traceback;traceback.print_exc()
