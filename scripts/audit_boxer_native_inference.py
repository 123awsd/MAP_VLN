"""Fixed-proposal single-frame inference checks: native depth vs local SAM2 and precision."""
from pathlib import Path
import sys,csv,json,random
import numpy as np,torch
from PIL import Image
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'boxer_ext'),str(R/'third_party/boxer')]
from habitat_loader import HabitatLoader
from sam2_box_segmenter import Sam2BoxSegmenter
from sam2_depth_filter import filter_box_background_depth
from boxernet.boxernet import BoxerNet
O=R/'outputs/boxer/coverage700_measured_20261017';two=list(csv.DictReader((O/'detections/episode/owl_2dbbs.csv').open()));loader=HabitatLoader(O/'episode');loader._prefetch_thread.join();loader.resize=960
model=BoxerNet.load_from_checkpoint(str(R/'third_party/boxer/ckpts/boxernet_hw960in2x6d768-c88128f8.ckpt'),device='cuda');sam=Sam2BoxSegmenter(device='cuda',model_name='facebook/sam2.1-hiera-large');results=[]
for index in [8,99,261]:
 np.random.seed(7);d=loader.load(index);stamp=int(d['time_ns0']);props=[v for v in two if int(v['time_ns'])==stamp];bb=torch.tensor([[float(v[k])for k in ('x1','x2','y1','y2')]for v in props]);d['bb2d']=bb
 rgb=(d['img0'][0].permute(1,2,0).numpy()*255).round().astype(np.uint8);masks=sam.segment(Image.fromarray(rgb),bb[:,[0,2,1,3]].tolist());dep,stats=filter_box_background_depth(d['depth0'],bb,masks,dilation_pixels=2)
 modes={}
 for name,masked,bf16 in [('native_depth_fp32',False,False),('sam_depth_fp32',True,False),('sam_depth_bf16',True,True)]:
  inp=dict(d)
  if masked:
   p=d['depth_projection'];np.random.seed(7);inp['sdp_w']=loader.sdp_from_depth(dep,p['fx'],p['fy'],p['cx'],p['cy'],p['rotation'],p['position'])
  with torch.inference_mode(),torch.autocast('cuda',dtype=torch.bfloat16,enabled=bf16):obb=model.forward(inp)['obbs_pr_w'].cpu()[0]
  modes[name]={'centers':obb.T_world_object.t.float().numpy().tolist(),'sizes':obb.bb3_diagonal.float().numpy().tolist(),'rotations':obb.T_world_object.R.float().numpy().tolist(),'scores3d':obb.prob.float().numpy().reshape(-1).tolist()}
 base=modes['native_depth_fp32'];masked=modes['sam_depth_fp32'];low=modes['sam_depth_bf16'];changes=[]
 for j,v in enumerate(props):
  changes.append({'label':v['name'],'score2d':float(v['prob']),'sam_vs_native_center_delta_m':float(np.linalg.norm(np.array(masked['centers'][j])-base['centers'][j])),'sam_vs_native_max_dimension_delta_m':float(np.max(np.abs(np.array(masked['sizes'][j])-base['sizes'][j]))),'bf16_vs_fp32_center_delta_m':float(np.linalg.norm(np.array(low['centers'][j])-masked['centers'][j])),'bf16_vs_fp32_max_dimension_delta_m':float(np.max(np.abs(np.array(low['sizes'][j])-masked['sizes'][j])))})
 results.append({'frame':loader.frames[index].name,'time_ns':stamp,'changes':changes,'outputs':modes,'sam_depth':stats});print(results[-1]['frame'],changes,flush=True)
(O/'native_inference_audit.json').write_text(json.dumps({'frames':results,'purpose':'sensitivity check only, not GT accuracy comparison','fixed_proposals':True,'no_exploration':True},indent=2))
