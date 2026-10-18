"""SAM3.1 mirror detector smoke test on original measured RGB-D; no GT input."""
from pathlib import Path
import sys,json,time,csv,random,argparse
import numpy as np,torch
from PIL import Image
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'third_party/sam3_source'),str(R/'boxer_ext'),str(R/'third_party/boxer')]
parser=argparse.ArgumentParser();parser.add_argument('--full',action='store_true');parser.add_argument('--resume',action='store_true');args=parser.parse_args()
O=R/('outputs/boxer/coverage700_sam31_20261018' if args.full else 'outputs/boxer/sam31_measured_smoke_20261018');O.mkdir(parents=True,exist_ok=True)
from safetensors.torch import load_file
from sam3 import model_builder as mb
from sam3.model.sam3_multiplex_detector import Sam3MultiplexDetector
from sam3.model.vl_combiner import SAM3VLBackboneTri
from sam3.model.sam3_image_processor import Sam3Processor
from torchvision.ops import batched_nms
labels=['door','window','mirror','chair','armchair','stool','bench','sofa','bed','table','coffee table','dining table','desk','cabinet','wardrobe','dresser','nightstand','shelf','bookshelf','television','lamp','refrigerator','oven','microwave','sink','toilet','bathtub','shower','washing machine','plant']
source=R/'outputs/boxer/coverage700_measured_20261017/episode';sequences=[80,260,990,1240,2000,2610,2870,2900]
if args.full:sequences=[int(p.stem.split('_')[-1])for p in sorted(source.glob('frame_*.npz'))]
random.seed(7);np.random.seed(7);torch.manual_seed(7)
ckpt=R/'models/sam31/sam3.1_multiplex.safetensors';print('BUILD_IMAGE_MODEL',flush=True)
bpe=str(R/'third_party/sam3_source/sam3/assets/bpe_simple_vocab_16e6.txt.gz')
visual=mb._create_multiplex_tri_backbone(compile_mode=None,use_fa3=False,use_rope_real=False)
backbone=SAM3VLBackboneTri(scalp=0,visual=visual,text=mb._create_text_encoder(bpe))
model=Sam3MultiplexDetector(num_feature_levels=1,backbone=backbone,transformer=mb._create_sam3_transformer(use_fa3=False),segmentation_head=mb._create_segmentation_head(use_fa3=False),semantic_segmentation_head=None,input_geometry_encoder=mb._create_geometry_encoder(),use_early_fusion=True,use_dot_prod_scoring=True,dot_prod_scoring=mb._create_dot_product_scoring(),supervise_joint_box_scores=True,is_multiplex=True)
all_weights=load_file(str(ckpt),device='cpu');weights={k[len('detector.'):]:v for k,v in all_weights.items()if k.startswith('detector.')};result=model.load_state_dict(weights,strict=True);del all_weights,weights
model=model.cuda().eval();processor=Sam3Processor(model,device='cuda',confidence_threshold=.5);manifest={'model_source':json.loads((R/'models/sam31/manifest.json').read_text()),'code_source':json.loads((R/'third_party/sam3_source/source_revision.json').read_text())['revision'],'detector_weights_strict_load':True,'mode':'SAM3.1 checkpoint image detector; multiplex video tracker not exercised','labels':labels,'threshold':.5,'same_class_nms_iou':.5,'cross_class_near_identical_nms_iou':.9,'frames':[],'uses_gt':False}
previous=json.loads((O/'manifest.json').read_text())if args.resume and (O/'manifest.json').exists()else {}
previous_frames={v['sequence']:v for v in previous.get('frames',[])}
manifest['status']='running';manifest['frames_count']=len(sequences);manifest['source_episode']=str(source)
for sequence in sequences:
 if sequence in previous_frames and (O/f'frame_{sequence:06d}_sam.npz').exists():
  manifest['frames'].append(previous_frames[sequence]);print('RESUME',sequence,flush=True);continue
 path=source/f'frame_{sequence:06d}.npz'
 with np.load(path)as a:rgb=a['rgb'];stamp=int(a['time_ns'])
 start=time.time();boxes=[];scores=[];classes=[];masks=[]
 with torch.inference_mode(),torch.autocast('cuda',dtype=torch.bfloat16):
  state=processor.set_image(Image.fromarray(rgb))
  for i,label in enumerate(labels):
   out=processor.set_text_prompt(label,state)
   if len(out['scores']):
    boxes.append(out['boxes'].float().cpu());scores.append(out['scores'].float().cpu());classes.extend([i]*len(out['scores']));masks.append(out['masks'][:,0].cpu())
 if boxes:
  boxes=torch.cat(boxes);scores=torch.cat(scores);classes=torch.tensor(classes);masks=torch.cat(masks)
  same=batched_nms(boxes,scores,classes,.5);cross=batched_nms(boxes[same],scores[same],torch.zeros(len(same),dtype=torch.long),.9);keep=same[cross];boxes,scores,classes,masks=boxes[keep],scores[keep],classes[keep],masks[keep]
 else:boxes=torch.empty((0,4));scores=torch.empty(0);classes=torch.empty(0,dtype=torch.long);masks=torch.empty((0,*rgb.shape[:2]),dtype=torch.bool)
 np.savez_compressed(O/f'frame_{sequence:06d}_sam.npz',boxes_xyxy=boxes.numpy(),scores=scores.numpy(),label_indices=classes.numpy(),masks=masks.numpy(),time_ns=stamp)
 row={'sequence':sequence,'time_ns':stamp,'detections':len(scores),'seconds':time.time()-start,'labels':[labels[i]for i in classes.tolist()]};manifest['frames'].append(row);(O/'manifest.json').write_text(json.dumps(manifest,indent=2));print(row,flush=True)
manifest['peak_cuda_allocated_gb']=torch.cuda.max_memory_allocated()/1e9
# Release SAM before loading Boxer, keeping GPU memory bounded.
del processor,model
for name in ['state','out']:
 if name in globals():del globals()[name]
torch.cuda.empty_cache()
from habitat_loader import HabitatLoader
from boxernet.boxernet import BoxerNet
from sam2_depth_filter import filter_box_background_depth
from utils.file_io import ObbCsvWriter2
from utils.tw.tensor_utils import string2tensor,pad_string
loader=HabitatLoader(source);loader._prefetch_thread.join();loader.resize=960
boxer=BoxerNet.load_from_checkpoint(str(R/'third_party/boxer/ckpts/boxernet_hw960in2x6d768-c88128f8.ckpt'),device='cuda');writer=ObbCsvWriter2(str(O/'boxer_3dbbs.csv'));two_rows=[]
for record in manifest['frames']:
 seq=record['sequence'];path=source/f'frame_{seq:06d}.npz';idx=loader.frames.index(path);d=loader.load(idx)
 with np.load(O/f'frame_{seq:06d}_sam.npz')as a:bb=a['boxes_xyxy'].copy();scores=a['scores'].copy();ids=a['label_indices'].copy();mask=a['masks'].copy()
 if not len(bb):record['boxer_3d_detections']=0;continue
 import cv2
 scaled=bb*np.array([1.5,2.,1.5,2.]);box=torch.tensor(scaled[:,[0,2,1,3]],dtype=torch.float32);resized=np.stack([cv2.resize(m.astype(np.uint8),(960,960),interpolation=cv2.INTER_NEAREST).astype(bool)for m in mask]);depth,stats=filter_box_background_depth(d['depth0'],box,resized,dilation_pixels=2);p=d['depth_projection'];np.random.seed(7);d['sdp_w']=loader.sdp_from_depth(depth,p['fx'],p['fy'],p['cx'],p['cy'],p['rotation'],p['position']);d['bb2d']=box
 with torch.inference_mode(),torch.autocast('cuda',dtype=torch.bfloat16):obs=boxer.forward(d)['obbs_pr_w'].cpu()[0]
 keep=obs.prob.reshape(-1)>=.2;obs=obs[keep].clone();used=ids[keep.numpy()]
 if len(obs):
  prob=(obs.prob.reshape(-1)+torch.tensor(scores[keep.numpy()]))/2;obs.set_prob(prob);obs.set_sem_id(torch.tensor(used,dtype=torch.int32));obs.set_text(torch.stack([string2tensor(pad_string(labels[i],max_len=128))for i in used]));writer.write(obs,timestamps_ns=record['time_ns'])
 record['boxer_3d_detections']=len(obs);record['depth_filter_stats']=stats
 for b,s,i in zip(bb,scores,ids):two_rows.append({'time_ns':record['time_ns'],'sequence':seq,'name':labels[i],'prob':float(s),'img_width':640,'img_height':480,'x1':float(b[0]),'y1':float(b[1]),'x2':float(b[2]),'y2':float(b[3])})
writer.close()
if two_rows:
 with (O/'sam_2dbbs.csv').open('w')as fp:w=csv.DictWriter(fp,fieldnames=list(two_rows[0]));w.writeheader();w.writerows(two_rows)
manifest['status']='completed';manifest['frames_count']=len(sequences);manifest['detector_seconds_total']=sum(v['seconds']for v in manifest['frames']);(O/'manifest.json').write_text(json.dumps(manifest,indent=2));print('FULL_COMPLETED' if args.full else 'SMOKE_COMPLETED',flush=True)
