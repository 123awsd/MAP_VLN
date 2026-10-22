"""One-room measured RGB + existing Boxer evidence semantic smoke test."""
from pathlib import Path
import base64, json, urllib.request, urllib.error, hashlib
import numpy as np
from PIL import Image

R=Path(__file__).resolve().parents[1]
O=R/'outputs/stage1_pipeline_flow_20261019/room_2_05_semantic_test'
O.mkdir(parents=True,exist_ok=True)
frames=[1280,1360]
source=R/'outputs/boxer/coverage700_measured_20261017/episode'
room=np.load(R/'outputs/coverage700_wall_room_preview/wall_structural_rooms/L2_constrained.npz')
lab=room['labels'];org=room['origin_xy'];res=float(room['resolution'])
inputs=[]
for number in frames:
 p=source/f'frame_{number:06d}.npz'
 with np.load(p) as z:
  pos=z['position'].astype(float);rgb=z['rgb'].copy()
 ij=np.floor((pos[:2]-org)/res).astype(int)
 assert 0<=ij[0]<lab.shape[1] and 0<=ij[1]<lab.shape[0] and int(lab[ij[1],ij[0]])==5
 dest=O/f'view_{number}.jpg';Image.fromarray(rgb).save(dest,quality=92)
 inputs.append({'frame':p.name,'image':str(dest.relative_to(R)),'position_xyz_m':pos.tolist(),'label_id':int(lab[ij[1],ij[0]]),'sha256':hashlib.sha256(dest.read_bytes()).hexdigest()})
box=json.loads((R/'outputs/boxer/coverage700_sam31_20261018/fusion_fixed/boxer_summary.json').read_text())
objects=[{'id':v['id'],'label':v['label'],'probability':v['probability'],'center_xyz':v['center_xyz']}for v in box['objects']if v.get('room_id')=='2-05']
assert objects
prompt={'room_id':'2-05','geometry_note':'自动几何分区；房间编号不是语义标签','boxer_objects':objects,'task':'结合两张同房间实测RGB与物体清单，判断房间语义。只能返回 bedroom, living_room, kitchen, dining_room, bathroom, office, corridor, unknown 之一；若图片与物体矛盾或证据不足输出 unknown。不要修改房间边界或物体。','return_format':{'room_type':'str','confidence':'number 0..1','visible_evidence':'list of short strings','boxer_evidence_ids':'list of object ids','limitations':'list of strings'}}
content=[{'type':'text','text':json.dumps(prompt,ensure_ascii=False)}]
for i,v in enumerate(inputs,1):
 data=(R/v['image']).read_bytes()
 content.append({'type':'text','text':f'同一房间的实测视角{i}'})
 content.append({'type':'image_url','image_url':{'url':'data:image/jpeg;base64,'+base64.b64encode(data).decode('ascii')}})
key=(R/'.secrets/dashscope_api_key').read_text().strip()
req=urllib.request.Request('https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions',data=json.dumps({'model':'qwen3-vl-plus','messages':[{'role':'system','content':'你是室内房间语义分类器。仅依据提供的两张真实图片和Boxer物体列表，严格返回JSON。不得因为room_id推断类别；证据不足返回unknown。'},{'role':'user','content':content}],'response_format':{'type':'json_object'},'enable_thinking':False,'temperature':0,'max_tokens':700},ensure_ascii=False).encode(),headers={'Authorization':'Bearer '+key,'Content-Type':'application/json'},method='POST')
with urllib.request.urlopen(req,timeout=120)as response:body=json.load(response)
raw=body['choices'][0]['message']['content'];result=json.loads(raw)
assert result.get('room_type') in {'bedroom','living_room','kitchen','dining_room','bathroom','office','corridor','unknown'}
assert 0<=float(result.get('confidence',-1))<=1
record={'format':'single_room_multiview_semantic_test.v1','model':body.get('model','qwen3-vl-plus'),'room_id':'2-05','inputs':inputs,'boxer_objects':objects,'result':result,'usage':body.get('usage',{}),'uses_gt':False,'geometry_changed':False,'detector_rerun':False}
(O/'result.json').write_text(json.dumps(record,ensure_ascii=False,indent=2))
print(json.dumps({'room_type':result['room_type'],'confidence':result['confidence'],'objects':len(objects),'output':str(O/'result.json')},ensure_ascii=False))
