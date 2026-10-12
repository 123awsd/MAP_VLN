from pathlib import Path
import sys,json,subprocess
import numpy as np,cv2
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent;sys.path.insert(0,str(ROOT/'PRE_MAP_VLN/scripts'));import render_exploration_three_view as r
scene=json.loads((ROOT/'PRE_MAP_VLN/outputs/room_pipeline/00337_transition_aware_qwen_v1/multifloor_scene_graph_qwen.json').read_text());floor=scene['floors'][1];p=np.load(ROOT/'视频制作/探索剪辑素材_无字30帧/geometry.npz')['p1'];BG=(18,21,26);W,H=1920,1080
roomcolors=[(110,163,190),(128,151,195),(140,185,176),(166,158,197),(122,177,191),(158,175,196)]
labels={'bedroom':'卧室','bathroom':'浴室','living_room':'客厅','kitchen':'厨房','entry_hall':'门厅','corridor':'走廊'}
rot=r.rotation(np.pi,0);v=p@rot.T;centre=(v.min(0)+v.max(0))/2;extent=v.max(0)-v.min(0);scale=min(1400/extent[0],850/extent[1]);origin=np.array([920,570])
def project(q,R=rot,ce=centre,sc=scale,ori=origin):
 vv=np.array(q)@R.T;return np.column_stack([ori[0]+(vv[:,0]-ce[0])*sc,ori[1]-(vv[:,1]-ce[1])*sc]).round().astype('int32'),vv[:,2]
def raster(q,R,ce,sc):
 im=np.tile(BG,(H*W,1)).astype('uint8');uv,z=project(q,R,ce,sc);valid=(uv[:,0]>=0)&(uv[:,0]<W)&(uv[:,1]>=130)&(uv[:,1]<H-80);uv,z=uv[valid],z[valid];addr=uv[:,1]*W+uv[:,0];order=np.lexsort((z,addr));ids=order[np.r_[addr[order][:-1]!=addr[order][1:],True]];im[addr[ids]]=[184,192,201];return im.reshape(H,W,3)
top=raster(p,rot,centre,scale)
room=next(x for x in floor['rooms']if x['id']==8);polygon=np.array(room['polygon_xy_m']);lo=polygon.min(0)-1.5;hi=polygon.max(0)+1.5;local=p[(p[:,0]>=lo[0])&(p[:,0]<=hi[0])&(p[:,1]>=lo[1])&(p[:,1]<=hi[1])];R=r.rotation(2.4,-.78);vv=local@R.T;ce=(vv.min(0)+vv.max(0))/2;ex=vv.max(0)-vv.min(0);sc=min(1300/ex[0],720/ex[1]);oblique=raster(local,R,ce,sc)
objects=[o for o in room['objects']if o['label']in ['cabinet','countertop','microwave','table']][:5];objectnames={'cabinet':'柜体','countertop':'台面','microwave':'微波炉','table':'桌子'}
def corners(o):
 c=np.array(o['center_xyz_m']);h=np.array(o['size_xyz_m'])/2;w,x,y,z=o['orientation_wxyz'];M=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]);off=np.array([[a,b,c]for a in [-1,1]for b in [-1,1]for c in [-1,1]])*h;return off@M.T+c
edges=[(i,j)for i in range(8)for j in range(i+1,8)if bin(i^j).count('1')==1]
# Quantify registration scale and map coverage rather than silently treating old results as same-run outputs.
tree=cv2.flann_Index(p.astype('float32'),dict(algorithm=1,trees=4));check=[]
for o in objects:
 uv,d=tree.knnSearch(np.array(o['center_xyz_m'],dtype='float32')[None,:],1,params=dict(checks=64));check.append({'id':o['id'],'label':o['label'],'center_to_nearest_observed_point_m':float(np.sqrt(d[0,0]))})
(OUT/'对齐检查.json').write_text(json.dumps({'frame':floor['coordinate_frame'],'floor2_point_z_range':[float(p[:,2].min()),float(p[:,2].max())],'selected_objects':check,'note':'中心到点云距离仅为粗略诊断，物体中心通常不在表面；未验证全部旧房间轮廓与本轮覆盖一致。'},ensure_ascii=False,indent=2))
path=OUT/'房间分割_语义box_18秒预览.mp4';enc=subprocess.Popen(['gst-launch-1.0','-q','fdsrc','!','videoparse','format=rgb','width=1920','height=1080','framerate=30/1','!','videoconvert','!','x264enc','speed-preset=veryfast','bitrate=9000','!','h264parse','!','mp4mux','!','filesink',f'location={path}'],stdin=subprocess.PIPE)
try:
 for tick in range(540):
  t=tick/30
  if t<8:
   im=top.copy();progress=np.clip((t-2)/4,0,1);over=im.copy()
   for k,rm in enumerate(floor['rooms']):
    poly=np.array(rm['polygon_xy_m']);q=np.c_[poly,np.full(len(poly),3.)];uv,_=project(q);c=roomcolors[k%len(roomcolors)];cv2.fillPoly(over,[uv],c,cv2.LINE_AA)
   im=cv2.addWeighted(over,.18*progress,im,1-.18*progress,0)
   for k,rm in enumerate(floor['rooms']):
    poly=np.array(rm['polygon_xy_m']);uv,_=project(np.c_[poly,np.full(len(poly),3.)]);cv2.polylines(im,[uv],True,tuple(int(c*progress+BG[j]*(1-progress))for j,c in enumerate(roomcolors[k%len(roomcolors)])),1,cv2.LINE_AA)
   out=Image.fromarray(im);d=ImageDraw.Draw(out);title='点云地图'if t<2 else'房间分割 · 二层';sub='从观测几何到房间轮廓'
   if t>=5:
    for rm in floor['rooms']:
     uv,_=project(np.array([[*rm['centroid_xy_m'],3.]]));name=labels.get(rm.get('qwen_semantic_type'),labels.get(rm.get('semantic_type'),'区域'));d.text(tuple(uv[0]-[30,10]),name,font=r.font(20),fill=(220,231,240))
  else:
   im=oblique.copy();n=int(np.clip((t-9)*1.1,0,len(objects)));text=[]
   for k,o in enumerate(objects[:n]):
    uv,_=project(corners(o),R,ce,sc);c=roomcolors[k%len(roomcolors)]
    for a,b in edges:cv2.line(im,tuple(uv[a]),tuple(uv[b]),c,2,cv2.LINE_AA)
    anchor=np.array([uv[:,0].max(),uv[:,1].min()]);text.append((anchor,k,o,c))
   out=Image.fromarray(im);d=ImageDraw.Draw(out);title='语义物体 · 厨房示例';sub='物体类别、三维范围与所属房间'
   for anchor,k,o,c in text:
    x,y=1580,190+k*105;d.line([tuple(anchor),(x-20,y+15)],fill=c,width=1);d.rounded_rectangle((x-8,y-5,x+200,y+63),radius=5,fill=BG);d.text((x,y),objectnames[o['label']],font=r.font(26),fill=c);d.text((x,y+34),f"置信度 {o['probability']:.2f}",font=r.font(18),fill=(169,183,199))
  d.text((30,22),title,font=r.font(36),fill=(232,239,246));d.text((30,80),sub,font=r.font(23),fill=(170,190,209));d.text((30,1030),'已有房间与语义结果的展示预览 · 底图采用最新探索观测 · 非本轮在线生成记录',font=r.font(21),fill=(150,167,184));enc.stdin.write(np.asarray(out).tobytes())
  if tick in [210,450,539]:out.save(OUT/f'预览_{tick}.jpg')
finally:
 enc.stdin.close()
 if enc.wait():raise RuntimeError('encoder failed')
(OUT/'生成配置.json').write_text(json.dumps({'fps':30,'duration':18,'floor':2,'room_id':8,'semantic_source':'PRE_MAP_VLN/outputs/room_pipeline/00337_transition_aware_qwen_v1/multifloor_scene_graph_qwen.json','cloud_source':'视频制作/探索剪辑素材_无字30帧/geometry.npz','object_ids':[o['id']for o in objects],'display_only':True},ensure_ascii=False,indent=2));print(path)
