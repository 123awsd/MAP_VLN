from pathlib import Path
import sys,subprocess,json,time
import numpy as np,cv2
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent;sys.path.insert(0,str(ROOT/'PRE_MAP_VLN/scripts'));import render_exploration_three_view as r
CACHE=ROOT/'PRE_MAP_VLN/outputs/exploration_recovery_local_trial/round6_extended/preview_trimmed';data=np.load(CACHE/'increments.npz');old=np.load(CACHE/'timeline.npz');geo=np.load(Path(__file__).resolve().parent/'geometry.npz');poses=np.load(CACHE/'camera_pose.npy');targets=data['targets'];xyz=old['trajectory'];times=old['trajectory_times'];ground=np.array([-.155,2.595,5.595]);bases=np.array([-.2,2.6,5.4]);upper=np.array([2.05,4.85,7.05]);rot=r.rotation(2.4,-1.12);BG=np.array([18,21,26]);W,H=1280,720;FPS=20;N=800
palettes=[['#2171b5','#6baed6','#c6dbef'],['#238b45','#66c2a4','#ccece6'],['#6a51a3','#9e9ac8','#dadaeb']]
def rgb(h):return np.array([int(h[i:i+2],16)for i in (1,3,5)],float)
def color(f,t):
 p=np.array([rgb(h)for h in palettes[f]]);t=np.clip(t,0,1)*2;i=np.minimum(t.astype(int),1);u=t-i;return p[i]*(1-u[:,None])+p[i+1]*u[:,None]
refs=old['cloud_100'];ref=refs[(refs[:,2]>=-.25)&(refs[:,2]<=7.05)]@rot.T;centre=(ref.min(0)+ref.max(0))/2;extent=ref.max(0)-ref.min(0);scale=min((W-165)/extent[0],(H-170)/extent[1]);origin=np.array([W*.48,H*.53])
def project(q):
 v=q@rot.T;return np.column_stack([origin[0]+(v[:,0]-centre[0])*scale,origin[1]-(v[:,1]-centre[1])*scale]).astype(int),v[:,2]
trees=[cv2.flann_Index(geo[f'p{f}'].astype(np.float32),dict(algorithm=1,trees=4))for f in range(3)];depths=np.full((3,H*W),-np.inf,np.float32);cols=np.zeros((3,H*W,3),np.uint8);alphas=np.zeros((3,H*W),np.float32);seen=[set()for _ in range(3)];light=np.array([.35,-.45,.82]);light/=np.linalg.norm(light)
def update(p):
 for f in range(3):
  floor=np.abs(p[:,2]-ground[f])<.06;keep=((p[:,2]>bases[f]+.2)&(p[:,2]<upper[f]))|floor;q=p[keep];fl=floor[keep]
  if not len(q):continue
  cells=np.floor(q/.025).astype(np.int32);_,ids=np.unique(cells,axis=0,return_index=True);q,fl,cells=q[ids],fl[ids],cells[ids];keys=[tuple(c)for c in cells];fresh=np.fromiter((k not in seen[f]for k in keys),bool,len(keys));seen[f].update(keys);q,fl=q[fresh],fl[fresh]
  if not len(q):continue
  ni,_=trees[f].knnSearch(q.astype(np.float32),1,params=dict(checks=16));ni=ni[:,0];ns=geo[f'n{f}'][ni];illum=.64+.36*np.abs(ns@light);c=color(f,.25+.70*np.clip((q[:,2]-ground[f])/2.05,0,1))*illum[:,None];c[fl]*=.42;front=(np.abs(ns@rot[2])>.55)&(np.abs(ns[:,2])<.45);alpha=np.where(front,.24,.78);alpha[fl]=.08
  uv,z=project(q);ok=(uv[:,0]>=0)&(uv[:,0]<W)&(uv[:,1]>=70)&(uv[:,1]<H-55);uv,z,c,alpha=uv[ok],z[ok],c[ok],alpha[ok]
  if not len(z):continue
  addr=uv[:,1]*W+uv[:,0];order=np.lexsort((z,addr));ids=order[np.r_[addr[order][:-1]!=addr[order][1:],True]];addr=addr[ids];fresh=z[ids]>depths[f,addr];addr,ids=addr[fresh],ids[fresh];depths[f,addr]=z[ids];cols[f,addr]=c[ids].astype('uint8');alphas[f,addr]=alpha[ids]
def frame(idx):
 stamp=targets[idx];a=np.arange(H*W);order=np.argsort(depths,axis=0);pixels=np.tile(BG,(H*W,1)).astype(np.float32)
 for k in range(3):
  fi=order[k];alpha=alphas[fi,a,None];pixels=pixels*(1-alpha)+cols[fi,a]*alpha
 im=pixels.reshape(H,W,3).astype('uint8');valid=times<=stamp;uv,_=project(xyz[valid])
 if len(uv)>1:cv2.polylines(im,[uv],False,(107,145,174),1,cv2.LINE_AA)
 recent=times[valid]>stamp-10
 if recent.sum()>1:cv2.polylines(im,[uv[recent]],False,(215,238,246),2,cv2.LINE_AA)
 pose=poses[max(0,np.searchsorted(poses[:,0],stamp)-1)];qx,qy,qz,qw=pose[4:];pr=np.array([[1-2*(qy*qy+qz*qz),2*(qx*qy-qz*qw),2*(qx*qz+qy*qw)],[2*(qx*qy+qz*qw),1-2*(qx*qx+qz*qz),2*(qy*qz-qx*qw)],[2*(qx*qz-qy*qw),2*(qy*qz+qx*qw),1-2*(qx*qx+qy*qy)]]);corners=np.array([[-.55,-.4,.85],[.55,-.4,.85],[.55,.4,.85],[-.55,.4,.85]])@pr.T+pose[1:4];cu,_=project(np.vstack([pose[1:4],corners]))
 for s,t in [(0,1),(0,2),(0,3),(0,4),(1,2),(2,3),(3,4),(4,1)]:cv2.line(im,tuple(cu[s]),tuple(cu[t]),(192,242,247),1,cv2.LINE_AA)
 cv2.circle(im,tuple(cu[0]),4,(235,250,255),-1,cv2.LINE_AA)
 out=Image.fromarray(im).resize((1920,1080),Image.Resampling.LANCZOS);d=ImageDraw.Draw(out);d.text((28,16),'三层自主探索 · 蓝／青绿／紫渐变',font=r.font(31),fill=(228,237,244));active=int(np.clip(np.searchsorted(bases+.15,pose[3])-1,0,2));d.text((28,64),f'当前：{active+1}层   |   已观测点云累积',font=r.font(23),fill=(174,193,211));d.text((28,1037),f'仿真观测加速回放 · {stamp-targets[0]:.1f}s / {targets[-1]-targets[0]:.1f}s',font=r.font(20),fill=(175,189,203))
 for f in range(3):
  x=30+f*470;y=988;sw=color(f,np.linspace(0,1,260));out.paste(Image.fromarray(np.tile(sw.astype('uint8')[None,:,:],(11,1,1))),(x,y));d.text((x,y+15),f'{f+1}层 · 层内高度低 → 高',font=r.font(18),fill=(190,203,213))
 return out

OUT=ROOT/'视频制作/三层探索_俯视主图三视角';TW,TH=1290,850;toprot=r.rotation(np.pi,0.);top=[]
for f in range(3):
 q=refs[(refs[:,2]>bases[f]+.2)&(refs[:,2]<upper[f])];v=q@toprot.T;ce=(v.min(0)+v.max(0))/2;ex=v.max(0)-v.min(0);sc=min((TW-100)/ex[0],(TH-90)/ex[1]);top.append({'centre':ce,'scale':sc,'depth':np.full(TW*TH,-np.inf,np.float32),'pixels':np.tile(BG,(TW*TH,1)).astype('uint8')})
def topproject(q,f):
 v=q@toprot.T;t=top[f];return np.column_stack([TW/2+(v[:,0]-t['centre'][0])*t['scale'],TH/2-(v[:,1]-t['centre'][1])*t['scale']]).astype(int),v[:,2]
updateold=update
def update(p):
 updateold(p)
 for f in range(3):
  q=p[(p[:,2]>bases[f]+.2)&(p[:,2]<upper[f])];
  if not len(q):continue
  uv,z=topproject(q,f);ok=(uv[:,0]>=0)&(uv[:,0]<TW)&(uv[:,1]>=0)&(uv[:,1]<TH);q,uv,z=q[ok],uv[ok],z[ok];
  if not len(q):continue
  addr=uv[:,1]*TW+uv[:,0];order=np.lexsort((z,addr));ids=order[np.r_[addr[order][:-1]!=addr[order][1:],True]];addr=addr[ids];fresh=z[ids]>top[f]['depth'][addr];addr,ids=addr[fresh],ids[fresh];top[f]['depth'][addr]=z[ids];top[f]['pixels'][addr]=color(f,.40+.55*np.clip((q[ids,2]-ground[f])/2.05,0,1)).astype('uint8')
floors=np.clip(np.searchsorted(bases+.15,xyz[:,2])-1,0,2)
def lines(im,q,proj,c,w):
 if len(q)>1:cv2.polylines(im,[proj(q)[0]],False,c,w,cv2.LINE_AA)
def getcamera(pose):
 qx,qy,qz,qw=pose[4:];pr=np.array([[1-2*(qy*qy+qz*qz),2*(qx*qy-qz*qw),2*(qx*qz+qy*qw)],[2*(qx*qy+qz*qw),1-2*(qx*qx+qz*qz),2*(qy*qz-qx*qw)],[2*(qx*qz-qy*qw),2*(qy*qz+qx*qw),1-2*(qx*qx+qy*qy)]]);corners=np.array([[-.55,-.4,.85],[.55,-.4,.85],[.55,.4,.85],[-.55,.4,.85]])@pr.T+pose[1:4];return np.vstack([pose[1:4],corners])
def cone(im,cam,proj):
 uv,_=proj(cam)
 for s,t in [(0,1),(0,2),(0,3),(0,4),(1,2),(2,3),(3,4),(4,1)]:cv2.line(im,tuple(uv[s]),tuple(uv[t]),(192,242,247),1,cv2.LINE_AA)
 cv2.circle(im,tuple(uv[0]),4,(235,250,255),-1,cv2.LINE_AA)
def fit(out,im,box):
 x,y,w,h=box;im=Image.fromarray(im)if isinstance(im,np.ndarray)else im;im=im.copy();im.thumbnail((w,h),Image.Resampling.LANCZOS);out.paste(im,(x+(w-im.width)//2,y+(h-im.height)//2))
side_bounds=np.zeros((H,W),bool)
uv,_=project(refs[(refs[:,2]>=-.25)&(refs[:,2]<=7.05)])
ok=(uv[:,0]>=0)&(uv[:,0]<W)&(uv[:,1]>=70)&(uv[:,1]<H-55);uv=uv[ok];side_bounds[uv[:,1],uv[:,0]]=True

OUT=ROOT/'视频制作/探索剪辑素材_无字30帧'
CONFIG=json.loads((OUT/'生成配置.json').read_text());FPS=int(CONFIG['fps']);COUNT=int(round(FPS*CONFIG['duration_seconds']));palettes=CONFIG['palettes']
if CONFIG.get('yaw_matched_camera',False):poses=np.load(OUT/'yaw_matched_camera_pose.npy')
ys,xs=np.where(side_bounds);x0,x1=max(0,int(xs.min())-20),min(W,int(xs.max())+21);y0,y1=max(0,int(ys.min())-20),min(H,int(ys.max())+21)
SW=x1-x0;SH=y1-y0
# H.264 yuv420 requires even output dimensions; pad rather than distort the view.
SW=((SW+3)//4)*4;SH+=SH%2
specs=[('01_俯视图_无字_30fps.mp4',((TW+3)//4)*4,TH,9000),('02_全局点云_无字_30fps.mp4',SW,SH,6500),('03_第一视角_无字_30fps.mp4',640,480,6500),('04_合并版_无字_30fps.mp4',1920,1080,12000)]
def encode(fn,w,h,bitrate):return subprocess.Popen(['gst-launch-1.0','-q','fdsrc','!','videoparse','format=rgb',f'width={w}',f'height={h}',f'framerate={FPS}/1','!','videoconvert','!','x264enc','speed-preset=veryfast',f'bitrate={bitrate}','!','h264parse','!','mp4mux','!','filesink',f'location={OUT/fn}'],stdin=subprocess.PIPE)
encoders=[encode(*s)for s in specs]
def topfan(im,cam,proj):
 uv,_=proj(cam);eye=uv[0];forward=uv[1:].mean(0)-eye;length=float(np.linalg.norm(forward))
 if length<1e-6:return
 direction=forward/length;side=np.array([-direction[1],direction[0]]);half=float(np.max(np.abs((uv[1:]-eye)@side)));angle=np.arctan2(direction[1],direction[0]);halfangle=np.arctan2(half,length);rad=np.hypot(half,length);angles=np.linspace(angle-halfangle,angle+halfangle,50);arc=np.column_stack([eye[0]+rad*np.cos(angles),eye[1]+rad*np.sin(angles)]).round().astype('int32');pts=np.vstack([eye,arc]);overlay=im.copy();C=(173,221,236);cv2.fillConvexPoly(overlay,pts,C,cv2.LINE_AA);cv2.addWeighted(overlay,.10,im,.90,0,dst=im);cv2.polylines(im,[arc],False,C,1,cv2.LINE_AA)
 for q in [arc[0],arc[-1]]:cv2.line(im,tuple(eye),tuple(q),C,1,cv2.LINE_AA)
 cv2.circle(im,tuple(eye),4,tuple(map(int,BG)),-1,cv2.LINE_AA);cv2.circle(im,tuple(eye),3,(236,250,255),-1,cv2.LINE_AA)
def render(idx,stamp):
 pose=poses[max(0,np.searchsorted(poses[:,0],stamp)-1)];active=int(np.clip(np.searchsorted(bases+.15,pose[3])-1,0,2));cam=getcamera(pose);a=np.arange(H*W);order=np.argsort(depths,axis=0);pix=np.tile(BG,(H*W,1)).astype(np.float32)
 for k in range(3):
  fi=order[k];alpha=alphas[fi,a,None];pix=pix*(1-alpha)+cols[fi,a]*alpha
 side=pix.reshape(H,W,3).astype('uint8');recent=(times<=stamp)&(times>stamp-CONFIG['oblique_trail_source_seconds']);lines(side,xyz[recent],project,(215,238,246),2);cone(side,cam,project);side=Image.fromarray(side[y0:y1,x0:x1]);padded=Image.new('RGB',(SW,SH),tuple(BG));padded.paste(side,(0,0));side=padded
 planar=top[active]['pixels'].reshape(TH,TW,3).copy();mask=(times<=stamp)&(floors==active);edge=np.diff(np.r_[False,mask,False].astype(int))
 for start,end in zip(np.where(edge==1)[0],np.where(edge==-1)[0]):lines(planar,xyz[start:end],lambda q:topproject(q,active),(157,185,205),1)
 lines(planar,xyz[recent&(floors==active)],lambda q:topproject(q,active),(236,247,250),2);topfan(planar,cam,lambda q:topproject(q,active));planar=Image.fromarray(planar)
 rgbidx=int(np.argmin(np.abs(targets-stamp)));camera=Image.open((OUT/'yaw_matched_rgb'/f'rgb_{int(np.argmin(np.abs(poses[:,0]-stamp))):04d}.jpg') if CONFIG.get('yaw_matched_camera',False) else (CACHE/f'rgb_{rgbidx:03d}.jpg')).convert('RGB')
 out=Image.new('RGB',(1920,1080),tuple(BG));out.paste(planar,(20,115));fit(out,side,(1330,120,570,410));fit(out,camera,(1330,550,570,410));return [planar,side,camera,out]
previous=-1
try:
 for tick,stamp in enumerate(np.linspace(targets[0],targets[-1],COUNT)):
  idx=max(0,int(np.searchsorted(targets,stamp,side='right')-1))
  for j in range(previous+1,idx+1):update(data[f'new_{j:03d}'])
  previous=idx;frames=render(idx,stamp)
  for e,im,spec in zip(encoders,frames,specs):
   if im.size!=(spec[1],spec[2]):
    padded=Image.new('RGB',(spec[1],spec[2]),tuple(BG));padded.paste(im,(0,0));im=padded
   e.stdin.write(np.asarray(im).tobytes())
  if tick in [300,600,975,1199]:frames[-1].save(OUT/f'合并预览_{tick}.jpg')
  if tick%120==0:print('frame',tick,'/',COUNT,flush=True)
finally:
 for e in encoders:e.stdin.close()
 for e in encoders:
  if e.wait():raise RuntimeError('encoder failed')
(OUT/'素材说明.json').write_text(json.dumps({'source':str(CACHE),'fps':30,'seconds':40,'frames':1200,'files':[{'name':fn,'width':w,'height':h}for fn,w,h,b in specs],'oblique_trail_source_seconds':15,'top_history':True,'top_camera_marker':'样式04：淡填充二维扇形视野示意','text_and_legends':False,'merged_layout':{'top':[20,115,1290,850],'global':[1330,120,570,410],'camera':[1330,550,570,410],'right_total_height_including_gap':840},'sync':'四个视频相同时间轴，同步起点、终点、时长；原相机观测为800帧，30fps输出按最近源帧重采样，不生成虚构RGB帧；轨迹按输出时刻更新。'},ensure_ascii=False,indent=2))
print('四个无字30fps视频完成',flush=True)
