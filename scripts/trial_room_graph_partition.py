"""Experimental global spectral partition of multi-height structural evidence.
No manual room seeds, line extension or known room counts.
"""
from pathlib import Path
import numpy as np,cv2,json
from scipy.ndimage import gaussian_filter,binary_fill_holes
from scipy.sparse import coo_matrix,diags
from scipy.sparse.linalg import eigsh
from sklearn.cluster import KMeans
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'outputs/room_quality_ceiling';OUT=ROOT/'outputs/room_graph_partition';OUT.mkdir(exist_ok=True);VIS=ROOT.parent/'视频制作/整体房间图划分';VIS.mkdir(exist_ok=True);geo=np.load(BASE/'geometry.npz');font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',26);small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',19);BG=(18,22,28);records=[]
for f,ground in enumerate([-.155,2.595,5.595]):
 p,n=geo[f'p{f}'],geo[f'n{f}'];res=.15;origin=np.floor((p[:,:2].min(0)-.3)/res)*res;wh=np.ceil((p[:,:2].max(0)+.3-origin)/res).astype(int)+1;shape=tuple(wh[::-1]);xy=np.floor((p[:,:2]-origin)/res).astype(int);counts=np.zeros(shape,np.int32);np.add.at(counts,(xy[:,1],xy[:,0]),1);foot=cv2.morphologyEx((counts>0).astype('uint8'),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8));domain=binary_fill_holes(foot)>0
 rel=p[:,2]-ground;bins=np.zeros((14,*shape),bool)
 for z in range(14):
  ok=(abs(n[:,2])<.35)&(rel>=z*.2)&(rel<(z+1)*.2)&(rel>.3);uv=xy[ok];bins[z,uv[:,1],uv[:,0]]=1
 # Continuous boundary likelihood instead of discarding weak columns.
 height=bins.sum(0);upper=bins[7:].sum(0);evidence=np.clip(height/7,0,1)*np.clip(upper/2,0,1)
 methods=[('多高度全局划分',evidence),('上部结构全局划分',np.clip(upper/4,0,1))]
 coords=np.argwhere(domain);ids=np.full(shape,-1,int);ids[domain]=np.arange(len(coords));ridx=[];cidx=[];strength=[]
 board=Image.new('RGB',(2700,1500),BG)
 for mi,(name,ev)in enumerate(methods):
  rows=[];cols=[];values=[]
  for dy,dx in [(0,1),(1,0)]:
   y,x=coords[:,0],coords[:,1];yy,xx=y+dy,x+dx;ok=(yy<shape[0])&(xx<shape[1]);a=ids[y[ok],x[ok]];b=ids[yy[ok],xx[ok]];valid=b>=0;cut=np.maximum(ev[y[ok],x[ok]],ev[yy[ok],xx[ok]])[valid];v=np.exp(-6*cut)+.005;a,b=a[valid],b[valid];rows.extend([a,b]);cols.extend([b,a]);values.extend([v,v])
  adj=coo_matrix((np.concatenate(values),(np.concatenate(rows),np.concatenate(cols))),shape=(len(coords),len(coords))).tocsr();degree=np.asarray(adj.sum(1)).ravel();inv=diags(1/np.sqrt(np.maximum(degree,1e-12)));lap=diags(np.ones(len(coords)))-inv@adj@inv
  eig,vec=eigsh(lap,k=23,sigma=-1e-5,which='LM',tol=1e-5);order=np.argsort(eig);eig,vec=eig[order],vec[:,order];gaps=np.diff(eig);k=int(np.argmax(gaps[1:19])+2)
  emb=vec[:,:k];emb/=np.maximum(np.linalg.norm(emb,axis=1,keepdims=True),1e-8);lab=KMeans(n_clusters=k,n_init=10,random_state=42).fit_predict(emb)+1;labels=np.zeros(shape,int);labels[domain]=lab
  boundary=np.zeros(shape,np.uint8);boundary[:,1:]|=((labels[:,1:]!=labels[:,:-1])&(labels[:,1:]>0)).astype('uint8');boundary[1:]|=((labels[1:]!=labels[:-1])&(labels[1:]>0)).astype('uint8');boundary|=(domain&~cv2.erode(domain.astype('uint8'),np.ones((3,3),np.uint8)).astype(bool)).astype('uint8')
  np.savez_compressed(OUT/f'L{f+1}_{mi}.npz',labels=labels,boundary=boundary,wall_evidence=ev,origin=origin,resolution=res,eigenvalues=eig)
  # Report evidence agreement rather than treating the selected count as truth.
  record=dict(floor=f+1,method=name,regions=k,eigenvalues=eig.tolist(),areas_m2=[float((labels==j).sum()*res**2)for j in range(1,k+1)]);records.append(record);print(f+1,name,k,flush=True)
  for panelidx in ([0,mi+1]if mi==0 else [mi+1]):
   im=np.full((*shape,3),BG,np.uint8);shade=(35+75*np.clip(np.log1p(counts)/np.log(100),0,1)).astype('uint8');im[counts>0]=shade[counts>0,None]
   if panelidx:im[boundary>0]=250
   pic=Image.fromarray(np.flipud(im).copy());sc=min(830/pic.width,1210/pic.height);pic=pic.resize((round(pic.width*sc),round(pic.height*sc)),Image.Resampling.NEAREST);panel=Image.new('RGB',(900,1500),BG);left=(900-pic.width)//2;panel.paste(pic,(left,170));dr=ImageDraw.Draw(panel);dr.text((24,20),f'{f+1}层 · '+(name if panelidx else '原始点云'),font=font,fill='white');dr.text((24,75),'白：区域边界假设；灰：实测点云',font=small,fill=(190,202,216))
   if panelidx:
    for j in range(1,k+1):
     yy,xx=np.where(labels==j);cent=np.array([yy.mean(),xx.mean()]);near=np.argmin((yy-cent[0])**2+(xx-cent[1])**2);px=left+xx[near]*sc;py=170+(shape[0]-1-yy[near])*sc;dr.text((px,py),str(j),font=small,fill='white',stroke_width=2,stroke_fill=BG)
   panel.save(VIS/f'{f+1}层_{panelidx}.png');board.paste(panel,(panelidx*900,0))
 board.save(VIS/f'{f+1}层_对比.png')
(OUT/'metrics.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
