"""Ceiling-supported envelope + measured interior plane candidates. Display/inference only."""
from pathlib import Path
import numpy as np,cv2,json
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];base=ROOT/'outputs/room_quality_ceiling';out=ROOT.parent/'视频制作/探索地图完整性检查';geo=np.load(base/'geometry.npz');font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',32);small=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',22);board=Image.new('RGB',(2700,1500),(18,22,28));metrics=[]
for f,z in enumerate([-.155,2.595,5.595]):
 d=np.load(base/f'D_墙面方向统一_L{f+1}.npz');p,n=geo[f'p{f}'],geo[f'n{f}'];origin=d['origin'];res=float(d['resolution']);h,w=d['wall'].shape;uv=np.floor((p[:,:2]-origin)/res).astype(int);hi=np.full((h,w),-np.inf);np.maximum.at(hi,(uv[:,1],uv[:,0]),p[:,2]);sel=(p[:,2]>=hi[uv[:,1],uv[:,0]]-.12)&(abs(n[:,2])>.55)&(p[:,2]>z+1.55);roof=np.zeros((h,w),np.uint8);q=uv[sel];roof[q[:,1],q[:,0]]=1;roof=cv2.morphologyEx(roof,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7)));cs,_=cv2.findContours(roof,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);shell=np.zeros_like(roof);envelope=np.zeros_like(roof)
 for c in cs:
  if cv2.contourArea(c)*res**2<1.:continue
  c=cv2.approxPolyDP(c,.12/res,True);cv2.polylines(shell,[c],True,1,1);cv2.fillPoly(envelope,[c],1)
 interior=d['wall']&(envelope>0);combined=(shell>0)|interior
 np.savez_compressed(base/f'E_天花板边界辅助_L{f+1}.npz',wall_candidates=interior,envelope_boundary=shell>0,envelope=envelope>0,roof_observed=roof>0,origin=origin,resolution=res)
 im=np.full((h,w,3),(18,22,28),np.uint8);im[combined]=(235,242,250);yy,xx=np.where(combined);crop=np.flipud(im[max(0,yy.min()-6):yy.max()+7,max(0,xx.min()-6):xx.max()+7]).copy();pic=Image.fromarray(crop);s=min(830/pic.width,1250/pic.height);pic=pic.resize((round(pic.width*s),round(pic.height*s)),Image.Resampling.NEAREST);panel=Image.new('RGB',(900,1500),(18,22,28));panel.paste(pic,((900-pic.width)//2,130));dr=ImageDraw.Draw(panel);dr.text((25,20),f'{f+1}层 · 天花板外边界＋内部墙面',font=font,fill='white');dr.text((25,78),'结构推断试验 · 外轮廓不等于全部真实墙',font=small,fill=(189,201,216));panel.save(out/f'{f+1}层_天花板辅助_仅结构.png');board.paste(panel,(900*f,0));metrics.append({'floor':f+1,'ceiling_points':int(sel.sum()),'ceiling_observed_xy_cells':int(roof.sum()),'envelope_m2':float(envelope.sum()*res**2)})
board.save(out/'三层_天花板辅助_仅结构.png');(base/'ceiling_enclosure_metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2));print(metrics)
