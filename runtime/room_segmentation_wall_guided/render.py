"""ROSE2 polygons over actual observed cloud, faint fill only in observed free cells."""
from pathlib import Path
import json,sys
import numpy as np
from PIL import Image,ImageDraw,ImageFont
root=Path('outputs/room_wall_guided_trial')/sys.argv[1];variant='' ;W,H=1200,1450;panels=[];palette=np.array([[130,190,225],[244,174,139],[179,213,153],[200,171,219],[238,216,125],[144,211,193],[225,159,186],[179,190,230],[212,187,155],[170,216,215],[229,197,208],[180,222,170],[232,161,123],[143,171,216]])
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',23);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',17)
for f in [1,2,3]:
 p=root/f'L{f}{variant}';d=np.load(p/'segmentation.npz');grid=d['grid'];labels=d['labels'];origin=d['origin'];res=float(d['resolution']);points=np.load(Path('outputs/room_apf_latest_trial')/f'L{f}_input.npy');ground=[-.155,2.595,5.595][f-1];q=points[(points[:,2]>ground+.15)&(points[:,2]<ground+2.2)]
 lo=q[:,:2].min(0)-.4;hi=q[:,:2].max(0)+.4;scale=min(1100/(hi[0]-lo[0]),1250/(hi[1]-lo[1]));left=(W-(hi[0]-lo[0])*scale)/2;top=110+(1250-(hi[1]-lo[1])*scale)/2
 def project(q):return np.c_[left+(hi[0]-q[:,0])*scale,top+(q[:,1]-lo[1])*scale]
 im=Image.new('RGB',(W,H),(250,251,253));draw=ImageDraw.Draw(im)
 for lab in np.unique(labels[labels>=0]):
  mask=labels==lab;yx=np.argwhere(mask);xy=origin+yx[:,::-1]*res;rgb=tuple(np.rint(.60*palette[lab%len(palette)]+.40*np.array([250,251,253])).astype(int));uv=project(xy)
  for x,y in uv:
   sz=res*scale;draw.rectangle((x-sz,y,x,y+sz),fill=rgb)
 arr=np.array(im);uv=np.rint(project(q)).astype(int);good=(uv[:,0]>=0)&(uv[:,0]<W)&(uv[:,1]>=100)&(uv[:,1]<H-60);uv=uv[good];arr[uv[:,1],uv[:,0]]=(64,75,84);im=Image.fromarray(arr);draw=ImageDraw.Draw(im)
 for lab in np.unique(labels[labels>=0]):
  yx=np.argwhere(labels==lab);xy=origin+yx[:,::-1]*res;centre=xy.mean(0);closest=xy[np.argmin(np.linalg.norm(xy-centre,axis=1))];x,y=project(closest[None])[0];draw.rounded_rectangle((x-15,y-15,x+15,y+15),radius=4,fill='white',outline=(170,180,190));draw.text((x-7,y-14),str(lab),font=font,fill=(25,35,45))
 draw.text((30,25),f'Floor {f} | wall-guided watershed '+root.name,font=font,fill=(25,35,45));draw.text((30,60),'60% tint | measured cloud | unknown not colored',font=small,fill=(85,95,105));draw.text((30,H-42),'Input occupancy slab: floor+0.8 to1.2m; cloud shown at0.15 to2.2m',font=small,fill=(85,95,105));im.save(p/'cloud_overlay_light.png');panels.append(im)
wide=Image.new('RGB',(W*3,H),'white')
for i,im in enumerate(panels):wide.paste(im,(i*W,0))
wide.save(root/('three_floor_cloud_overlay_voronoi.png' if variant else 'three_floor_cloud_overlay_wide.png'));print('saved wide overlays')
