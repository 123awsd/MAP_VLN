"""Wide APF review with faint regions and foreground measured point clouds."""
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
root=Path('outputs/room_apf_latest_trial');W,H=1200,1450;panels=[]
palette=np.array([[130,190,225],[244,174,139],[179,213,153],[200,171,219],[238,216,125],[144,211,193],[225,159,186],[179,190,230],[212,187,155],[170,216,215],[229,197,208],[180,222,170],[232,161,123],[143,171,216],[180,171,133],[210,171,171],[133,200,215],[190,200,140]])
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',23);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',17)
for f in [1,2,3]:
 p=root/f'L{f}_corrected_fullvis';d=np.load(p/'segmentation.npz');points=d['points'];cells=d['cells'];labels=d['cell_labels'];origin=d['grid_origin'];v=float(d['voxel']);pf=d['potential_field'];nx,ny=pf.shape
 # Match original top view orientation (rotation pi about Z).
 scale=min(1100/(nx*v),1250/(ny*v));left=(W-nx*v*scale)/2;top=115+(1250-ny*v*scale)/2
 def project(q):return np.c_[left+(nx*v-(q[:,0]-origin[0]))*scale,top+(q[:,1]-origin[1])*scale]
 image=Image.new('RGB',(W,H),(250,251,253));draw=ImageDraw.Draw(image)
 for c,lab in zip(cells,labels):
  if lab<0:continue
  corner=project(np.array([[origin[0]+c[0]*v,origin[1]+c[1]*v],[origin[0]+(c[0]+1)*v,origin[1]+(c[1]+1)*v]]));rgb=np.rint(.18*palette[lab%len(palette)]+.82*np.array([250,251,253])).astype(int)
  x0,x1=sorted(corner[:,0]);y0,y1=sorted(corner[:,1]);draw.rectangle((x0,y0,x1,y1),fill=tuple(rgb))
 # Regions do not paint over cloud: cloud always drawn last.
 ground=[-.155,2.595,5.595][f-1];q=points[(points[:,2]>ground+.15)&(points[:,2]<ground+2.2)];uv=np.rint(project(q)).astype(int);ok=(uv[:,0]>=0)&(uv[:,0]<W)&(uv[:,1]>=100)&(uv[:,1]<H-65);uv=uv[ok]
 arr=np.array(image);arr[uv[:,1],uv[:,0]]=(64,75,84);image=Image.fromarray(arr);draw=ImageDraw.Draw(image)
 for lab in np.unique(labels[labels>=0]):
  selected=cells[labels==lab];area=len(selected)*v*v
  if area<.5:continue
  centre=selected.mean(0);closest=selected[np.argmin(np.linalg.norm(selected-centre,axis=1))];pos=project(np.array([origin[:2]+(closest+.5)*v]))[0];text=str(lab);box=draw.textbbox((0,0),text,font=font);bw,bh=box[2]+12,30
  draw.rounded_rectangle((pos[0]-bw/2,pos[1]-bh/2,pos[0]+bw/2,pos[1]+bh/2),radius=5,fill=(255,255,255),outline=(175,183,193));draw.text((pos[0]-bw/2+6,pos[1]-bh/2+1),text,font=font,fill=(30,40,55))
 draw.text((30,25),f'Floor {f} | APF regions over measured point cloud',font=font,fill=(30,40,55));draw.text((30,60),'Region tint 18% | original top orientation | no filled cloud gaps',font=small,fill=(90,100,110));draw.text((30,H-45),'Cloud height slice: floor + 0.15 to 2.20 m; tiny regions retained, labels hidden',font=small,fill=(90,100,110))
 image.save(p/'cloud_overlay_light.png');panels.append(image)
wide=Image.new('RGB',(W*3,H),(250,251,253))
for i,im in enumerate(panels):wide.paste(im,(i*W,0))
wide.save(root/'three_floor_cloud_overlay_wide.png')
# Separate wide PF/region review: three floors as columns, regions above PF.
review=Image.new('RGB',(3600,1800),'white');draw=ImageDraw.Draw(review)
for i,f in enumerate([1,2,3]):
 p=root/f'L{f}_corrected_fullvis';draw.text((i*1200+30,15),f'Floor {f}: room regions / potential field',font=font,fill=(25,35,45))
 for row,name in enumerate(['room_regions_numbered.png','potential_field.png']):
  im=Image.open(p/name);im.thumbnail((1120,830),Image.Resampling.NEAREST);review.paste(im,(i*1200+(1200-im.width)//2,60+row*860+(830-im.height)//2))
review.save(root/'three_floor_regions_pf_wide.png')
print('Saved wide review and faint cloud overlay')
