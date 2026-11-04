"""Tight two-row paper figure using a true lower-elevation 3D projection."""
from pathlib import Path
import base64
import html
import io
import json
import math
import shutil

import cairo
import gi
gi.require_version('Rsvg','2.0')
from gi.repository import Rsvg
import numpy as np
from PIL import Image,ImageFont

HERE=Path(__file__).resolve().parent
ASSETS=HERE/'assets'
PREVIOUS=HERE.parent/'spatial_callouts_v5/assets'
W=3600
BLUE='#2379B9';ORANGE='#DB8739';INK='#242A2F';GREY='#6B7680'
scene=json.loads((ASSETS/'scene_az0.json').read_text())
for name in ['task_1_laptop.jpg','task_2_toolbox.jpg','task_3_microwave.jpg','task_4_cup.jpg']:
    if not (ASSETS/name).exists():shutil.copyfile(PREVIOUS/name,ASSETS/name)

# Reprojected at 22 degrees, then cropped with one uniform display scale.
# Neutral shading changes color only; alpha and image geometry are unchanged.
source=Image.open(ASSETS/'scene_az0.png').convert('RGBA')
box=source.getchannel('A').getbbox()
crop=(max(0,box[0]-20),max(0,box[1]-20),min(source.width,box[2]+20),min(source.height,box[3]+20))
pixels=np.array(source.crop(crop))
lum=pixels[:,:,:3]@np.array([.2126,.7152,.0722]);level=180+lum*.22
pixels[:,:,:3]=np.stack([level-4,level,level+3],axis=-1).astype('uint8')
neutral=ASSETS/'scene_neutral_cropped.png';Image.fromarray(pixels).save(neutral)
mx,my,mw=40,132,3520
ratio=mw/(crop[2]-crop[0]);mh=(crop[3]-crop[1])*ratio
photo_y=math.ceil(my+mh+18)
photo_height=395
caption_y=photo_y+photo_height+54
H=photo_y+photo_height+80
project=lambda p:(np.asarray(p)-np.array(crop[:2]))*ratio+[mx,my]
parts=[f'''<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink"
 width="180mm" height="{H*.05}mm" viewBox="0 0 {W} {H}">
 <title>Compact real-world multi-task execution</title>
 <desc>Recorded flight and calibrated observation poses in measured 3D geometry,
 above four aligned third-person task photographs. No room labels.</desc>
 <rect width="{W}" height="{H}" fill="white"/>''']
labels=[]


def text(x,y,value,size=48,color=INK,anchor='start',bold=False,italic=False,sans=False):
    family='Liberation Sans' if sans else 'Liberation Serif'
    stem='LiberationSans' if sans else 'LiberationSerif'
    style='Bold' if bold else 'Italic' if italic else 'Regular'
    font=ImageFont.truetype(f'/usr/share/fonts/truetype/liberation/{stem}-{style}.ttf',size)
    width=font.getlength(value)
    left=x-(width/2 if anchor=='middle' else width if anchor=='end' else 0)
    assert left>=0 and left+width<=W+1,(value,left,width)
    assert y-size>=0 and y<=H,(value,y)
    labels.append({'text':value,'x':left,'y':y-size,'width':width,'height':size})
    parts.append(f'<text x="{x:.3f}" y="{y:.3f}" font-family="{family}" font-size="{size}" '
                 f'font-weight="{"bold" if bold else "normal"}" font-style="{"italic" if italic else "normal"}" '
                 f'fill="{color}" text-anchor="{anchor}">{html.escape(value)}</text>')


def line(points,color,width=3):
    coordinates=' '.join(f'{x:.3f},{y:.3f}' for x,y in points)
    parts.append(f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="{width}" '
                 f'stroke-linecap="round" stroke-linejoin="round"/>')


def polygon(points,fill,stroke='none',width=1,opacity=1):
    coordinates=' '.join(f'{x:.3f},{y:.3f}' for x,y in points)
    parts.append(f'<polygon points="{coordinates}" fill="{fill}" fill-opacity="{opacity}" '
                 f'stroke="{stroke}" stroke-width="{width}" stroke-linejoin="round"/>')


def circle(x,y,r,fill,stroke='white',width=3):
    parts.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>')


def image(path,x,y,width,height,fit='none'):
    im=Image.open(path)
    im.thumbnail((round(width*1.3),round(height*1.3)),Image.Resampling.LANCZOS)
    fmt='PNG' if im.mode=='RGBA' or path.suffix=='.png' else 'JPEG'
    stream=io.BytesIO();im.save(stream,format=fmt,**({'quality':96} if fmt=='JPEG' else {}))
    encoded=base64.b64encode(stream.getvalue()).decode()
    parts.append(f'<image x="{x}" y="{y}" width="{width}" height="{height}" preserveAspectRatio="{fit}" '
                 f'xlink:href="data:image/{fmt.lower()};base64,{encoded}"/>')


text(W/2,61,'“Check the laptop, toolbox, microwave and cup; find the laptop if it is absent.”',54,anchor='middle',italic=True)
line([(45,100),(104,100)],BLUE,8);text(120,114,'Recorded flight',40,GREY,sans=True)
line([(450,100),(509,100)],ORANGE,8);text(525,114,'Recovery (A → 1)',40,GREY,sans=True)
circle(980,100,9,'#2D9C85','white',2);text(1000,114,'Start',40,GREY,sans=True)
bar=scene['scale_pixels_per_meter']*ratio*5
bar_x=3490-bar
line([(bar_x,102),(3490,102)],GREY,3)
line([(bar_x,95),(bar_x,109)],GREY,3)
line([(3490,95),(3490,109)],GREY,3)
text(3510,115,'5 m',38,GREY,sans=True)

image(neutral,mx,my,mw,mh)
route=project(scene['route'])
line(route,'white',19);line(route,BLUE,10)
times=np.array(scene['route_bag_seconds']);begin,end=scene['recovery_bag_seconds']
mask=(times>=begin)&(times<=end)
line(route[mask],'white',19);line(route[mask],ORANGE,10)
for seconds,color in [(59,BLUE),(82,ORANGE),(130,BLUE),(181,BLUE)]:
    index=int(np.argmin(abs(times-seconds)))
    v=route[min(len(route)-1,index+10)]-route[max(0,index-10)]
    if np.linalg.norm(v)<1:continue
    v/=np.linalg.norm(v);normal=np.array([-v[1],v[0]]);p=route[index]
    polygon([p+v*18,p-v*11+normal*10,p-v*11-normal*10],color,'white',2)

offsets={'A':(-25,-63),'1':(16,-63),'2':(47,58),'3':(-30,64),'4':(-35,62)}
for marker in scene['markers']:
    center=project(marker['camera']);corners=project(marker['frustum_corners']);body=project(marker['point'])
    color=ORANGE if marker['id']=='A' else BLUE
    polygon(corners,'#DEEDF6',color,2.5,.35)
    for corner in corners:line([center,corner],color,2.5)
    circle(*body,8,color,'white',2.5)
    label=body+np.array(offsets[marker['id']])
    line([body,label],color,2)
    circle(*label,24,'white',color,2)
    text(label[0],label[1]+15,marker['id'],44,color,anchor='middle',bold=True,sans=True)
circle(*route[0],11,'#2D9C85','white',3)

photos=[('task_1_laptop.jpg','(1) Laptop found','xMidYMid slice'),
        ('task_2_toolbox.jpg','(2) Toolbox','xMidYMin slice'),
        ('task_3_microwave.jpg','(3) Microwave','xMidYMid slice'),
        ('task_4_cup.jpg','(4) Cup','xMidYMid slice')]
pw=869
for i,(filename,label,fit) in enumerate(photos):
    x=40+i*883
    image(ASSETS/filename,x,photo_y,pw,photo_height,fit)
    text(x+pw/2,caption_y,label,50,anchor='middle')

parts.append('</svg>')
svg=HERE/'main_figure.svg';svg.write_text('\n'.join(parts))
handle=Rsvg.Handle.new_from_file(str(svg))
viewport=Rsvg.Rectangle();viewport.x=0;viewport.y=0;viewport.width=W;viewport.height=H
surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,W,H)
assert handle.render_document(cairo.Context(surface),viewport)
surface.write_to_png(str(HERE/'main_figure.png'));surface.finish()
png=Image.open(HERE/'main_figure.png').convert('RGB');png.save(HERE/'main_figure.png',dpi=(508,508))
png.resize((1800,round(H/2)),Image.Resampling.LANCZOS).save(HERE/'preview.jpg',quality=96)
scale=.05*72/25.4
surface=cairo.PDFSurface(str(HERE/'main_figure.pdf'),W*scale,H*scale)
context=cairo.Context(surface);context.scale(scale,scale)
assert handle.render_document(context,viewport);context.show_page();surface.finish()
(HERE/'layout_audit.json').write_text(json.dumps({'canvas':[W,H],'source_crop':crop,'map_ratio':ratio,
    'map_rect':[mx,my,mw,mh],'photo_y':photo_y,'height_reduction_vs_v5':1-H/2190,'labels':labels},indent=2))
print(f'Built {W} x {H} compact teaser; height reduced {(1-H/2190)*100:.1f}% vs V5.')
