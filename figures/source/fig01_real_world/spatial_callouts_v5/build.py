"""Spatially arranged real photographs around a neutral measured 3D scene."""
from pathlib import Path
import base64
import html
import io
import json
import shutil

import cairo
import gi
gi.require_version('Rsvg','2.0')
from gi.repository import Rsvg
import numpy as np
from PIL import Image,ImageFont

HERE=Path(__file__).resolve().parent
ASSETS=HERE/'assets';ASSETS.mkdir(exist_ok=True)
UPSTREAM=HERE.parent/'white_teaser_v4/assets'
W,H=3600,2190
BLUE='#2379B9';ORANGE='#DB8739';INK='#20272D';GREY='#77838D'
for filename in ['scene_az-10.json','task_2_toolbox.jpg','task_3_microwave.jpg','task_4_cup.jpg']:
    if not (ASSETS/filename).exists():shutil.copyfile(UPSTREAM/filename,ASSETS/filename)
if not (ASSETS/'task_1_laptop.jpg').exists():
    shutil.copyfile(HERE.parent/'trajectory_photo_draft/assets/task_1_laptop.jpg',ASSETS/'task_1_laptop.jpg')
hardware=ASSETS/'uav_platform.png'
if not hardware.exists():shutil.copyfile(HERE.parent/'hardware_assets/drone_cutout.png',hardware)

# Neutral material is a display treatment of the existing projected geometry.
# Preserve every pixel's alpha: no points are added, removed or repositioned.
neutral=ASSETS/'measured_scene_neutral.png'
if not neutral.exists():
    source=np.array(Image.open(UPSTREAM/'scene_az-10.png').convert('RGBA'))
    luminance=source[:,:,:3]@np.array([.2126,.7152,.0722])
    level=180+luminance*.22
    source[:,:,:3]=np.stack([level-4,level,level+3],axis=-1).astype('uint8')
    Image.fromarray(source).save(neutral)
scene=json.loads((ASSETS/'scene_az-10.json').read_text())
parts=[f'''<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink"
 width="180mm" height="{H*.05}mm" viewBox="0 0 {W} {H}">
 <title>Real-world task-conditioned flight and recovery</title>
 <desc>Neutral rendering of the measured RGB-D scene, actual recorded flight,
 and four spatially linked real observations. Hardware image is a separate inset.</desc>
 <rect width="{W}" height="{H}" fill="white"/>''']
audit=[]


def text(x,y,value,size=52,color=INK,anchor='start',bold=False,italic=False,sans=False):
    family='Liberation Sans' if sans else 'Liberation Serif'
    stem='LiberationSans' if sans else 'LiberationSerif'
    style='Bold' if bold else 'Italic' if italic else 'Regular'
    font=ImageFont.truetype(f'/usr/share/fonts/truetype/liberation/{stem}-{style}.ttf',size)
    width=font.getlength(value)
    left=x-(width/2 if anchor=='middle' else width if anchor=='end' else 0)
    assert left>=0 and left+width<=W+1,(value,left,width)
    assert y-size>=0 and y<=H,(value,y)
    audit.append({'text':value,'left':left,'top':y-size,'width':width,'height':size})
    parts.append(f'<text x="{x:.3f}" y="{y:.3f}" fill="{color}" font-family="{family}" '
                 f'font-size="{size}" font-weight="{"bold" if bold else "normal"}" '
                 f'font-style="{"italic" if italic else "normal"}" text-anchor="{anchor}">{html.escape(value)}</text>')


def polyline(points,color,width=3,opacity=1):
    coordinates=' '.join(f'{x:.3f},{y:.3f}' for x,y in points)
    parts.append(f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="{width}" '
                 f'stroke-opacity="{opacity}" stroke-linecap="round" stroke-linejoin="round"/>')


def polygon(points,fill,stroke='none',width=1,opacity=1):
    coordinates=' '.join(f'{x:.3f},{y:.3f}' for x,y in points)
    parts.append(f'<polygon points="{coordinates}" fill="{fill}" fill-opacity="{opacity}" '
                 f'stroke="{stroke}" stroke-width="{width}" stroke-linejoin="round"/>')


def circle(x,y,r,fill,stroke='white',width=3):
    parts.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>')


def image(path,x,y,width,height,fit='xMidYMid slice'):
    source=Image.open(path)
    source.thumbnail((round(width*1.35),round(height*1.35)),Image.Resampling.LANCZOS)
    fmt='PNG' if source.mode=='RGBA' or path.suffix=='.png' else 'JPEG'
    stream=io.BytesIO();source.save(stream,format=fmt,**({'quality':96} if fmt=='JPEG' else {}))
    data=base64.b64encode(stream.getvalue()).decode()
    parts.append(f'<image x="{x}" y="{y}" width="{width}" height="{height}" preserveAspectRatio="{fit}" '
                 f'xlink:href="data:image/{fmt.lower()};base64,{data}"/>')


def curve(start,c1,c2,end,color='#B4BEC6',width=3):
    parts.append(f'<path d="M {start[0]:.2f},{start[1]:.2f} C {c1[0]:.2f},{c1[1]:.2f} '
                 f'{c2[0]:.2f},{c2[1]:.2f} {end[0]:.2f},{end[1]:.2f}" fill="none" '
                 f'stroke="{color}" stroke-width="{width}"/>')


# The instruction occupies the quiet central space between upper photographs.
text(1800,183,'“Check the laptop, toolbox, microwave and cup.',61,anchor='middle',italic=True)
text(1800,270,'If the laptop is absent, find it.”',64,anchor='middle',italic=True)

# Single central spatial view. No floor grid or semantic room labels.
mx,my,mw=400,430,2800
ratio=mw/scene['size'][0];mh=scene['size'][1]*ratio
project=lambda p:np.asarray(p)*ratio+[mx,my]
image(neutral,mx,my,mw,mh,'none')
route=project(scene['route'])
polyline(route,'white',20);polyline(route,BLUE,10)
times=np.array(scene['route_bag_seconds']);begin,end=scene['recovery_bag_seconds']
recovery=(times>=begin)&(times<=end)
polyline(route[recovery],'white',20);polyline(route[recovery],ORANGE,10)
for seconds,color in [(59,BLUE),(82,ORANGE),(130,BLUE),(181,BLUE)]:
    index=int(np.argmin(abs(times-seconds)))
    v=route[min(len(route)-1,index+10)]-route[max(0,index-10)]
    if np.linalg.norm(v)<1:continue
    v=v/np.linalg.norm(v);normal=np.array([-v[1],v[0]]);p=route[index]
    polygon([p+v*20,p-v*12+normal*11,p-v*12-normal*11],color,'white',2)

points={m['id']:project(m['point']) for m in scene['markers']}
# Deliberately placed callouts follow the spatial layout, rather than task order.
curve([860,285],[1060,285],[820,790],points['4'])
curve([2740,285],[2540,285],[2880,660],points['1'])
curve([460,1630],[460,1450],[1170,1220],points['3'])
curve([3140,1630],[3140,1450],[2920,1270],points['2'])

offsets={'A':(-40,-96),'1':(38,-75),'2':(50,78),'3':(-46,70),'4':(-45,70)}
for marker in scene['markers']:
    center=project(marker['camera']);corners=project(marker['frustum_corners']);body=points[marker['id']]
    color=ORANGE if marker['id']=='A' else BLUE
    polygon(corners,'#DEEDF6',color,2.5,.40)
    for corner in corners:polyline([center,corner],color,2.5)
    circle(*body,9,color,'white',3)
    label=body+np.array(offsets[marker['id']])
    polyline([body,label],color,2)
    circle(*label,25,'white',color,2)
    text(label[0],label[1]+15,marker['id'],44,color,anchor='middle',bold=True,sans=True)
circle(*route[0],12,'#2D9C85','white',3)

# Independent real image frames: no cross-camera or perspective compositing.
photos=[(60,60,'task_4_cup.jpg','(4) Cup'),
        (2740,60,'task_1_laptop.jpg','(1) Laptop found'),
        (60,1630,'task_3_microwave.jpg','(3) Microwave'),
        (2740,1630,'task_2_toolbox.jpg','(2) Toolbox')]
for x,y,filename,label in photos:
    image(ASSETS/filename,x,y,800,450)
    text(x+400,y+515,label,52,anchor='middle')

# A separate hardware inset identifies the actual platform without placing a
# manufactured drone instance inside the reconstructed scene.
image(hardware,1630,1630,560,477,'xMidYMid meet')
text(1910,2160,'UAV platform',47,anchor='middle')

polyline([(1080,1760),(1150,1760)],BLUE,9)
text(1170,1776,'Recorded flight',44,GREY,sans=True)
polyline([(1080,1840),(1150,1840)],ORANGE,9)
text(1170,1856,'Recovery search',44,GREY,sans=True)
circle(1115,1920,11,'#2D9C85','white',2)
text(1170,1936,'Start',44,GREY,sans=True)
bar=scene['scale_pixels_per_meter']*ratio*5
polyline([(1080,2020),(1080+bar,2020)],GREY,3)
polyline([(1080,2011),(1080,2029)],GREY,3)
polyline([(1080+bar,2011),(1080+bar,2029)],GREY,3)
text(1080+bar+30,2035,'5 m',42,GREY,sans=True)
text(2330,1795,'A → 1',58,ORANGE,anchor='middle')
text(2330,1860,'Laptop search',44,GREY,anchor='middle',sans=True)

parts.append('</svg>')
svg=HERE/'main_figure.svg';svg.write_text('\n'.join(parts))
handle=Rsvg.Handle.new_from_file(str(svg))
viewport=Rsvg.Rectangle();viewport.x=0;viewport.y=0;viewport.width=W;viewport.height=H
surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,W,H)
assert handle.render_document(cairo.Context(surface),viewport)
surface.write_to_png(str(HERE/'main_figure.png'));surface.finish()
png=Image.open(HERE/'main_figure.png').convert('RGB');png.save(HERE/'main_figure.png',dpi=(508,508))
png.resize((1800,1095),Image.Resampling.LANCZOS).save(HERE/'preview.jpg',quality=96)
scale=.05*72/25.4
surface=cairo.PDFSurface(str(HERE/'main_figure.pdf'),W*scale,H*scale)
context=cairo.Context(surface);context.scale(scale,scale)
assert handle.render_document(context,viewport);context.show_page();surface.finish()
(HERE/'layout_audit.json').write_text(json.dumps({'canvas':[W,H],'map_ratio':ratio,'labels':audit},indent=2))
print('Built spatial-callout figure with neutral measured geometry.')
