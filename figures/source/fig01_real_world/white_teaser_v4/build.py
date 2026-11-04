"""White-background paper teaser: measured 3D scene, actual route, real views."""
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
ASSETS=HERE/'assets'
W,H=3600,2290
TEAL='#256F7D'
ORANGE='#C97B36'
INK='#24282B'
GREY='#647079'
selected='scene_az-10'
scene=json.loads((ASSETS/f'{selected}.json').read_text())
previous=HERE.parent/'scene_teaser_v3/assets'
for filename in ['recovery_found.jpg','task_2_toolbox.jpg','task_3_microwave.jpg','task_4_cup.jpg']:
    if not (ASSETS/filename).exists():shutil.copyfile(previous/filename,ASSETS/filename)

parts=[f'''<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink"
 width="180mm" height="{H*.05}mm" viewBox="0 0 {W} {H}">
 <title>Real-world multi-task flight with conditional recovery</title>
 <desc>A white-background view of measured RGB-D geometry, recorded flight and
 calibrated observation poses. Real onboard and third-person target views appear below.</desc>
 <rect width="{W}" height="{H}" fill="white"/>
 </svg>'''.replace('</svg>','')]
bounds=[]


def text(x,y,value,size=50,color=INK,bold=False,italic=False,anchor='start',sans=False):
    family='Liberation Sans' if sans else 'Liberation Serif'
    filename='LiberationSans' if sans else 'LiberationSerif'
    style='Bold' if bold else 'Italic' if italic else 'Regular'
    font=ImageFont.truetype(f'/usr/share/fonts/truetype/liberation/{filename}-{style}.ttf',size)
    width=font.getlength(value)
    left=x-(width/2 if anchor=='middle' else width if anchor=='end' else 0)
    assert 0<=left and left+width<=W+1,(value,left,width)
    assert size<=y<=H,(value,y)
    bounds.append({'text':value,'x':left,'y':y-size,'width':width,'height':size})
    parts.append(f'<text x="{x:.3f}" y="{y:.3f}" fill="{color}" font-family="{family}" '
                 f'font-size="{size}" font-weight="{"bold" if bold else "normal"}" '
                 f'font-style="{"italic" if italic else "normal"}" text-anchor="{anchor}">{html.escape(value)}</text>')


def polygon(points,fill,stroke='none',width=1,opacity=1):
    coordinates=' '.join(f'{x:.3f},{y:.3f}' for x,y in points)
    parts.append(f'<polygon points="{coordinates}" fill="{fill}" fill-opacity="{opacity}" '
                 f'stroke="{stroke}" stroke-width="{width}" stroke-linejoin="round"/>')


def line(points,color,width=3,opacity=1):
    coordinates=' '.join(f'{x:.3f},{y:.3f}' for x,y in points)
    parts.append(f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="{width}" '
                 f'stroke-opacity="{opacity}" stroke-linecap="round" stroke-linejoin="round"/>')


def circle(x,y,radius,fill,stroke='white',width=3):
    parts.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="{radius}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>')


def image(path,x,y,width,height,fit='none'):
    source=Image.open(path)
    source.thumbnail((round(width*1.3),round(height*1.3)),Image.Resampling.LANCZOS)
    fmt='PNG' if source.mode=='RGBA' or path.suffix=='.png' else 'JPEG'
    stream=io.BytesIO();source.save(stream,format=fmt,**({'quality':96} if fmt=='JPEG' else {}))
    encoded=base64.b64encode(stream.getvalue()).decode()
    parts.append(f'<image x="{x}" y="{y}" width="{width}" height="{height}" preserveAspectRatio="{fit}" '
                 f'xlink:href="data:image/{fmt.lower()};base64,{encoded}"/>')


text(W/2,87,'“Check the laptop, toolbox, microwave and cup; if the laptop is absent, find it.”',60,italic=True,anchor='middle')

# Compact legend, away from the geometry; no title block or room labels.
line([(2380,158),(2450,158)],TEAL,8)
text(2470,173,'Flight',44,GREY,sans=True)
line([(2675,158),(2745,158)],ORANGE,8)
text(2765,173,'Recovery',44,GREY,sans=True)
circle(3040,158,11,'#479B83','white',2)
text(3064,173,'Start',44,GREY,sans=True)

# Main 3D panel. Ground grid is a metric coordinate reference, not added geometry.
map_x,map_y,map_w=60,200,3480
ratio=map_w/scene['size'][0]
map_h=scene['size'][1]*ratio
project=lambda p:np.asarray(p)*ratio+[map_x,map_y]
polygon(project(scene['floor_polygon']),'#FAFBFC','#EEF1F3',1.5)
for grid_line in scene['floor_grid']:line(project(grid_line),'#E7ECEF',1.5)
image(ASSETS/f'{selected}.png',map_x,map_y,map_w,map_h)
route=project(scene['route'])
line(route,'white',18)
line(route,TEAL,9)
times=np.array(scene['route_bag_seconds'])
begin,end=scene['recovery_bag_seconds']
recovery=(times>=begin)&(times<=end)
line(route[recovery],'white',18)
line(route[recovery],ORANGE,9)

for seconds,color in [(59,TEAL),(82,ORANGE),(130,TEAL),(181,TEAL)]:
    index=int(np.argmin(abs(times-seconds)))
    direction=route[min(len(route)-1,index+8)]-route[max(0,index-8)]
    if np.linalg.norm(direction)<1:continue
    direction=direction/np.linalg.norm(direction)
    perpendicular=np.array([-direction[1],direction[0]])
    point=route[index]
    polygon([point+direction*19,point-direction*13+perpendicular*10,point-direction*13-perpendicular*10],color,'white',2.5)

label_offsets={'A':(-55,-100),'1':(35,-85),'2':(55,84),'3':(-55,80),'4':(-45,92)}
for marker in scene['markers']:
    center=project(marker['camera'])
    corners=project(marker['frustum_corners'])
    body=project(marker['point'])
    color=ORANGE if marker['id']=='A' else TEAL
    polygon(corners,'#D9EDF1',color,2.6,.32)
    for corner in corners:line([center,corner],color,2.6)
    circle(*body,9,color,'white',2.5)
    offset=np.array(label_offsets[marker['id']]);label=body+offset
    line([body+offset*.18,label-offset*.26],color,2.3)
    circle(*label,27,'white',color,2.5)
    text(label[0],label[1]+16,marker['id'],46,color,bold=True,anchor='middle',sans=True)
circle(*route[0],13,'#479B83','white',3)

# Sparse annotation and a scale reference, outside the scene.
text(78,1562,'A: initial laptop check',44,GREY,sans=True)
sx,sy=3130,1546
length=scene['scale_pixels_per_meter']*ratio*5
line([(sx,sy),(sx+length,sy)],GREY,3)
line([(sx,sy-8),(sx,sy+8)],GREY,3)
line([(sx+length,sy-8),(sx+length,sy+8)],GREY,3)
text(sx+length/2,sy-20,'5 m',42,GREY,anchor='middle',sans=True)

# One aligned row of real observations, with labels below the images.
photos=[('recovery_found.jpg','1  Laptop found (onboard)'),
        ('task_2_toolbox.jpg','2  Toolbox'),
        ('task_3_microwave.jpg','3  Microwave'),
        ('task_4_cup.jpg','4  Cup')]
for i,(filename,label) in enumerate(photos):
    x=60+i*880;y=1644;pw,ph=840,520
    image(ASSETS/filename,x,y,pw,ph,'xMidYMid slice')
    text(x+pw/2,2210,label,52,INK,anchor='middle')

parts.append('</svg>')
svg=HERE/'main_figure.svg';svg.write_text('\n'.join(parts))
handle=Rsvg.Handle.new_from_file(str(svg))
viewport=Rsvg.Rectangle();viewport.x=0;viewport.y=0;viewport.width=W;viewport.height=H
surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,W,H)
assert handle.render_document(cairo.Context(surface),viewport)
surface.write_to_png(str(HERE/'main_figure.png'));surface.finish()
png=Image.open(HERE/'main_figure.png').convert('RGB')
png.save(HERE/'main_figure.png',dpi=(508,508))
png.resize((W//2,H//2),Image.Resampling.LANCZOS).save(HERE/'preview.jpg',quality=96)
points_per_pixel=.05*72/25.4
surface=cairo.PDFSurface(str(HERE/'main_figure.pdf'),W*points_per_pixel,H*points_per_pixel)
context=cairo.Context(surface);context.scale(points_per_pixel,points_per_pixel)
assert handle.render_document(context,viewport);context.show_page();surface.finish()
(HERE/'layout_audit.json').write_text(json.dumps({'canvas':[W,H],'map_scale':ratio,'labels':bounds},indent=2))
print('Built white-background 3D teaser.')
