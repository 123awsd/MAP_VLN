"""Measured voxel overview + recovery detail + real observations."""
from pathlib import Path
import base64
import html
import io
import json
import math

import cairo
import gi
gi.require_version('Rsvg','2.0')
from gi.repository import Rsvg
import numpy as np
from PIL import Image,ImageFont

HERE=Path(__file__).resolve().parent
ASSETS=HERE/'assets'
W=3600
BLUE='#2377AF';ORANGE='#D67F2E';INK='#222B31';GREY='#6D7880'
data={name:json.loads((ASSETS/f'{name}.json').read_text()) for name in ['global','recovery']}
crop={};rasters={}
for name in ['global','recovery']:
    im=Image.open(ASSETS/f'{name}.png').convert('RGBA');box=im.getchannel('A').getbbox()
    pad=10
    crop[name]=(max(0,box[0]-pad),max(0,box[1]-pad),min(im.width,box[2]+pad),min(im.height,box[3]+pad))
    raster=ASSETS/f'{name}_cropped.png';im.crop(crop[name]).save(raster);rasters[name]=raster

gx,gy,gw=40,100,2720
global_ratio=gw/(crop['global'][2]-crop['global'][0])
gh=(crop['global'][3]-crop['global'][1])*global_ratio
legend_y=math.ceil(gy+gh+38)
photo_y=legend_y+30
photo_h=350
caption_y=photo_y+photo_h+53
H=max(caption_y+30,1620)
rx,ry,rw=2820,105,740
recovery_ratio=rw/(crop['recovery'][2]-crop['recovery'][0])
rh=(crop['recovery'][3]-crop['recovery'][1])*recovery_ratio
project_global=lambda p:(np.asarray(p)-np.array(crop['global'][:2]))*global_ratio+[gx,gy]
project_recovery=lambda p:(np.asarray(p)-np.array(crop['recovery'][:2]))*recovery_ratio+[rx,ry]

parts=[f'''<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink"
 width="180mm" height="{H*.05}mm" viewBox="0 0 {W} {H}">
 <title>Measured three-dimensional multi-task flight and recovery detail</title>
 <desc>Archived occupied-voxel geometry and recorded body trajectory with
 calibrated observation poses. Six real images provide task and recovery evidence.</desc>
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
    labels.append({'text':value,'left':left,'top':y-size,'width':width,'height':size})
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


def trajectory(name,project,only_recovery=False):
    meta=data[name];path=project(meta['route']);times=np.array(meta['times'])
    start,end=meta['recovery_seconds'];mask=(times>=start)&(times<=end)
    if not only_recovery:
        line(path,'white',17);line(path,BLUE,8)
    line(path[mask],'white',18);line(path[mask],ORANGE,9)
    for seconds,color in ([(82,ORANGE)] if only_recovery else [(59,BLUE),(82,ORANGE),(130,BLUE),(181,BLUE)]):
        i=int(np.argmin(abs(times-seconds)))
        vector=path[min(len(path)-1,i+8)]-path[max(0,i-8)]
        if np.linalg.norm(vector)<1:continue
        vector=vector/np.linalg.norm(vector);normal=np.array([-vector[1],vector[0]]);p=path[i]
        polygon([p+vector*18,p-vector*11+normal*10,p-vector*11-normal*10],color,'white',2)
    if not only_recovery:circle(*path[0],10,'#369984','white',3)


def poses(name,project,only_recovery=False):
    offsets={'A':(-25,-65),'1':(30,-60),'2':(42,65),'3':(-30,65),'4':(-42,65)}
    for marker in data[name]['markers']:
        if only_recovery and marker['id'] not in ['A','1']:continue
        center=project(marker['camera']);corners=project(marker['corners']);body=project(marker['body'])
        color=ORANGE if marker['id']=='A' else BLUE
        polygon(corners,'#DCEBF4',color,2.4,.28)
        for corner in corners:line([center,corner],color,2.4)
        circle(*body,7,color,'white',2.5)
        offset=np.array(offsets[marker['id']],dtype=float)
        if only_recovery:offset*=.8
        label=body+offset
        line([body,label],color,2)
        circle(*label,23,'white',color,2)
        text(label[0],label[1]+14,marker['id'],42,color,anchor='middle',bold=True,sans=True)


text(1400,64,'“Check the laptop, toolbox, microwave and cup; find the laptop if it is absent.”',52,anchor='middle',italic=True)
text(2820,64,'Recovery: A → 1',50,ORANGE)

image(rasters['global'],gx,gy,gw,gh)
trajectory('global',project_global);poses('global',project_global)
line([(60,legend_y-13),(120,legend_y-13)],BLUE,8);text(138,legend_y,'Recorded flight',40,GREY,sans=True)
line([(480,legend_y-13),(540,legend_y-13)],ORANGE,8);text(558,legend_y,'Recovery',40,GREY,sans=True)
circle(830,legend_y-13,9,'#369984','white',2);text(850,legend_y,'Start',40,GREY,sans=True)
bar=data['global']['scale_pixels_per_m']*global_ratio*5
line([(2420,legend_y-13),(2420+bar,legend_y-13)],GREY,3)
text(2440+bar,legend_y,'5 m',38,GREY,sans=True)

image(rasters['recovery'],rx,ry,rw,rh)
trajectory('recovery',project_recovery,True);poses('recovery',project_recovery,True)

# Right-hand evidence column, kept close to the enlarged recovery scene.
evidence_a_y=math.ceil(ry+rh+84)
evidence_h=280
text(rx,evidence_a_y-24,'A  Initial check: not found',43,INK)
image(ASSETS/'absent_onboard.jpg',rx,evidence_a_y,rw,evidence_h,'xMidYMid slice')
evidence_one_y=evidence_a_y+evidence_h+88
text(rx,evidence_one_y-24,'1  Laptop found',43,INK)
image(ASSETS/'found_onboard.jpg',rx,evidence_one_y,rw,evidence_h,'xMidYMid slice')
assert evidence_one_y+evidence_h<=H-15,(evidence_one_y,H)

# Four full-width task examples fill the overview's lower edge.
specs=[('task_1_laptop.jpg','(1) Laptop','xMidYMid slice'),
       ('task_2_toolbox.jpg','(2) Toolbox','xMidYMin slice'),
       ('task_3_microwave.jpg','(3) Microwave','xMidYMid slice'),
       ('task_4_cup.jpg','(4) Cup','xMidYMid slice')]
pw=671;gap=12
for i,(filename,label,fit) in enumerate(specs):
    x=40+i*(pw+gap)
    image(ASSETS/filename,x,photo_y,pw,photo_h,fit)
    text(x+pw/2,caption_y,label,46,anchor='middle')

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
(HERE/'layout_audit.json').write_text(json.dumps({'canvas':[W,H],'crops':crop,
    'global_rect':[gx,gy,gw,gh],'recovery_rect':[rx,ry,rw,rh],
    'global_ratio':global_ratio,'recovery_ratio':recovery_ratio,'labels':labels},indent=2))
print(f'Built occupied-volume teaser: {W} x {H}.')
