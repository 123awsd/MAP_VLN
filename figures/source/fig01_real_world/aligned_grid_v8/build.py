"""One six-column grid: aligned 3D views and six equal evidence frames."""
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
PREVIOUS=HERE.parent/'voxel_teaser_v7/assets'
W,H=3600,1250
M,GAP,PW=26,16,578
PLOT_Y,PLOT_H=150,680
PHOTO_Y,PHOTO_H=846,325
BLUE='#2377AF';ORANGE='#D67F2E';INK='#253039';GREY='#6D7880'
data={name:json.loads((ASSETS/f'{name}.json').read_text()) for name in ['global','recovery']}
for name in ['task_1_laptop.jpg','task_2_toolbox.jpg','task_3_microwave.jpg','task_4_cup.jpg','absent_onboard.jpg','found_onboard.jpg','evidence_sources.json']:
    if not (ASSETS/name).exists():shutil.copyfile(PREVIOUS/name,ASSETS/name)

parts=[f'''<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink"
 width="180mm" height="62.5mm" viewBox="0 0 {W} {H}">
 <title>Compact multi-task execution and recovery in one aligned grid</title>
 <desc>Two equally tall measured-geometry views share a row; six equally sized
 real image frames share the row below. All text uses one font family.</desc>
 <rect width="{W}" height="{H}" fill="white"/>''']
labels=[];view_transforms={};image_rects=[]


def text(x,y,value,size=50,color=INK,anchor='start',bold=False):
    face='/usr/share/fonts/truetype/liberation/LiberationSans-'+('Bold' if bold else 'Regular')+'.ttf'
    font=ImageFont.truetype(face,size);width=font.getlength(value)
    left=x-(width/2 if anchor=='middle' else width if anchor=='end' else 0)
    assert left>=0 and left+width<=W+1,(value,left,width)
    assert y-size>=0 and y<=H,(value,y)
    labels.append({'text':value,'x':left,'y':y-size,'width':width,'height':size})
    parts.append(f'<text x="{x:.3f}" y="{y:.3f}" font-family="Liberation Sans" font-size="{size}" '
                 f'font-weight="{"bold" if bold else "normal"}" fill="{color}" text-anchor="{anchor}">{html.escape(value)}</text>')


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
    im.thumbnail((round(width*1.35),round(height*1.35)),Image.Resampling.LANCZOS)
    fmt='PNG' if im.mode=='RGBA' or path.suffix=='.png' else 'JPEG'
    stream=io.BytesIO();im.save(stream,format=fmt,**({'quality':96} if fmt=='JPEG' else {}))
    encoded=base64.b64encode(stream.getvalue()).decode()
    parts.append(f'<image x="{x}" y="{y}" width="{width}" height="{height}" preserveAspectRatio="{fit}" '
                 f'xlink:href="data:image/{fmt.lower()};base64,{encoded}"/>')
    image_rects.append([x,y,width,height])


def prepare_view(name,slot):
    im=Image.open(ASSETS/f'{name}.png').convert('RGBA')
    box=im.getchannel('A').getbbox();pad=8
    box=(max(0,box[0]-pad),max(0,box[1]-pad),min(im.width,box[2]+pad),min(im.height,box[3]+pad))
    crop=im.crop(box);path=ASSETS/f'{name}_cropped.png';crop.save(path)
    x,y,w,h=slot;ratio=min(w/crop.width,h/crop.height)
    actual_w,actual_h=crop.width*ratio,crop.height*ratio
    ox,oy=x+(w-actual_w)/2,y+(h-actual_h)/2
    image(path,ox,oy,actual_w,actual_h)
    view_transforms[name]={'slot':slot,'crop':box,'ratio':ratio,'origin':[ox,oy]}
    return lambda points:(np.asarray(points)-np.array(box[:2]))*ratio+[ox,oy]


def draw_path(name,project,recovery_only=False):
    z=data[name];points=project(z['route']);times=np.asarray(z['times'])
    begin,end=z['recovery_seconds'];mask=(times>=begin)&(times<=end)
    if not recovery_only:line(points,'white',17);line(points,BLUE,8)
    line(points[mask],'white',18);line(points[mask],ORANGE,9)
    for seconds,color in ([(82,ORANGE)] if recovery_only else [(59,BLUE),(82,ORANGE),(130,BLUE),(181,BLUE)]):
        i=int(np.argmin(abs(times-seconds)))
        vector=points[min(len(points)-1,i+8)]-points[max(0,i-8)]
        if np.linalg.norm(vector)<1:continue
        vector=vector/np.linalg.norm(vector);normal=np.array([-vector[1],vector[0]]);p=points[i]
        polygon([p+vector*16,p-vector*10+normal*9,p-vector*10-normal*9],color,'white',2)
    if not recovery_only:circle(*points[0],9,'#369984','white',2)


def draw_poses(name,project,recovery_only=False):
    offsets={'A':(-15,-55),'1':(25,-58),'2':(40,54),'3':(-25,57),'4':(-28,57)}
    for marker in data[name]['markers']:
        if recovery_only and marker['id'] not in ['A','1']:continue
        body=project(marker['body']);center=project(marker['camera']);corners=project(marker['corners'])
        color=ORANGE if marker['id']=='A' else BLUE
        polygon(corners,'#DEEDF5',color,2.2,.22)
        for corner in corners:line([center,corner],color,2.2)
        circle(*body,6,color,'white',2)
        label=body+np.array(offsets[marker['id']],dtype=float)
        line([body,label],color,2)
        circle(*label,25,'white',color,2)
        text(label[0],label[1]+16,marker['id'],46,color,anchor='middle',bold=True)


text(M,64,'Instruction: Check the laptop, toolbox, microwave and cup; find the laptop if absent.',52)
text(M,124,'(a) Multi-task flight',48,bold=True)
line([(630,109),(685,109)],BLUE,8);text(703,124,'Flight',44,GREY)
line([(940,109),(995,109)],ORANGE,8);text(1013,124,'Recovery',44,GREY)
circle(1325,109,8,'#369984','white',2);text(1344,124,'Start',44,GREY)
text(2402,124,'(b) Recovery detail: A → 1',48,bold=True)

# Six-column grid. The overview spans four columns; the detail spans two.
global_project=prepare_view('global',[M,PLOT_Y,4*PW+3*GAP,PLOT_H])
recovery_project=prepare_view('recovery',[2402,PLOT_Y,2*PW+GAP,PLOT_H])
draw_path('global',global_project);draw_poses('global',global_project)
draw_path('recovery',recovery_project,True);draw_poses('recovery',recovery_project,True)

# Scale reference sits in the shared overview header, not a separate block.
ratio=view_transforms['global']['ratio']
bar=data['global']['scale_pixels_per_m']*ratio*5
line([(2110,109),(2110+bar,109)],GREY,3)
text(2130+bar,124,'5 m',40,GREY)

photos=[('task_1_laptop.jpg','(1) Laptop','xMidYMid slice'),
        ('task_2_toolbox.jpg','(2) Toolbox','xMidYMin slice'),
        ('task_3_microwave.jpg','(3) Microwave','xMidYMid slice'),
        ('task_4_cup.jpg','(4) Cup','xMidYMid slice'),
        ('absent_onboard.jpg','(A) Not found','xMidYMid slice'),
        ('found_onboard.jpg','(1) Found','xMidYMid slice')]
for i,(filename,label,fit) in enumerate(photos):
    x=M+i*(PW+GAP)
    image(ASSETS/filename,x,PHOTO_Y,PW,PHOTO_H,fit)
    text(x+PW/2,1221,label,52,anchor='middle')

parts.append('</svg>')
svg=HERE/'main_figure.svg';svg.write_text('\n'.join(parts))
handle=Rsvg.Handle.new_from_file(str(svg))
viewport=Rsvg.Rectangle();viewport.x=0;viewport.y=0;viewport.width=W;viewport.height=H
surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,W,H)
assert handle.render_document(cairo.Context(surface),viewport)
surface.write_to_png(str(HERE/'main_figure.png'));surface.finish()
png=Image.open(HERE/'main_figure.png').convert('RGB');png.save(HERE/'main_figure.png',dpi=(508,508))
png.resize((1800,625),Image.Resampling.LANCZOS).save(HERE/'preview.jpg',quality=96)
scale=.05*72/25.4
surface=cairo.PDFSurface(str(HERE/'main_figure.pdf'),W*scale,H*scale)
context=cairo.Context(surface);context.scale(scale,scale)
assert handle.render_document(context,viewport);context.show_page();surface.finish()
(HERE/'layout_audit.json').write_text(json.dumps({'canvas':[W,H],'grid':{'columns':6,'column_width':PW,'gutter':GAP,'margin':M},
    'view_transforms':view_transforms,'photo_rectangles':image_rects[-6:],'labels':labels,
    'height_reduction_vs_v7':1-H/1659},indent=2))
print(f'Built aligned six-column figure: {W} x {H}; height reduced {(1-H/1659)*100:.1f}%.')
