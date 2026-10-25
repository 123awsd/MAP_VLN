#!/usr/bin/python3
"""Fig. 8 preview using the unmodified official HM3D RGB floor image.

No publication export. Route and Qwen candidate are explanatory, not measured
planner output. Native Habitat RGB crops retain their original pixels.
"""
from pathlib import Path
import json
import html
import re
import math
import cairo
import gi
gi.require_version('Rsvg', '2.0')
from gi.repository import Rsvg
import compose_preview as c

OUT = Path(__file__).resolve().parent
ASSETS = OUT/'assets/hm3d_official'
G = json.loads((ASSETS/'geometry.json').read_text())
W, H = 1920, 1220
c.S.clear(); c.LABELS.clear()
c.C.update(ink='#273039', muted='#737A81', line='#CDD2D5',
           blue='#357D9B', amber='#B87A37', green='#32816A', purple='#6551BB')
add=c.add; line=c.line; path=c.path; rect=c.rect; circle=c.circle
LABELS=[]


def text(x,y,s,size=28,color='ink',weight=400,anchor='start'):
    LABELS.append(s)
    add(f'<text x="{x}" y="{y}" font-family="TeX Gyre Heros" font-size="{size}" '
        f'font-weight="{weight}" fill="{c.col(color)}" text-anchor="{anchor}">'
        f'{html.escape(s)}</text>')


def flag(x,y,color,label='',size=1,check=False):
    # The foot denotes the actual map location. Identical flags repeat on the axis.
    add(f'<g transform="translate({x},{y}) scale({size})">')
    circle(0,0,5.6,'white',color,2)
    path('M0,0 L0,-49','white',6)
    path('M0,0 L0,-49','ink',2)
    d='M1,-48 Q12,-55 23,-48 Q34,-41 45,-48 L45,-20 Q34,-13 23,-20 Q12,-27 1,-20 Z'
    add(f'<path d="{d}" fill="{c.col(color)}" stroke="white" stroke-width="3"/>')
    if check:
        # Restrained chequer marks identify the successful destination.
        for xx,yy in [(7,-46),(23,-40),(7,-30),(33,-28)]:
            rect(xx,yy,8,8,'white','none')
    elif label:
        text(22,-28,label,23,'white',700,'middle')
    add('</g>')


def icon(x,y,size):
    # Show only the official Qwen symbol via an SVG viewport; original PNG retained.
    add(f'<svg x="{x}" y="{y}" width="{size}" height="{size}" '
        'viewBox="245 290 1080 1050" overflow="hidden"><use href="#qwen"/></svg>')


def arrowhead(a,b,color,size=9):
    dx,dy=b[0]-a[0],b[1]-a[1]; n=math.hypot(dx,dy);dx/=n;dy/=n
    c.polygon([b,(b[0]-dx*size-dy*size*.52,b[1]-dy*size+dx*size*.52),
                 (b[0]-dx*size+dy*size*.52,b[1]-dy*size-dx*size*.52)],color)


def rgb(key,box,crop):
    v=c.rgb('official-'+key,key,crop,box)
    return v


def build():
    add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
    add('<title>Illustrative semantic recovery on an official HM3D RGB floor map</title><defs>')
    c.asset('floor',ASSETS/'topdown_floors.png',1280,1269)
    c.asset('qwen',ASSETS/'qwen3_official.png',3944,1555)
    c.asset('miss',c.FRAMES/'obs_000001.png',1600,1200)
    c.asset('found',c.FRAMES/'obs_000003.png',1600,1200)
    add('</defs><g stroke-linejoin="round" stroke-linecap="round">')
    rect(0,0,W,H,'white','none')
    text(36,43,'Find a cushion.',31,weight=600)
    text(W-35,43,'Illustrative sequence',23,'muted',anchor='end')

    m=c.View('official-floor','floor',1280,1269,(0,0,1280,1269),(18,65,1110,1100))
    m.image(.68)
    def p(q):return m.point((q[0]/1280,q[1]/1269))
    def pts(key):return [p(q) for q in G['route'][key]]
    # Straight segments preserve the navmesh-guided route geometry.
    for key,color,width,dash in [('approach','blue',3.6,None),('local','blue',3.6,None),
                                  ('recovery','green',4.1,None),('verify','green',4.1,None)]:
        pp=pts(key)
        line(pp,c='white',width=width+2,opacity=.88)
        line(pp,c=color,width=width,dash=dash)
    for key,i in [('approach',4),('recovery',4),('verify',1)]:
        pp=pts(key);a,b=pp[i],pp[i+1]
        end=(a[0]*.4+b[0]*.6,a[1]*.4+b[1]*.6)
        arrowhead(a,end,'blue' if key=='approach' else 'green',11)
    start=p(G['poses']['start']); miss=p(G['poses']['q2']);found=p(G['poses']['q3'])
    target=p(G['anchors']['cushion']);chair=p(G['anchors']['armchair'])
    circle(*p(G['poses']['q1']),4.7,'white','blue',2)
    flag(*start,'blue',size=.9)
    text(start[0]+51,start[1]-20,'Start',25,'blue')
    flag(*miss,'amber','1',size=1.03)
    flag(*found,'green',size=1.03,check=True)
    # A short sight line ties the positive observation to the target on the sofa.
    line([found,target],c='green',width=1.5,dash='4 5',opacity=.85)
    rect(target[0]-17,target[1]-18,34,36,'none','green',2)
    line([miss,chair],c='amber',width=1.3,dash='4 5',opacity=.8)

    # Two aligned observations, directly tied to the map with fine leaders.
    first=(1225,125,635,355);last=(1225,760,635,355)
    line([miss,(1135,miss[1]),(1182,450),(1225,450)],c='amber',width=1.35,dash='5 6',opacity=.8)
    line([found,(1126,found[1]),(1182,815),(1225,815)],c='green',width=1.35,dash='5 6',opacity=.8)
    text(1225,99,'1  Not seen',30,'amber',600)
    text(1225,734,'2  Found',30,'green',600)
    v1=rgb('miss',first,(120,390,1420,794))
    v2=rgb('found',last,(160,360,1420,794))
    # This box denotes the visible target, not a fabricated detector output.
    a=v2.point((952.35/1600,603.525/1200));b=v2.point(((952.35+207.8)/1600,(603.525+189.825)/1200))
    rect(*a,b[0]-a[0],b[1]-a[1],'none','green',2.1)
    # A single thought box expresses the candidate; it is explicitly illustrative.
    line([(1542,480),(1542,510)],c='muted',width=1.4)
    circle(1542,517,3,'white','muted',1.1)
    circle(1542,531,5,'white','muted',1.1)
    rect(1250,549,585,132,'white','#BDB5DA',1.5,8)
    icon(1272,572,66)
    text(1355,590,'Qwen',24,'purple',600)
    text(1355,628,'Most likely:',26)
    text(1501,628,'living-room sofa',26,weight=600)
    line([(1542,681),(1542,701)],c='muted',width=1.4)
    arrowhead((1542,681),(1542,706),'muted',8)

    # Compact flag time axis; no duplicate map thumbnails or explanatory panels.
    y=1180
    line([(90,y),(1090,y)],c='muted',width=1.6)
    arrowhead((1070,y),(1095,y),'muted',9)
    for x,color,label,n,check in [(115,'blue','Start','',False),
          (412,'amber','Not seen','1',False),(987,'green','Found','',True)]:
        flag(x,y,color,n,.62,check)
        text(x+37,y-9,label,24,color)
    circle(710,y,4,'white','purple',1.5)
    icon(680,y-49,35)
    text(725,y-9,'Replan',24,'purple')
    add('</g></svg>')
    return ''.join(c.S)


def render(svg,path,scale=1):
    handle=Rsvg.Handle.new_with_flags(Rsvg.HandleFlags(1));handle.write(svg.encode());handle.close()
    surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,round(W*scale),round(H*scale))
    ctx=cairo.Context(surface);ctx.scale(scale,scale)
    vp=Rsvg.Rectangle();vp.x=vp.y=0;vp.width=W;vp.height=H
    handle.render_document(ctx,vp);surface.write_to_png(str(path))


if __name__=='__main__':
    svg=build()
    render(svg,OUT/'fig8_official_hm3d_preview.png')
    render(svg,Path('/tmp/fig8_official_review.png'),.75)
    (OUT/'official_preview_notes.json').write_text(json.dumps(dict(
        preview_only=True,scene='00166-RaYrxWt5pR1',
        base_map='Official HM3D topdown_floors.png, unchanged raster, displayed at 0.68 opacity.',
        source=G['source_url'],source_page=G['source_page'],
        registration=G['registration'],
        route='Authored explanatory route, not algorithm output.',
        qwen='Illustrative candidate statement; no Qwen inference was run for this preview.',
        rgb='Native Habitat observations at q2 and q3; original pixels, viewport crops only.',
        qwen_icon_source='https://github.com/QwenLM/Qwen3/blob/main/README.md',
        labels=LABELS,english_word_count=sum(len(re.findall(r'[A-Za-z]+',s)) for s in LABELS)
    ),ensure_ascii=False,indent=2)+'\n')
    print('Preview ready:',OUT/'fig8_official_hm3d_preview.png')
