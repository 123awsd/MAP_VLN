#!/usr/bin/python3
"""Compact, continuous sequence study. Render PNG previews only.

Reference: BSC-Nav Fig. 3b, specifically its map / cumulative route / RGB
alignment. All scene data are from the project, not copied from the paper.
"""
from pathlib import Path
import json
import html
import math
import re
import cairo
import gi
gi.require_version('Rsvg', '2.0')
from gi.repository import Rsvg
import compose_preview as core

OUT=Path(__file__).resolve().parent
G=core.G
D=json.loads((OUT/'assets/sequence_geometry.json').read_text())
W,H=2360,920
core.S.clear(); core.LABELS.clear()
core.C.update(blue='#168CA5',amber='#BE7339',green='#2F8C58',
              ink='#212429',muted='#666A70',seen='#B2B9BE',line='#C8CDCF')
add=core.add;path=core.path;line=core.line;rect=core.rect
polygon=core.polygon;circle=core.circle;camera=core.camera
scene=core.scene;rgb=core.rgb;asset=core.asset;smooth=core.smooth
LABELS=[]

def text(x,y,s,size=27,c='ink',weight=400,anchor='start',italic=False):
    LABELS.append(dict(text=s,x=x,y=y,size=size))
    add(f'<text x="{x}" y="{y}" fill="{core.col(c)}" font-family="TeX Gyre Termes" '
        f'font-size="{size}" font-weight="{weight}" text-anchor="{anchor}" '
        f'font-style="{"italic" if italic else "normal"}">{html.escape(s)}</text>')

def badge(x,y,n,c,r=15):
    circle(x,y,r,'white',c,2.4)
    text(x,y+9,str(n),27,c,700,'middle')

def target_box(view,native,colour,width=2):
    x,y,w,h=native
    a=view.point((x/1600,y/1200));b=view.point(((x+w)/1600,(y+h)/1200))
    rect(*a,b[0]-a[0],b[1]-a[1],'none',colour,width)

def route(v,stop=4,main=False):
    keys=['approach','local','recovery','verify'][:stop]
    for i,key in enumerate(keys):
        # Earlier approach is a thin dashed line; actual recovery is continuous.
        width=(4.4 if i>=2 else 3.1) if main else 3.0
        color='blue' if i<3 else 'green'
        path(smooth(v.points(D['route'][key])),color,width,
             dash='7 5' if main and i==0 else None,opacity=.82 if i==0 else 1)

def mini_map(n,x,y,w,h):
    v=scene(f'mini-{n}',(.058,.09,.86,.80),(x,y,w,h))
    # Flat navigation mesh, preserving the full apartment and the same orientation.
    commands=[]
    for tri in D['triangles']:
        commands.append('M'+' L'.join(f'{a:.3f},{b:.3f}' for a,b in v.points(tri))+' Z')
    add('<path d="'+' '.join(commands)+'" fill="#62666A" stroke="#62666A" stroke-width=".4"/>')
    route(v,n)
    pose=['q1','q2','qpre','q3'][n-1]
    c=['amber','amber','blue','green'][n-1]
    p=v.point(D['poses'][pose])
    a=v.point(G['anchors']['armchair' if n<3 else 'cushion'])
    # A small white disc provides separation from the dark walkable-space mask.
    circle(*p,8,'white','none',0)
    camera(p,a,c,7.5)
    if n>=3:
        t=v.point(G['anchors']['cushion'])
        circle(*t,4,'green','white',.7)
    return v

def build():
    add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
    add('<title>Illustrative semantic recovery in an HM3D apartment</title><defs>')
    for key in ['blue','amber','green','muted','ink']:
        add(f'<marker id="arrow-{key}" viewBox="0 0 12 10" refX="10" refY="5" '
            f'markerWidth="12" markerHeight="10" markerUnits="userSpaceOnUse" orient="auto">'
            f'<path d="M0,0 L12,5 L0,10 Z" fill="{core.col(key)}"/></marker>')
    asset('apartment',OUT/'assets/apartment.png',2800,2800)
    for key,file in [('first','obs_000000.png'),('second','obs_000001.png'),
        ('found','obs_000003.png')]:
        asset(key,core.FRAMES/file,1600,1200)
    asset('earlier',OUT/'assets/recovery_observation.png',1600,1200)
    add('</defs><g stroke-linejoin="round" stroke-linecap="round">')
    rect(0,0,W,H,'white','none')
    text(22,46,'“Find a cushion; check the bedroom armchair first.”',32,italic=True)
    text(2338,45,'Illustrative sequence',25,'muted',anchor='end')

    # Full apartment with an explicit entry, local retry and return route.
    main=scene('main-v2',(.058,.09,.86,.80),(10,115,790,735))
    main.image()
    route(main,4,main=True)
    a=main.point(G['anchors']['armchair']);t=main.point(G['anchors']['cushion'])
    # Compact object boxes match the borders on the corresponding RGB observations.
    rect(a[0]-23,a[1]-32,46,67,'none','amber',2.2)
    rect(t[0]-15,t[1]-13,30,27,'none','green',2.2)
    start=main.point(D['poses']['start'])
    circle(*start,5,'blue','none',0)
    text(start[0]+12,start[1]+8,'Start',25,'ink')
    for n,key,c,offset in [(1,'q1','amber',(-40,18)),(2,'q2','amber',(40,-10)),
                            (3,'qpre','blue',(31,20)),(4,'q3','green',(-35,-19))]:
        p=main.point(D['poses'][key]);aim=a if n<3 else t
        camera(p,aim,c,10)
        b=(p[0]+offset[0],p[1]+offset[1])
        line([p,b],c=c,width=1.4)
        badge(*b,n,c)
        # Only the last two views need sight lines for identifying the cushion.
        if n>=3:line([p,t],c='green',width=1.2,dash='3 5',opacity=.75)
    # A few arrowheads on the route show motion without a heavy outlined curve.
    pp=main.points(D['route']['approach'])
    line([pp[4],pp[5]],c='blue',width=3.1,dash='7 5',arrow=True,opacity=.82)
    pp=main.points(D['route']['recovery'])
    line([pp[4],pp[5]],c='blue',width=4.4,arrow=True)

    # One continuous time axis. Each station owns its map, RGB and short status.
    xs=[850,1226,1602,1978];pw=350
    line([(856,103),(2331,103)],c='ink',width=2.2,arrow=True)
    for n,x,label,c in zip(range(1,5),xs,
                          ['Not found','Local limit','New location','Verified'],
                          ['amber','amber','blue','green']):
        text(x+pw/2,87,label,28,c,700,'middle')
        circle(x+pw/2,103,4.4,c,'white',1)
        mini_map(n,x+35,126,280,230)
        badge(x+25,140,n,c,13)

    pics=[]
    for i,(x,key,crop,c) in enumerate(zip(xs,
        ['first','second','earlier','found'],
        [(350,405,1100,794),(365,405,1100,794),(230,405,1100,794),(350,405,1100,794)],
        ['amber','amber','blue','green'])):
        v=rgb(f'rgb-v2-{i}',key,crop,(x,375,pw,253))
        rect(x,375,pw,253,'none',c,2)
        pics.append(v)
    target_box(pics[2],(738,654,132,153),'green')
    target_box(pics[3],(952.35,603.525,207.8,189.825),'green')
    # Two restrained enclosing boxes keep labels attached to their evidence.
    for x0,x1,label in [(850,1576,'Bedroom armchair'),(1602,2328,'Living-room sofa')]:
        rect(x0,652,x1-x0,236,'white','line',1.1)
        text((x0+x1)/2,686,label,27,'ink',700,'middle')

    # Local evidence is directly under its two observations. No separate section.
    for n,x in [(1,868),(2,1037)]:
        v=scene(f'cov-{n}',(.223,.252,.116,.151),(x,705,120,156))
        v.image(.70)
        core.coverage_cells(v,lambda c:c['first'],'seen',.60)
        if n==2:
            core.coverage_cells(v,lambda c:c['second'] and not c['first'],'amber',.95)
        rect(x,705,120,156,'none','line',1)
        badge(x+105,845,n,'amber',11)
    line([(998,785),(1024,785)],c='amber',width=1.7,arrow=True)
    rect(1202,723,15,15,'seen','none');text(1228,739,'Seen',25,'muted')
    rect(1343,723,15,15,'amber','none');text(1369,739,'New',25,'amber')
    text(1202,798,'New coverage lowers',28)
    text(1202,832,'search confidence.',28)
    # The same target observed in stations 3 and 4 gives a single evidence flow.
    # All supporting RGB crops already appear above; no repeated photographs.
    line([(1777,710),(1777,750),(2066,750)],c='green',width=1.5)
    line([(2153,710),(2153,750),(2066,750)],c='green',width=1.5)
    text(1965,733,'Consistent observations',27,'green',anchor='middle')
    text(2027,793,'After execution',25,'muted',anchor='end')
    line([(2066,750),(2066,817)],c='green',width=1.5,arrow=True)
    rect(1760,825,111,47,'white','line',1.3)
    text(1815.5,857,'Sofa',27,anchor='middle')
    line([(1871,848),(1978,848)],c='green',width=1.8)
    rect(1978,825,176,47,'#F0F7F2','green',1.5)
    text(2066,857,'+ Cushion',28,'green',anchor='middle')
    # Remaining same-task and novelty qualifications live in the caption draft.
    add('</g></svg>')
    return ''.join(core.S)

def render(svg,filename,scale=1):
    h=Rsvg.Handle.new_with_flags(Rsvg.HandleFlags(1));h.write(svg.encode());h.close()
    surf=cairo.ImageSurface(cairo.FORMAT_ARGB32,int(W*scale),int(H*scale))
    ctx=cairo.Context(surf);ctx.scale(scale,scale)
    viewport=Rsvg.Rectangle();viewport.x=viewport.y=0;viewport.width=W;viewport.height=H
    h.render_document(ctx,viewport);surf.write_to_png(str(filename))

if __name__=='__main__':
    svg=build()
    render(svg,OUT/'fig8_sequence_preview.png')
    render(svg,Path('/tmp/fig8_sequence_small.png'),.55)
    words=sum(len(re.findall(r'\b[A-Za-z]+\b',t['text'])) for t in LABELS)
    (OUT/'sequence_notes.json').write_text(json.dumps(dict(
        preview_only=True,reference='https://www.nature.com/articles/s41467-026-74358-5/figures/3',
        english_word_count=words,labels=LABELS,
        scene_and_rgb='Existing HM3D scene and native Habitat camera images; RGB pixels are not retouched. Station 3 is rendered at an authored pose, recorded in assets/recovery_observation.json.',
        route='Illustrative, navmesh-guided; not output of the paper planner.',
        coverage=G['coverage_note'],coverage_limit=G['source_mesh_cut'],
        assumptions=['Local limit reached after the second illustrated view.',
            'The cushion is initially unmapped in this mechanism example.',
            'Positive verification resolves the same task online; novel-instance insertion occurs after execution.'],
        removed=['Four section titles','Repeated RGB target crops','Redundant route legend',
                 'Repeated task-state descriptions','Thick white path outline']),ensure_ascii=False,indent=2)+'\n')
    print('Preview ready; English words:',words)
