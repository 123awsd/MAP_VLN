#!/usr/bin/python3
"""Compose the Blender scene and vector annotations into an editable figure.

Run with /usr/bin/python3 (Cairo and librsvg).
All primary labels remain native text; only the 3D scene layers are raster.
"""
from pathlib import Path
import base64
import html
import json
import math
import cairo
import gi
gi.require_version('Rsvg', '2.0')
from gi.repository import Rsvg

OUT = Path(__file__).resolve().parent
W, H = 1600, 900
WIDTH_MM = 184
INK = '#263942'
MUTED = '#6F7C84'
GRAY = '#9AA7AF'
LIGHT = '#DCE2E5'
BLUE = '#287A91'
ORANGE = '#C5713F'
BG = '#FFFFFF'
pieces = []
geometry = json.loads((OUT/'scene_geometry.json').read_text())


def add(s):
    pieces.append(s)


def text(x, y, value, size=24, color=INK, anchor='start', bold=False,
         italic=False, halo=False):
    style = ('font-weight:600;' if bold else '')+('font-style:italic;' if italic else '')
    if halo:
        style += 'paint-order:stroke;stroke:white;stroke-width:5;stroke-linejoin:round;'
    add(f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" fill="{color}" '
        f'text-anchor="{anchor}" style="{style}">{html.escape(value)}</text>')


def line(points, color=GRAY, width=1.6, dash=None):
    p = ' '.join(f'{x:.2f},{y:.2f}' for x,y in points)
    extra = f' stroke-dasharray="{dash}"' if dash else ''
    add(f'<polyline points="{p}" fill="none" stroke="{color}" stroke-width="{width}"{extra}/>')


def path(d, color=GRAY, width=1.6, arrow=False, dash=None):
    extra = f' stroke-dasharray="{dash}"' if dash else ''
    marker = f' marker-end="url(#arrow-{color[1:]})"' if arrow else ''
    add(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{marker}{extra}/>')


def rect(x,y,w,h,fill='none',edge=LIGHT,width=1.5):
    add(f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" '
        f'fill="{fill}" stroke="{edge}" stroke-width="{width}"/>')


def circle(x,y,r,fill='white',edge=GRAY,width=1.6):
    add(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r:.2f}" fill="{fill}" '
        f'stroke="{edge}" stroke-width="{width}"/>')


def polygon(points,fill,edge='none',width=1.4):
    p=' '.join(f'{x:.2f},{y:.2f}' for x,y in points)
    add(f'<polygon points="{p}" fill="{fill}" stroke="{edge}" stroke-width="{width}"/>')


def image(name,x,y,w,h):
    b64=base64.b64encode((OUT/name).read_bytes()).decode()
    add(f'<image x="{x}" y="{y}" width="{w}" height="{h}" '
        f'xlink:href="data:image/png;base64,{b64}"/>')


IMG = (-14, 50, 1090, 1090*geometry['render_size'][1]/geometry['render_size'][0])


def point(name):
    uv=geometry['main_points'][name]
    return (IMG[0]+IMG[2]*uv[0], IMG[1]+IMG[3]*uv[1])


def connect(start,end,bend=28,color=GRAY,width=2):
    ax,ay=start;bx,by=end
    dx=bx-ax;dy=by-ay;length=math.hypot(dx,dy)
    ux,uy=dx/length,dy/length
    ax+=ux*11;ay+=uy*11;bx-=ux*13;by-=uy*13
    nx,ny=-uy,ux
    path(f'M {ax:.2f} {ay:.2f} C {ax+dx*.32+nx*bend:.2f} {ay+dy*.32+ny*bend:.2f}, '
         f'{ax+dx*.68+nx*bend:.2f} {ay+dy*.68+ny*bend:.2f}, {bx:.2f} {by:.2f}',
         color,width,True)


def sample_region(x,y,covered,new=False):
    # A shared projected grid makes the set update visually comparable.
    u=(126.,36.);v=(55.,-64.)
    def p(a,b):return (x+a*u[0]+b*v[0],y+a*u[1]+b*v[1])
    polygon([p(0,0),p(1,0),p(1,1),p(0,1)],'#F8F9F9',GRAY)
    polygon([p(0,0),p(2/6,0),p(2/6,1),p(0,1)],'#E5E9EB')
    if new:
        polygon([p(2/6,0),p(4/6,0),p(4/6,1),p(2/6,1)],'#BDD8DE')
    for a in range(1,6):line([p(a/6,0),p(a/6,1)],LIGHT,1)
    for b in range(1,4):line([p(0,b/4),p(1,b/4)],LIGHT,1)
    for a in range(6):
        for b in range(4):
            q=p((a+.5)/6,(b+.5)/4)
            circle(*q,2.8,'white' if a<covered else MUTED,
                   BLUE if new and 2<=a<4 else GRAY,1)
    polygon([p(0,0),p(1,0),p(1,1),p(0,1)],'none',GRAY)


def cube(x,y,s=22,color=ORANGE):
    front=[(x,y),(x+s,y),(x+s,y+s),(x,y+s)]
    off=(10,-8)
    back=[(a+off[0],b+off[1]) for a,b in front]
    polygon([front[0],front[1],back[1],back[0]],'#FAF3EE',color,1.5)
    polygon([front[1],front[2],back[2],back[1]],'#F0DCCD',color,1.5)
    polygon(front,'#FCF8F4',color,1.5)


def build():
    add(f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" '
        f'width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
    add('<title>Observation-driven semantic recovery and persistent map updates</title>')
    add('<desc>Blender-rendered schematic geometry with vector explanations; not an experimental recording.</desc>')
    add('<defs>')
    for c in [INK,MUTED,GRAY,BLUE,ORANGE]:
        add(f'<marker id="arrow-{c[1:]}" viewBox="0 0 8 8" refX="7" refY="4" '
            f'markerWidth="7" markerHeight="7" orient="auto" markerUnits="userSpaceOnUse">'
            f'<path d="M 0 0 L 8 4 L 0 8 Z" fill="{c}"/></marker>')
    add('</defs>')
    rect(0,0,W,H,'white','none')
    add('<g font-family="Liberation Sans,Arial,sans-serif" stroke-linejoin="round" stroke-linecap="round">')
    image('scene.png',*IMG)
    # A narrow separator groups the explanatory details without colored cards.
    line([(1100,16),(1100,866)],LIGHT,1.4)
    line([(1130,406),(1575,406)],LIGHT,1.4)
    text(18,36,'(a)  Recovery within one task',28,bold=True)
    text(1130,36,'(b)  Coverage feedback',28,bold=True)
    text(1130,446,'(c)  Evidence → map update',28,bold=True)

    # Labels sit outside the spatial evidence and point to the relevant marks.
    text(40,153,'Local retries',27,bold=True)
    text(40,184,'Miss → another view',23,MUTED)
    q1,q2,q3,q4,q5=[point(k) for k in ('q1','q2','q3','q4','q5')]
    line([(173,202),(173,265),(q1[0]-12,q1[1]-15)],GRAY,1.5)
    text(428,244,'Current region',23,MUTED,halo=True)
    old=point('old_anchor')
    line([(439,254),(413,276),(old[0]+24,old[1]-16)],GRAY,1.3)
    connect(q1,q2,20,GRAY,2)
    connect(q2,q3,31,GRAY,2)
    connect(q3,q4,30,BLUE,2.7)
    for i,q in enumerate([q1,q2,q3,q4,q5],1):
        text(q[0]+(17 if i in (1,4) else 0),q[1]+29,
             ['q₁','q₂','q₃','q*','q′'][i-1],22,
             BLUE if i==4 else MUTED,anchor='middle',italic=True,halo=True)

    target=point('target')
    text(1045,322,'Target verified',27,ORANGE,anchor='end',bold=True)
    text(1045,353,'Resolve task online',23,MUTED,anchor='end')
    line([(927,370),(882,398),(target[0]+11,target[1]-5)],ORANGE,1.7)
    circle(target[0]+11,target[1]-5,3,ORANGE,ORANGE,0)
    text(786,715,'Alternative region',23,BLUE,halo=True)
    text(58,736,'Region recovery',27,bold=True)
    text(58,767,'After exhaustion / local limit',23,MUTED)
    text(58,798,'Infer + ground → replan',23,MUTED)
    line([(404,721),(451,657),(499,565)],GRAY,1.4)

    # Legend stays outside the scene; it describes roles, not fictitious measurements.
    line([(28,858),(57,858)],GRAY,2)
    text(68,866,'Other views',22,MUTED)
    line([(240,858),(270,858)],BLUE,2.5)
    text(281,866,'Selected view',22,MUTED)
    rect(489,850,17,17,'none',ORANGE,1.8)
    text(518,866,'Confirmed instance',22,MUTED)
    text(1065,866,'Arrows: search order',22,MUTED,anchor='end')

    # New coverage, rather than repeated misses, changes the regional belief.
    text(1130,78,'Negative observations',23,MUTED)
    text(1237,125,'Before',23,anchor='middle')
    text(1472,125,'After a miss',23,anchor='middle')
    sample_region(1145,224,2)
    sample_region(1380,224,4,True)
    path('M 1336 213 L 1367 213',MUTED,1.8,True)
    text(1237,297,'Uᵣ',24,anchor='middle',italic=True)
    text(1472,297,'Uᵣ ∪ V(q)',24,anchor='middle',italic=True)
    rect(1396,318,15,15,'#BDD8DE','none')
    text(1421,332,'New coverage',21,BLUE)
    text(1130,366,'New coverage lowers regional belief.',22)
    text(1130,393,'Repeated coverage adds no penalty.',21,MUTED)

    # The projected detection boxes are derived from the same 3D target box.
    text(1130,479,'Illustrative views of the same instance',22,MUTED)
    for key,x in [('q5',1130),('q4',1367)]:
        y=499;w=206;h=w*.7
        image(key+'_observation.png',x,y,w,h)
        rect(x,y,w,h,'none',LIGHT,1.3)
        b=geometry[key+'_bbox']
        rect(x+b[0]*w,y+b[1]*h,(b[2]-b[0])*w,(b[3]-b[1])*h,
             'none',ORANGE,1.9)
        text(x+12,y+25,'q*' if key=='q4' else 'q′',21,MUTED,italic=True,halo=True)
    path('M 1230 651 L 1280 676 L 1327 676',MUTED,1.5,True)
    path('M 1470 651 L 1422 676 L 1382 676',MUTED,1.5,True)
    cube(1338,662,23)
    text(1351,719,'Fuse consistent views',23,anchor='middle')
    line([(1130,741),(1575,741)],LIGHT,1.3,dash='4 6')
    text(1130,774,'After execution',22,MUTED)
    root=(1435,776)
    for q in [(1355,830),(1435,830),(1515,830)]:
        new=q[0]>1500
        line([root,q],ORANGE if new else GRAY,1.8)
        circle(*q,7,ORANGE if new else 'white',ORANGE if new else GRAY,1.8)
    circle(*root,7,'white',MUTED,1.8)
    text(1435,764,'Room',21,MUTED,anchor='middle')
    text(1555,863,'Insert new instance',22,ORANGE,anchor='end')
    add('</g></svg>')
    return ''.join(pieces).encode()


def render(svg):
    (OUT/'fig8_spatial_en.svg').write_bytes(svg)
    handle=Rsvg.Handle.new_from_data(svg)
    viewport=Rsvg.Rectangle()
    viewport.x=viewport.y=0
    viewport.width=W;viewport.height=H
    scale=WIDTH_MM/25.4*72/W
    surface=cairo.PDFSurface(str(OUT/'fig8_spatial_en.pdf'),W*scale,H*scale)
    surface.set_metadata(cairo.PDF_METADATA_TITLE,'Fig. 8 — Observation-driven recovery')
    ctx=cairo.Context(surface);ctx.scale(scale,scale)
    handle.render_document(ctx,viewport);surface.finish()
    for name,render_scale in [('fig8_spatial_en_preview.png',1.2),('fig8_spatial_en.png',WIDTH_MM/25.4*600/W)]:
        surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,math.ceil(W*render_scale),math.ceil(H*render_scale))
        ctx=cairo.Context(surface);ctx.scale(render_scale,render_scale)
        handle.render_document(ctx,viewport)
        surface.write_to_png(str(OUT/name));surface.finish()
    print(OUT/'fig8_spatial_en_preview.png')


if __name__=='__main__':
    render(build())
