#!/usr/bin/python3
"""Fig. 8: spatial evidence, same-task recovery, and explicit map change.

Native SVG crops and vector overlays; no generative alteration of source RGB.
All coverage polygons and the memory state are explanatory constructions.
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
ROOT = OUT.parents[2]
FRAMES = ROOT/'画图/Habitat长指令执行/keyframes/00166_cushion_recovery_20261024'
G = json.loads((OUT/'assets/projection.json').read_text())
W,H = 2400,1320
C = dict(ink='#273744',muted='#6D7882',line='#DBE1E6',gray='#9BA8B4',
         seen='#8493A3',amber='#C58B4E',blue='#4776A3',green='#288571')
S,TEXT,CROPS = [],[],[]
MEASURE = cairo.Context(cairo.ImageSurface(cairo.FORMAT_ARGB32,1,1))
IDS = 0

def add(s): S.append(s)
def color(c): return C.get(c,c)

def text(x,y,s,size=36,c='ink',bold=False,anchor='start',halo=False):
    assert size >= 34
    MEASURE.select_font_face('Liberation Sans',cairo.FONT_SLANT_NORMAL,
        cairo.FONT_WEIGHT_BOLD if bold else cairo.FONT_WEIGHT_NORMAL)
    MEASURE.set_font_size(size)
    width = MEASURE.text_extents(s).x_advance
    left = x if anchor=='start' else x-width if anchor=='end' else x-width/2
    assert left >= 0 and left+width <= W, (s,left,width)
    TEXT.append(dict(text=s,left=left,right=left+width,top=y-size,bottom=y+size*.2,size_px=size))
    stroke = ' paint-order="stroke" stroke="white" stroke-width="4"' if halo else ''
    add(f'<text x="{x}" y="{y}" fill="{color(c)}" font-family="Liberation Sans" '
        f'font-size="{size}" font-weight="{700 if bold else 400}" text-anchor="{anchor}"'
        f'{stroke}>{html.escape(s)}</text>')

def path(d,c='gray',width=2,arrow=False,dash=None,opacity=1):
    marker = f' marker-end="url(#arrow-{c})"' if arrow else ''
    ds = f' stroke-dasharray="{dash}"' if dash else ''
    add(f'<path d="{d}" stroke="{color(c)}" stroke-width="{width}" fill="none" '
        f'opacity="{opacity}"{marker}{ds}/>')

def line(pts,**kw): path('M'+' L'.join(f'{x:.3f},{y:.3f}' for x,y in pts),**kw)

def rect(x,y,w,h,fill='none',stroke='line',width=1.5,rx=0,opacity=1):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{color(fill)}" '
        f'stroke="{color(stroke)}" stroke-width="{width}" rx="{rx}" opacity="{opacity}"/>')

def circle(x,y,r,fill='white',stroke='gray',width=2):
    add(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{color(fill)}" '
        f'stroke="{color(stroke)}" stroke-width="{width}"/>')

def polygon(pts,fill='none',stroke='none',opacity=1,width=2,dash=None):
    ds = f' stroke-dasharray="{dash}"' if dash else ''
    add('<polygon points="'+' '.join(f'{x:.2f},{y:.2f}' for x,y in pts)+'" '
        f'fill="{color(fill)}" fill-opacity="{opacity}" stroke="{color(stroke)}" '
        f'stroke-width="{width}"{ds}/>')

def asset(name,p,w,h):
    data = base64.b64encode(Path(p).read_bytes()).decode()
    add(f'<image id="{name}" width="{w}" height="{h}" href="data:image/png;base64,{data}"/>')

class View:
    def __init__(self,name,asset_name,dimensions,crop,box):
        self.name,self.asset = name,asset_name
        self.iw,self.ih = dimensions
        self.crop,self.box = crop,box
        cx,cy,cw,ch = crop
        x,y,w,h = box
        self.scale = min(w/cw,h/ch)
        self.ox = x+(w-cw*self.scale)/2-cx*self.scale
        self.oy = y+(h-ch*self.scale)/2-cy*self.scale
        add(f'<defs><clipPath id="clip-{name}"><rect x="{x}" y="{y}" width="{w}" height="{h}"/></clipPath></defs>')
    def point(self,uv):
        return self.ox+uv[0]*self.iw*self.scale,self.oy+uv[1]*self.ih*self.scale
    def points(self,uvs): return [self.point(p) for p in uvs]
    def image(self,opacity=1):
        add(f'<g clip-path="url(#clip-{self.name})"><use href="#{self.asset}" '
            f'transform="translate({self.ox},{self.oy}) scale({self.scale})" opacity="{opacity}"/></g>')
    def focus(self,background=.25):
        self.image(background)
        pieces=[]
        for key in ['active_bedroom','active_living']:
            pieces.append('<polygon points="'+' '.join(f'{x},{y}' for x,y in self.points(G[key]))+'"/>')
        add(f'<defs><clipPath id="active-{self.name}">'+''.join(pieces)+'</clipPath></defs>')
        add(f'<g clip-path="url(#active-{self.name})">')
        self.image(.92)
        add('</g>')

def scene_view(name,crop_uv,box):
    u,v,du,dv = crop_uv
    return View(name,'apartment',(3200,2800),(3200*u,2800*v,3200*du,2800*dv),box)

def rgb(name,asset_name,crop,box):
    CROPS.append(dict(asset=asset_name,crop_xywh=crop))
    v = View(name,asset_name,(1600,1200),crop,box)
    v.image()
    rect(*box)
    return v

def smooth(points):
    d=f'M{points[0][0]},{points[0][1]}'
    for i in range(len(points)-1):
        p0,p1,p2,p3=points[max(i-1,0)],points[i],points[i+1],points[min(i+2,len(points)-1)]
        a=(p1[0]+.12*(p2[0]-p0[0]),p1[1]+.12*(p2[1]-p0[1]))
        b=(p2[0]-.12*(p3[0]-p1[0]),p2[1]-.12*(p3[1]-p1[1]))
        d+=f' C{a[0]},{a[1]} {b[0]},{b[1]} {p2[0]},{p2[1]}'
    return d

def camera(p,aim,c='blue',size=15):
    dx,dy=aim[0]-p[0],aim[1]-p[1]
    length=math.hypot(dx,dy);dx/=length;dy/=length
    polygon([(p[0]+dx*size,p[1]+dy*size),
             (p[0]-dx*size-dy*size*.7,p[1]-dy*size+dx*size*.7),
             (p[0]-dx*size+dy*size*.7,p[1]-dy*size-dx*size*.7)],c,'white',1,2.5)

def header(x,label,c):
    text(x,145,label,40,c,bold=True)

def node(x,y,w,label,c='gray',new=False):
    rect(x,y,w,62,'#EAF5F0' if new else 'white',c,2)
    text(x+w/2,y+43,label,35,'green' if new else 'ink',new,'middle')

def graph(x,y,new=False):
    # Identical relative coordinates, without drawing unrelated scene-graph nodes.
    node(x-121,y,242,'Living room')
    line([(x,y+62),(x,y+112)],width=2)
    node(x-80,y+112,160,'Sofa')
    if new:
        line([(x,y+174),(x,y+224)],c='green',width=3,arrow=True)
        node(x-105,y+224,210,'+ Cushion','green',True)

def build():
    add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
    add('<title>Coverage-aware recovery and observation-driven map update</title>')
    add('<desc>Illustrative local coverage in two matched spatial views, recovery to '
        'another mapped room within the same task, and before/after insertion of a '
        'confirmed novel cushion instance. Real HM3D mesh and RGB views; route, '
        'coverage partitions, and initially unmapped target are illustrative.</desc>')
    add('<defs>')
    for key,value in C.items():
        add(f'<marker id="arrow-{key}" markerWidth="20" markerHeight="20" refX="17" refY="8" '
            f'viewBox="0 0 18 16" orient="auto" markerUnits="userSpaceOnUse">'
            f'<path d="M0,0 L18,8 L0,16 Z" fill="{value}"/></marker>')
    asset('apartment',OUT/'assets/apartment.png',3200,2800)
    asset('miss',FRAMES/'obs_000000.png',1600,1200)
    asset('earlier',FRAMES/'frame_000420.png',1600,1200)
    asset('verified',FRAMES/'obs_000003.png',1600,1200)
    add('</defs><g stroke-linecap="round" stroke-linejoin="round">')
    rect(0,0,W,H,'white','none')
    text(32,55,'Find a cushion; check the bedroom armchair first.',39,bold=True)
    text(2368,55,'Mechanism illustration',34,'muted',anchor='end')
    line([(32,82),(2368,82)],c='line')
    header(32,'1  Local retry','ink')
    header(800,'2  Semantic recovery','ink')
    header(1830,'3  Verify & update','ink')
    line([(640,132),(754,132)],c='blue',width=2.5,arrow=True)
    line([(1657,132),(1784,132)],c='green',width=2.5,arrow=True)
    text(32,202,'Two views of the same search region',35,'muted')
    text(800,202,'Same task · another mapped room',35,'muted')
    text(1830,202,'Consistent RGB-D observations',35,'muted')
    line([(760,234),(760,1272)],c='line',width=1)
    line([(1798,234),(1798,1272)],c='line',width=1)

    # Local state change: same camera, same crop, same reference region.
    views=[]
    for n,x in [(1,32),(2,412)]:
        text(x+160,267,f'View {n}',36,bold=True,anchor='middle')
        v=scene_view(f'local-{n}',G['local_roi'],(x,290,320,400))
        v.image(.72)
        polygon(v.points(G['covered']),'seen','seen',.36,2)
        if n==2:
            polygon(v.points(G['newly_covered']),'amber','amber',.44,2.5)
        polygon(v.points(G['search_region']),'none','gray',1,2,'7 5')
        a=v.point(G['anchors']['armchair'])
        p=v.point(G['floor_poses'][f'q{n}'])
        # Schematic observation direction; the patch records cumulative coverage.
        line([p,a],c='seen' if n==1 else 'amber',width=2,dash='5 5')
        if n==2:
            q=v.point(G['floor_poses']['q1'])
            camera(q,a,'gray',11)
            path(f'M{q[0]},{q[1]+14} Q{q[0]+75},{q[1]+15} {p[0]+10},{p[1]+26}',
                 c='amber',width=2.7,arrow=True)
        camera(p,a,'seen' if n==1 else 'amber',16)
        rect(x,290,320,400)
        text(x+160,733,'Miss' if n==1 else 'Miss · local limit',35,'muted',anchor='middle')
        views.append(v)
    line([(365,490),(398,490)],c='amber',width=2.8,arrow=True)
    rect(75,784,28,23,'seen','none')
    text(118,810,'Already seen',34,'muted')
    rect(397,784,28,23,'amber','none')
    text(440,810,'New coverage',34,'amber')
    text(382,866,'Only new coverage lowers belief',35,'ink',True,'middle')
    rgb('miss-rgb','miss',(420,500,1080,700),(162,926,440,285.185))
    text(382,1260,'Armchair retained · target missing',34,'muted',anchor='middle')

    # Recovery focuses on the relevant rooms; complete apartment remains as context.
    main=scene_view('main',G['focus_roi'],(790,247,988,742))
    main.focus()
    polygon(main.points(G['search_region']),'amber','amber',.18,2.5)
    local=smooth(main.points(G['route_local']))
    recovery=smooth(main.points(G['route_recovery']))
    for d,c in [(local,'amber'),(recovery,'blue')]:
        path(d,'white',11)
        path(d,c,5.5,arrow=True)
    a=main.point(G['anchors']['armchair']);t=main.point(G['anchors']['cushion'])
    p=main.point(G['poses']['q3'])
    line([p,t],c='green',width=2.5,dash='6 5')
    camera(main.point(G['poses']['q2']),a,'amber',12)
    camera(p,t,'green',14)
    circle(*t,19,'white','green',3)
    path(f'M{t[0]-9},{t[1]} l6,7 l13,-17','green',3)
    text(a[0]-31,a[1]-27,'Armchair',35,'amber',True,'end',True)
    text(t[0]-31,t[1]+53,'Cushion',35,'green',True,'end',True)
    room=main.point(G['anchors']['bedroom'])
    text(room[0]-5,room[1]+115,'Bedroom',36,'ink',False,'middle',True)
    room=main.point(G['anchors']['living_room'])
    text(room[0]+42,room[1]+58,'Living room',36,'ink',False,'middle',True)
    # Recover is tied to the actual doorway rather than a detached process arrow.
    pts=main.points(G['route_recovery'])
    text(pts[5][0]-5,pts[5][1]+65,'Recover',35,'blue',True,'middle',True)

    context=scene_view('context',(0,0,1,1),(1045,1020,440,250))
    context.focus(background=.48)
    u,v,du,dv=G['focus_roi']
    box=context.points([(u,v),(u+du,v),(u+du,v+dv),(u,v+dv)])
    polygon(box,'none','blue',1,1.7)
    text(1265,1303,'Apartment context',34,'muted',anchor='middle')

    # The goal position leads directly to two observations of the same cushion.
    rgb('earlier-rgb','earlier',(955,632,300,300),(1838,267,240,240))
    rgb('verified-rgb','verified',(926,566,300,300),(2126,267,240,240))
    line([(t[0]+25,t[1]+5),(1783,t[1]+5),(1783,387),(1825,387)],
         c='green',width=1.8,arrow=True)
    text(2102,563,'Verified · task resolved',35,'green',True,'middle')
    line([(1958,587),(1958,615),(2102,615)],c='green',width=2)
    line([(2246,587),(2246,615),(2102,615),(2102,641)],c='green',width=2,arrow=True)
    text(2102,685,'Confirmed 3D track',35,'green',anchor='middle')
    line([(1830,729),(2370,729)],c='line',width=1.5,dash='6 5')
    text(2102,787,'AFTER EXECUTION',34,'muted',anchor='middle')
    # Before and after share the same view and identical existing node positions.
    text(1958,856,'Before',36,bold=True,anchor='middle')
    text(2246,856,'After',36,'green',True,'middle')
    graph(1958,901)
    graph(2246,901,True)
    line([(2087,985),(2116,985)],c='green',width=2.5,arrow=True)
    text(2102,1260,'Existing nodes retained',34,'muted',anchor='middle')
    add('</g></svg>')
    svg='\n'.join(S)
    name='fig8_mechanism_en'
    (OUT/f'{name}.svg').write_text(svg)
    handle=Rsvg.Handle.new_with_flags(Rsvg.HandleFlags(1))
    handle.write(svg.encode());handle.close()
    scale=184/25.4*72/W
    pdf=cairo.PDFSurface(str(OUT/f'{name}.pdf'),W*scale,H*scale)
    ctx=cairo.Context(pdf);ctx.scale(scale,scale);handle.render_cairo(ctx);pdf.finish()
    preview=cairo.ImageSurface(cairo.FORMAT_ARGB32,W,H)
    handle.render_cairo(cairo.Context(preview));preview.write_to_png(str(OUT/f'{name}.png'))
    (OUT/'layout.json').write_text(json.dumps(dict(text=TEXT,crops=CROPS,
        width_px=W,height_px=H,width_mm=184,height_mm=184*H/W,
        minimum_font_pt=min(t['size_px'] for t in TEXT)*scale),indent=2)+'\n')
    print(name,'exported:',len(TEXT),'labels')

if __name__=='__main__':
    build()
