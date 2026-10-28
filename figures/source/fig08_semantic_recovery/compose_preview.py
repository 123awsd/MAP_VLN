#!/usr/bin/python3
"""Reference-led Fig. 8 layout study. PNG preview only; no manuscript export.

The scene and RGB images come from the project's HM3D assets. The route,
search sequence, initial memory state and insertion are illustrative. The
coverage slice is ray tested against the display mesh, not planner evidence.
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
W, H = 2320, 1180
C = dict(ink='#25303A', muted='#6C747B', line='#D8DEE2', blue='#356E91',
         amber='#B47B38', green='#25806B', seen='#9AA9B5', pale='#EFF5F2')
S, LABELS = [], []

def add(s): S.append(s)
def col(c): return C.get(c,c)
def text(x,y,s,size=27,c='ink',weight=400,anchor='start'):
    LABELS.append(dict(text=s,x=x,y=y,size=size))
    add(f'<text x="{x}" y="{y}" fill="{col(c)}" font-family="TeX Gyre Heros" '
        f'font-size="{size}" font-weight="{weight}" text-anchor="{anchor}">{html.escape(s)}</text>')

def rect(x,y,w,h,fill='none',stroke='line',width=1,rx=0):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{col(fill)}" '
        f'stroke="{col(stroke)}" stroke-width="{width}" rx="{rx}"/>')

def path(d,c='muted',width=2,dash=None,arrow=False,opacity=1):
    ds=f' stroke-dasharray="{dash}"' if dash else ''
    marker=f' marker-end="url(#arrow-{c})"' if arrow else ''
    add(f'<path d="{d}" fill="none" stroke="{col(c)}" stroke-width="{width}" '
        f'opacity="{opacity}"{ds}{marker}/>')

def line(points,**kw): path('M'+' L'.join(f'{x:.2f},{y:.2f}' for x,y in points),**kw)
def polygon(points,fill='none',stroke='none',opacity=1,width=1):
    add('<polygon points="'+' '.join(f'{x:.2f},{y:.2f}' for x,y in points)+'" '
        f'fill="{col(fill)}" fill-opacity="{opacity}" stroke="{col(stroke)}" stroke-width="{width}"/>')
def circle(x,y,r,fill='white',stroke='blue',width=2):
    add(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{col(fill)}" stroke="{col(stroke)}" stroke-width="{width}"/>')
def number(x,y,n,c='blue',r=18):
    circle(x,y,r,c,'white',2.5)
    text(x,y+9,str(n),25,'white',700,'middle')

def asset(name,path,w,h):
    data=base64.b64encode(path.read_bytes()).decode()
    add(f'<image id="{name}" width="{w}" height="{h}" href="data:image/png;base64,{data}"/>')

class View:
    def __init__(self,name,asset,iw,ih,crop,box):
        self.name,self.asset,self.iw,self.ih=name,asset,iw,ih
        cx,cy,cw,ch=crop
        x,y,w,h=box
        self.scale=min(w/cw,h/ch)
        self.ox=x+(w-cw*self.scale)/2-cx*self.scale
        self.oy=y+(h-ch*self.scale)/2-cy*self.scale
        add(f'<defs><clipPath id="clip-{name}"><rect x="{x}" y="{y}" width="{w}" height="{h}"/></clipPath></defs>')
    def point(self,p): return self.ox+p[0]*self.iw*self.scale,self.oy+p[1]*self.ih*self.scale
    def points(self,ps): return [self.point(p) for p in ps]
    def image(self,opacity=1):
        add(f'<g clip-path="url(#clip-{self.name})"><use href="#{self.asset}" '
            f'transform="translate({self.ox},{self.oy}) scale({self.scale})" opacity="{opacity}"/></g>')

def scene(name,crop,box):
    return View(name,'apartment',2800,2800,[v*2800 for v in crop],box)

def rgb(name,asset,crop,box):
    v=View(name,asset,1600,1200,crop,box)
    v.image()
    rect(*box,stroke='line',width=1.2)
    return v

def smooth(points):
    d=f'M{points[0][0]},{points[0][1]}'
    for i in range(len(points)-1):
        p0,p1,p2,p3=points[max(i-1,0)],points[i],points[i+1],points[min(i+2,len(points)-1)]
        a=(p1[0]+.12*(p2[0]-p0[0]),p1[1]+.12*(p2[1]-p0[1]))
        b=(p2[0]-.12*(p3[0]-p1[0]),p2[1]-.12*(p3[1]-p1[1]))
        d+=f' C{a[0]},{a[1]} {b[0]},{b[1]} {p2[0]},{p2[1]}'
    return d

def camera(p,aim,c='blue',size=12):
    dx,dy=aim[0]-p[0],aim[1]-p[1]
    ln=math.hypot(dx,dy);dx/=ln;dy/=ln
    polygon([(p[0]+dx*size,p[1]+dy*size),
        (p[0]-dx*size-dy*size*.7,p[1]-dy*size+dx*size*.7),
        (p[0]-dx*size+dy*size*.7,p[1]-dy*size-dx*size*.7)],c,'white',1,1.8)

def bracket(x0,x1,y,label,c='muted'):
    path(f'M{x0},{y-7} v7 H{x1} v-7',c,1.3)
    text((x0+x1)/2,y+31,label,24,c,anchor='middle')

def coverage_cells(v,predicate,fill,opacity):
    add(f'<g fill="{col(fill)}" opacity="{opacity}" stroke="none">')
    # A single compound path avoids opacity accumulating at shared cell borders.
    d=[]
    for cell in G['cells']:
        if predicate(cell):
            d.append('M'+' L'.join(f'{x:.3f},{y:.3f}' for x,y in v.points(cell['polygon']))+' Z')
    add(f'<path d="{" ".join(d)}"/>')
    add('</g>')

def build():
    add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
    add('<title>Semantic recovery and map update: illustrative HM3D example</title>')
    add('<defs>')
    for key in ['blue','amber','green','muted','ink']:
        add(f'<marker id="arrow-{key}" viewBox="0 0 12 10" refX="10" refY="5" '
            f'markerWidth="12" markerHeight="10" markerUnits="userSpaceOnUse" orient="auto">'
            f'<path d="M0,0 L12,5 L0,10 Z" fill="{col(key)}"/></marker>')
    asset('apartment',OUT/'assets/apartment.png',2800,2800)
    for key,name in [('first','obs_000000.png'),('second','obs_000001.png'),
                     ('found','obs_000003.png'),('earlier','frame_000420.png')]:
        asset(key,FRAMES/name,1600,1200)
    add('</defs><g stroke-linecap="round" stroke-linejoin="round">')
    rect(0,0,W,H,'white','none')
    text(28,51,'Find a cushion. Check the bedroom armchair first.',32)
    text(2292,51,'Illustrative search sequence',24,'muted',anchor='end')
    line([(28,77),(2292,77)],c='line',width=1)
    text(28,125,'a',31,weight=700)
    text(65,125,'Search and recovery',29)
    text(1080,125,'b',31,weight=700)
    text(1118,125,'Observations along the route',29)

    main=scene('main',(.058,.09,.86,.80),(18,159,1000,930))
    main.image()
    # Dashed search window is tied to the coverage detail, not a synthetic room.
    roi=(.226,.252,.107,.151)
    points=main.points([(roi[0],roi[1]),(roi[0]+roi[2],roi[1]),
                       (roi[0]+roi[2],roi[1]+roi[3]),(roi[0],roi[1]+roi[3])])
    path('M'+' L'.join(f'{x},{y}' for x,y in points)+' Z','blue',1.5,'5 5',opacity=.9)
    local=smooth(main.points(G['route_local']))
    recovery=smooth(main.points(G['route_recovery']))
    for d,c in [(local,'amber'),(recovery,'blue')]:
        path(d,'white',8)
        path(d,c,4.5,arrow=True)
    anchor=main.point(G['anchors']['armchair'])
    target=main.point(G['anchors']['cushion'])
    p1,p2,p3=[main.point(G['poses'][q]) for q in ['q1','q2','q3']]
    for p,a,c in [(p1,anchor,'amber'),(p2,anchor,'amber'),(p3,target,'green')]:
        line([p,a],c=c,width=1.8,dash='4 4')
        camera(p,a,c,11)
    # Separate route markers by short leaders so closely spaced local poses stay legible.
    for p,n,offset,c in [(p1,1,(-35,15),'amber'),(p2,2,(36,2),'amber'),(p3,3,(-31,-16),'green')]:
        x,y=p[0]+offset[0],p[1]+offset[1]
        line([(x,y),p],c=c,width=1.8)
        number(x,y,n,c,17)
    circle(*target,13,'none','green',3)
    line([(target[0],target[1]+13),(target[0],target[1]+44),(target[0]-17,target[1]+58)],c='green',width=1.6)
    text(target[0]-20,target[1]+84,'Cushion',25,'ink',anchor='middle')
    line([(anchor[0],anchor[1]-7),(anchor[0],anchor[1]-43),(anchor[0]-23,anchor[1]-56)],c='amber',width=1.6)
    text(anchor[0]-30,anchor[1]-59,'Armchair',25,'ink',anchor='end')
    # Room names sit on quiet floor areas rather than on objects.
    text(118,664,'Bedroom',26,'ink')
    text(535,740,'Living room',26,'ink')
    rect(288,660,200,42,'white','none',rx=3)
    text(388,689,'Change location',25,'blue',anchor='middle')
    line([(388,659),(388,621)],c='blue',width=1.6)
    line([(55,1113),(105,1113)],c='amber',width=4)
    text(120,1122,'Local retry',24,'muted')
    line([(310,1113),(360,1113)],c='blue',width=4)
    text(375,1122,'Same-task recovery',24,'muted')

    # Three observations share one timeline and the same image size.
    xs=[1080,1492,1904]
    pw,ph=388,280
    for x in xs[:-1]:
        line([(x+164,195),(x+379,195)],c='muted',width=1.4,arrow=True)
    for x,n,title,c in zip(xs,[1,2,3],['Inspect','Retry','Verify'],['amber','amber','green']):
        number(x+20,195,n,c,18)
        text(x+51,204,title,29)
    rgb('view1','first',(350,405,1100,794),(xs[0],232,pw,ph))
    rgb('view2','second',(365,405,1100,794),(xs[1],232,pw,ph))
    v3=rgb('view3','found',(350,405,1100,794),(xs[2],232,pw,ph))
    # The thin target box is editorial, not a claimed detector output.
    a=v3.point((952.35/1600,603.525/1200));b=v3.point((1160.15/1600,793.35/1200))
    rect(a[0],a[1],b[0]-a[0],b[1]-a[1],'none','green',2.8)
    for x,label,c in zip(xs,['Target absent','Local limit reached','Target verified'],['amber','amber','green']):
        text(x+pw/2,551,label,27,c,anchor='middle')
    bracket(xs[0],xs[1]+pw,576,'Bedroom armchair · task remains active')
    bracket(xs[2],xs[2]+pw,576,'Living-room sofa · task resolved','green')
    line([(1470,626),(1840,626)],c='blue',width=2,arrow=True)
    text(1655,656,'Recover to another mapped location',23,'blue',anchor='middle')

    line([(1080,693),(2292,693)],c='line',width=1)
    text(1080,739,'c',30,weight=700)
    text(1117,739,'Coverage-aware evidence',28)
    text(1744,739,'d',30,weight=700)
    text(1781,739,'Map update',28)
    # A matched spatial detail gives the new-evidence layer a physical referent.
    detail=scene('coverage',(.223,.252,.116,.151),(1080,773,246,320))
    detail.image(.82)
    coverage_cells(detail,lambda c:c['first'],'seen',.48)
    coverage_cells(detail,lambda c:c['second'] and not c['first'],'amber',.95)
    for q,c,n in [('q1','seen',1),('q2','amber',2)]:
        p=detail.point(G['poses'][q]);a=detail.point(G['anchors']['armchair'])
        camera(p,a,'blue' if n==1 else 'amber',8)
        text(p[0]+13,p[1]+8,str(n),23,'blue' if n==1 else 'amber',700)
    rect(1080,773,246,320,'none','line',1.2)
    # Call out the actual small newly visible patch without inflating its area.
    new=[c for c in G['cells'] if c['second'] and not c['first'] and c['uv'][0]<.27]
    p=detail.point([sum(c['uv'][k] for c in new)/len(new) for k in [0,1]])
    circle(*p,13,'none','amber',1.6)
    line([(p[0]+13,p[1]),(1346,p[1]),(1363,828)],c='amber',width=1.5)
    rect(1363,806,20,20,'amber','none')
    text(1396,825,'Newly seen',24,'amber')
    rect(1363,860,20,20,'seen','none')
    text(1396,879,'Previously seen',24,'muted')
    text(1363,949,'Only new coverage',25)
    text(1363,982,'reduces search belief.',25)
    text(1363,1042,'Anchor stays in the map.',23,'muted')
    text(1203,1125,'Local search region',23,'muted',anchor='middle')

    # Two real crops illustrate spatially consistent support for a novel instance.
    rgb('track1','earlier',(955,632,300,300),(1744,778,116,116))
    rgb('track2','found',(926,566,300,300),(1874,778,116,116))
    line([(2007,836),(2073,836)],c='green',width=2,arrow=True)
    circle(2143,836,35,'pale','green',1.8)
    text(2143,846,'3D',26,'green',anchor='middle')
    text(1867,932,'Consistent observations',24,'muted',anchor='middle')
    text(2143,932,'Track',24,'green',anchor='middle')
    line([(2143,954),(2143,991)],c='green',width=1.8,arrow=True)
    text(2078,980,'After execution',23,'muted',anchor='end')
    # Existing room/object nodes remain; one confirmed new instance is attached.
    rect(1744,1017,175,61,'white','line',1.4,3)
    text(1831.5,1057,'Living room',24,anchor='middle')
    line([(1919,1047),(1956,1047)],c='muted',width=1.5)
    rect(1956,1017,104,61,'white','line',1.4,3)
    text(2008,1057,'Sofa',24,anchor='middle')
    line([(2060,1047),(2097,1047)],c='green',width=1.8)
    rect(2097,1017,195,61,'pale','green',1.8,3)
    text(2194.5,1057,'+ Cushion',25,'green',anchor='middle')
    text(2018,1125,'Confirmed novel instance',23,'muted',anchor='middle')
    add('</g></svg>')
    return ''.join(S)

if __name__=='__main__':
    svg=build()
    handle=Rsvg.Handle.new_with_flags(Rsvg.HandleFlags(1))
    handle.write(svg.encode());handle.close()
    surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,W,H)
    cr=cairo.Context(surface)
    viewport=Rsvg.Rectangle()
    viewport.x=viewport.y=0
    viewport.width=W;viewport.height=H
    handle.render_document(cr,viewport)
    surface.write_to_png(str(OUT/'fig8_reference_preview.png'))
    (OUT/'preview_notes.json').write_text(json.dumps(dict(
        reference=dict(title='Brain-inspired spatial intelligence for embodied agents',
            figure='3b',url='https://www.nature.com/articles/s41467-026-74358-5/figures/3',
            borrowing='Whole-scene route beside chronological observations; no source figure content reused.'),
        provenance=dict(scene=G['source_scene'],rgb_directory=str(FRAMES),
            illustrative=['route and selected visits','local limit after two visits',
                'initially unmapped cushion and graph insertion','editorial target rectangle'],
            coverage=G['coverage_note'],coverage_geometry_limit=G['source_mesh_cut']),
        preview_only=True,labels=LABELS),ensure_ascii=False,indent=2)+'\n')
    print(OUT/'fig8_reference_preview.png')
