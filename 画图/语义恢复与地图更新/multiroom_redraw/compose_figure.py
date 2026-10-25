#!/usr/bin/python3
"""Compose a textured mechanism illustration as editable SVG and vector PDF.

Run with /usr/bin/python3. Scene texture and RGB views are raster assets;
routes, view footprints, leaders, graph, and all labels remain vector objects.
"""
from pathlib import Path
import base64
import html
import json
import math
import cairo
import gi
gi.require_version('Rsvg','2.0')
from gi.repository import Rsvg

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
FRAMES=ROOT/'画图/Habitat长指令执行/keyframes/00166_cushion_recovery_20261024'
G=json.loads((OUT/'assets/projection.json').read_text())
W,H=2500,1810
MAP=(-45,470,1870,1402.5)
C={'ink':'#24323C','muted':'#68747D','gray':'#A5ADB2','line':'#D7DDE0',
   'blue':'#3678A0','orange':'#B98047','green':'#35846C','pale':'#EEF4F6'}
LABELS={
 'en':dict(instruction='“Find a cushion. Check the bedroom armchair first.”',
  schematic='Illustrative mechanism',first='First observation',retry='Local retry',found='Target verified',
  miss='Not found',again='Not found · local limit reached',success='Found · task resolved',
  coverage='New coverage → lower search belief',retained='The armchair remains mapped',
  bedroom='Bedroom',living='Living room',armchair='Armchair',sofa='Sofa',
  bedroom2='Bedroom',bathroom='Bathroom',kitchen='Kitchen',hall='Hall',start='Start',
  recovery='Search another mapped region',hypothesis='Next hypothesis: living-room sofa',
  active='The same task stays active',map_title='Observation-driven map update',
  views='Consistent RGB-D observations',viewa='Earlier view',viewb='Verification view',
  associate='Associate in 3D',track='Confirmed cushion track',
  during='DURING EXECUTION',after='AFTER EXECUTION',insert='Insert a new instance',
  room='Living room',table='Table',cushion='Cushion',new='NEW',
  unchanged='Existing nodes retained',memory='Available to later missions',
  route='Recovery route',local='Local search',cone='View footprint',approach='Approach',
  footer='Illustrative route and coverage; textured scene and observations from HM3D.',
  repeat='Repeated coverage adds no further decrease'),
 'zh':dict(instruction='“寻找一个靠垫，先检查卧室里的扶手椅。”',
  schematic='执行机制示意',first='第一次观察',retry='换视角重试',found='确认目标',
  miss='未找到目标',again='仍未找到 · 达到局部搜索限制',success='找到靠垫 · 当前任务完成',
  coverage='新增覆盖 → 区域搜索置信度降低',retained='扶手椅仍保留在地图中',
  bedroom='卧室',living='客厅',armchair='扶手椅',sofa='沙发',
  bedroom2='卧室',bathroom='浴室',kitchen='厨房',hall='走廊',start='起点',
  recovery='转向另一个已建图区域',hypothesis='新的搜索假设：客厅沙发',
  active='仍然是同一个寻找任务',map_title='观测驱动的地图更新',
  views='多个 RGB-D 视角相互印证',viewa='先前视角',viewb='验证视角',
  associate='在三维空间中关联',track='确认同一靠垫实例',
  during='执行过程中',after='执行结束后',insert='新增一个物体节点',
  room='客厅',table='桌子',cushion='靠垫',new='新增',
  unchanged='已有节点保留',memory='供后续任务使用',
  route='跨区域恢复路线',local='局部搜索',cone='观察范围',approach='前往初始位置',
  footer='轨迹与覆盖为机制示意；场景和观测来自 HM3D。',
  repeat='重复覆盖不再次降低置信度'),
}
PART=[]
TEXT=[]
CACHE={}
MEASURE=cairo.Context(cairo.ImageSurface(cairo.FORMAT_ARGB32,1,1))
LANG='en'

def add(s):PART.append(s)
def text(x,y,s,size=32,color='ink',bold=False,anchor='start',halo=False):
    size=max(30,size)  # >= 6.26 pt at the exported 184 mm width.
    family='Noto Sans CJK SC' if LANG=='zh' else 'Liberation Sans'
    fill=C.get(color,color)
    MEASURE.select_font_face(family,cairo.FONT_SLANT_NORMAL,cairo.FONT_WEIGHT_BOLD if bold else cairo.FONT_WEIGHT_NORMAL)
    MEASURE.set_font_size(size)
    width=MEASURE.text_extents(s).x_advance
    left=x if anchor=='start' else x-width/2 if anchor=='middle' else x-width
    assert left>=-1 and left+width<=W+1,(s,left,width)
    TEXT.append({'text':s,'left':left,'top':y-size,'right':left+width,'bottom':y+size*.2})
    stroke=' paint-order="stroke" stroke="white" stroke-width="5" stroke-linejoin="round"' if halo else ''
    add(f'<text x="{x}" y="{y}" font-family="{family}" font-size="{size}" font-weight="{700 if bold else 400}" fill="{fill}" text-anchor="{anchor}"{stroke}>{html.escape(s)}</text>')
def path(d,color='gray',width=2,arrow=False,dash=None,opacity=1):
    cc=C.get(color,color)
    mark=f' marker-end="url(#arrow-{cc[1:]})"' if arrow else ''
    ds=f' stroke-dasharray="{dash}"' if dash else ''
    add(f'<path d="{d}" fill="none" stroke="{cc}" stroke-width="{width}" opacity="{opacity}"{mark}{ds}/>')
def line(points,**kwargs):
    path('M'+' L'.join(f'{x:.2f},{y:.2f}' for x,y in points),**kwargs)
def rect(x,y,w,h,fill='none',stroke='line',width=1.5,rx=0):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{C.get(fill,fill)}" stroke="{C.get(stroke,stroke)}" stroke-width="{width}"/>')
def circle(x,y,r,fill='white',stroke='gray',width=2):
    add(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{C.get(fill,fill)}" stroke="{C.get(stroke,stroke)}" stroke-width="{width}"/>')
def badge(x,y,num,color,r=23):
    circle(x,y,r,'white',color,3)
    text(x,y+10,str(num),29,color,True,'middle')
def img(p,x,y,w,h,crop=None,opacity=1):
    p=Path(p)
    if p not in CACHE:CACHE[p]=base64.b64encode(p.read_bytes()).decode()
    if crop:
        cx,cy,cw,ch=crop
        add(f'<svg x="{x}" y="{y}" width="{w}" height="{h}" viewBox="{cx} {cy} {cw} {ch}" overflow="hidden">')
        add(f'<image width="1600" height="1200" href="data:image/png;base64,{CACHE[p]}"/>')
        add('</svg>')
    else:
        add(f'<image x="{x}" y="{y}" width="{w}" height="{h}" opacity="{opacity}" href="data:image/png;base64,{CACHE[p]}"/>')
def point(uv):return MAP[0]+MAP[2]*uv[0],MAP[1]+MAP[3]*uv[1]
def smooth(points):
    # Catmull-Rom, with conservative tension to preserve the chosen doorway.
    out=f'M {points[0][0]:.2f},{points[0][1]:.2f}'
    for i in range(len(points)-1):
        p0=points[max(0,i-1)];p1=points[i];p2=points[i+1];p3=points[min(len(points)-1,i+2)]
        c1=(p1[0]+(p2[0]-p0[0])*.12,p1[1]+(p2[1]-p0[1])*.12)
        c2=(p2[0]-(p3[0]-p1[0])*.12,p2[1]-(p3[1]-p1[1])*.12)
        out+=f' C{c1[0]:.2f},{c1[1]:.2f} {c2[0]:.2f},{c2[1]:.2f} {p2[0]:.2f},{p2[1]:.2f}'
    return out
def corners(x,y,w,h,color,width=3,length=20):
    for a,b,dx,dy in [(x,y,1,1),(x+w,y,-1,1),(x+w,y+h,-1,-1),(x,y+h,1,-1)]:
        path(f'M{a+dx*length},{b} L{a},{b} L{a},{b+dy*length}',color,width)
def obb(vertices,color,width):
    for i in range(8):
        for j in range(i+1,8):
            if (i^j) in [1,2,4]:line([point(vertices[i]),point(vertices[j])],color=color,width=width)
def check(x,y,color='green',scale=1):
    path(f'M{x-10*scale},{y} l{7*scale},{7*scale} l{15*scale},{-18*scale}',color,3*scale)
def node(x,y,w,h,label,color='gray',fill='white'):
    rect(x,y,w,h,fill,color,2,3)
    text(x+w/2,y+h/2+11,label,33,'ink' if color=='gray' else color,False,'middle')

def build(language):
    global LANG
    LANG=language;PART.clear();TEXT.clear();L=LABELS[LANG]
    add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
    add('<title>Semantic recovery and observation-driven map update — mechanism illustration</title>')
    add('<desc>Author-designed route and coverage over a textured HM3D cutaway. The target is treated as initially unmapped for illustration. Same-task recovery; task resolution online and new-instance insertion after execution.</desc>')
    add('<defs>')
    for color in C.values():
        add(f'<marker id="arrow-{color[1:]}" markerWidth="15" markerHeight="15" refX="11" refY="6" viewBox="0 0 12 12" orient="auto" markerUnits="userSpaceOnUse"><path d="M0,0 L12,6 L0,12 Z" fill="{color}"/></marker>')
    add('</defs><g stroke-linecap="round" stroke-linejoin="round">')
    rect(0,0,W,H,'white','none')
    text(42,53,L['instruction'],39,bold=True)
    text(2454,51,L['schematic'],29,'muted',anchor='end')
    line([(42,79),(2454,79)],color='line',width=1.3)

    # The scene is the spatial reference for all numbered observations.
    img(OUT/'assets/cutaway.png',*MAP)
    approach=smooth([point(uv) for uv in G['route_approach']])
    path(approach,'white',8,opacity=.85)
    path(approach,'gray',3.6,arrow=True,dash='10 8')
    start=point(G['poses']['start'])
    circle(*start,8,'white','gray',3)
    text(start[0]+15,start[1]+3,L['start'],30,'muted',halo=True)
    for key in ['q1','q2','q3']:
        pts=[point(uv) for uv in G['frusta'][key]]
        color=C['orange'] if key!='q3' else C['green']
        add(f'<polygon points="'+ ' '.join(f'{x},{y}' for x,y in pts)+f'" fill="{color}" fill-opacity=".10" stroke="{color}" stroke-width="1.8"/>')
    for sample in G['coverage_samples']:
        x,y=point(sample['uv'])
        circle(x,y,3.4,'orange' if sample['category']=='new' else 'white','orange',1.1)
    for key,color in [('route_local','orange'),('route_recovery','blue')]:
        pts=[point(uv) for uv in G[key]];d=smooth(pts)
        path(d,'white',10,opacity=.93)
        path(d,color,5,arrow=True)
    obb(G['target_box'],'green',3)
    obb(G['anchor_box'],'gray',1.7)
    for n,key,color,dx,dy in [(1,'q1','orange',-18,22),(2,'q2','orange',20,-22),(3,'q3','green',-17,8)]:
        x,y=point(G['poses'][key])
        line([(x,y),(x+dx,y+dy)],color=color,width=2)
        badge(x+dx,y+dy,n,color,21)
    # Mid-route arrow ensures the left-to-right transition reads immediately.
    pts=[point(uv) for uv in G['route_recovery']]
    a,b=pts[4],pts[5]
    line([(a[0]+(b[0]-a[0])*.25,a[1]+(b[1]-a[1])*.25),
          (a[0]+(b[0]-a[0])*.7,a[1]+(b[1]-a[1])*.7)],color='blue',width=5,arrow=True)

    # Tight RGB crops make the actual object and the negative observations legible.
    observations=[(50,'obs_000000.png',(420,500,1080,700),L['first'],L['miss'],'orange'),
                  (585,'obs_000001.png',(420,500,1080,700),L['retry'],L['again'],'orange'),
                  (1160,'obs_000003.png',(400,500,1080,700),L['found'],L['success'],'green')]
    iw,ih=460,298
    for i,(x,filename,crop,title,state,color) in enumerate(observations,1):
        badge(x+22,116,i,color,20)
        text(x+58,127,title,33,bold=True)
        img(FRAMES/filename,x,148,iw,ih,crop)
        rect(x,148,iw,ih,'none','line',1.5)
        text(x+iw/2,482,state,28,color,anchor='middle')
        if i==3:
            # The patterned loose cushion, with the actual recorded bounding box.
            b=[952.35,603.525,1160.15,793.35]
            sx,sy=iw/crop[2],ih/crop[3]
            corners(x+(b[0]-crop[0])*sx,148+(b[1]-crop[1])*sy,
                    (b[2]-b[0])*sx,(b[3]-b[1])*sy,'green',3,15)
    line([(527,297),(567,297)],color='gray',width=2,arrow=True)
    line([(1065,297),(1140,297)],color='blue',width=2,arrow=True)

    # Coverage feedback is attached to the actual armchair search region.
    text(48,554,L['coverage'],31,'orange',bold=True)
    text(48,596,L['retained'],30,'muted')
    circle(55,637,5,'orange','orange',1)
    text(73,647,'Newly seen' if LANG=='en' else '新增覆盖',30,'muted')
    circle(300,637,5,'white','orange',1.5)
    text(318,647,'Already seen' if LANG=='en' else '已有覆盖',30,'muted')
    arm=point(G['anchors']['armchair'])
    line([(465,612),(520,705),(arm[0]-20,arm[1]-16)],color='orange',width=1.8)
    for key,label,dx,dy in [('bedroom','bedroom',-60,160),('bedroom_2','bedroom2',-4,150),
                           ('kitchen','kitchen',30,30),('bathroom','bathroom',-45,25)]:
        x,y=point(G['anchors'][key]);text(x+dx,y+dy,L[label],33,'ink',True,'middle',True)
    x,y=point(G['anchors']['living_room']);text(x+105,y+210,L['living'],36,'ink',True,'middle',True)
    target=point(G['anchors']['cushion'])
    check(target[0]+38,target[1]-8,scale=1.5)
    text(target[0]+47,target[1]+38,L['cushion'],30,'green',bold=True,halo=True)
    text(697,1285,L['recovery'],33,'blue',bold=True,halo=True)
    text(697,1328,L['hypothesis'],30,'ink',halo=True)
    text(697,1371,L['active'],30,'muted',halo=True)
    door=point(G['anchors']['door'])
    line([(674,1296),(door[0]+18,door[1]+12)],color='blue',width=1.8)

    # Persistent memory: RGB evidence accumulated during execution, insertion after.
    line([(1777,106),(1777,1730)],color='line',width=1.5)
    text(1832,127,L['map_title'],36,bold=True)
    text(1832,190,L['during'],25,'muted')
    text(1832,240,L['views'],32)
    img(FRAMES/'frame_000420.png',1832,274,244,244,(955,632,300,300))
    img(FRAMES/'obs_000003.png',2180,274,244,244,(926,566,300,300))
    corners(1832,274,244,244,'green',2.5,20)
    corners(2180,274,244,244,'green',2.5,20)
    text(1954,560,L['viewa'],28,'muted',anchor='middle')
    text(2302,560,L['viewb'],28,'muted',anchor='middle')
    line([(1954,583),(1954,613),(2128,613)],color='gray',width=2)
    line([(2302,583),(2302,613),(2128,613),(2128,650)],color='gray',width=2,arrow=True)
    text(2128,695,L['associate'],31,'muted',anchor='middle')
    rect(1880,730,496,92,'#F4F9F6','green',2,4)
    check(1923,775,scale=1.4)
    text(2140,787,L['track'],33,'green',True,'middle')
    line([(2128,838),(2128,880)],color='green',width=2,arrow=True)
    text(1832,926,L['after'],25,'muted')
    text(1832,972,L['insert'],33,bold=True)
    # Existing hierarchy is gray; just the inserted target and its edge are green.
    node(2023,1004,210,63,L['room'])
    line([(2128,1067),(2128,1086),(1982,1086),(1982,1110)],color='gray',width=2)
    line([(2128,1086),(2294,1086),(2294,1110)],color='gray',width=2)
    node(1887,1110,190,63,L['sofa'])
    node(2199,1110,190,63,L['table'])
    line([(1982,1173),(1982,1220)],color='green',width=3,arrow=True)
    node(1887,1220,190,63,L['cushion'],'green','#F4F9F6')
    text(2104,1261,'+ '+L['new'],27,'green',bold=True)
    text(2424,1402,L['unchanged'],28,'muted',anchor='end')
    text(2424,1443,L['memory'],29,'muted',anchor='end')

    # A compact visual key, separate from the narrative.
    line([(48,1745),(95,1745)],color='gray',width=3.6,dash='9 7')
    text(108,1754,L['approach'],30,'muted')
    line([(440,1745),(487,1745)],color='orange',width=4)
    text(500,1754,L['local'],30,'muted')
    line([(800,1745),(847,1745)],color='blue',width=4,arrow=True)
    text(860,1754,L['route'],30,'muted')
    add('</g></svg>')
    svg='\n'.join(PART)
    name='fig8_multiroom_'+LANG
    (OUT/(name+'.svg')).write_text(svg)
    handle=Rsvg.Handle.new_with_flags(Rsvg.HandleFlags(1))
    handle.write(svg.encode())
    handle.close()
    # 184 mm wide, matching the manuscript's two-column figure width.
    scale=184/25.4*72/W
    pdf=cairo.PDFSurface(str(OUT/(name+'.pdf')),W*scale,H*scale)
    cr=cairo.Context(pdf);cr.scale(scale,scale);handle.render_cairo(cr);pdf.finish()
    surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,W,H)
    handle.render_cairo(cairo.Context(surface));surface.write_to_png(str(OUT/(name+'.png')))
    (OUT/(name+'_text_bounds.json')).write_text(json.dumps(TEXT,ensure_ascii=False,indent=2)+'\n')
    print(name, 'exported', len(TEXT),'labels')

if __name__=='__main__':
    build('en')
    build('zh')
