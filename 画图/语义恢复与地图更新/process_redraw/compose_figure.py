#!/usr/bin/python3
"""English process-first Fig. 8. Reuses the approved real apartment render.

Run with /usr/bin/python3. The PDF retains vector text, routes and diagram.
Image windows are native SVG crops of the unmodified source RGB renders.
"""
from pathlib import Path
import base64
import html
import json
import cairo
import gi
gi.require_version('Rsvg', '2.0')
from gi.repository import Rsvg

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
SOURCE = OUT.parent / 'multiroom_redraw'
FRAMES = ROOT / '画图/Habitat长指令执行/keyframes/00166_cushion_recovery_20261024'
G = json.loads((SOURCE / 'assets/projection.json').read_text())
W, H = 2400, 1280
MAP = (493, 159, 1460, 1095)
C = dict(ink='#29343C', muted='#66747E', gray='#AAB3B9', line='#DCE2E6',
         blue='#397B9D', amber='#A87943', green='#33846E')
PART, TEXT, CROPS = [], [], []
CACHE = {}
MEASURE = cairo.Context(cairo.ImageSurface(cairo.FORMAT_ARGB32, 1, 1))

def add(s):
    PART.append(s)

def col(s):
    return C.get(s, s)

def text(x, y, s, size=36, color='ink', bold=False, anchor='start', halo=False):
    assert size >= 34
    MEASURE.select_font_face('Liberation Sans', cairo.FONT_SLANT_NORMAL,
                             cairo.FONT_WEIGHT_BOLD if bold else cairo.FONT_WEIGHT_NORMAL)
    MEASURE.set_font_size(size)
    width = MEASURE.text_extents(s).x_advance
    left = x if anchor == 'start' else x-width/2 if anchor == 'middle' else x-width
    assert left >= 0 and left+width <= W, (s, left, width)
    TEXT.append(dict(text=s, left=left, top=y-size, right=left+width,
                     bottom=y+size*.22, size_px=size))
    stroke = ' paint-order="stroke" stroke="white" stroke-width="5"' if halo else ''
    add(f'<text x="{x}" y="{y}" font-family="Liberation Sans" font-size="{size}" '
        f'font-weight="{700 if bold else 400}" fill="{col(color)}" '
        f'text-anchor="{anchor}"{stroke}>{html.escape(s)}</text>')

def path(d, color='gray', width=2, arrow=False, dash=None, opacity=1):
    mark = f' marker-end="url(#arrow-{color})"' if arrow else ''
    ds = f' stroke-dasharray="{dash}"' if dash else ''
    add(f'<path d="{d}" fill="none" stroke="{col(color)}" stroke-width="{width}" '
        f'opacity="{opacity}"{mark}{ds}/>')

def line(points, **kwargs):
    path('M' + ' L'.join(f'{x:.2f},{y:.2f}' for x,y in points), **kwargs)

def rect(x,y,w,h,fill='none',stroke='line',width=1.5,rx=0):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" '
        f'fill="{col(fill)}" stroke="{col(stroke)}" stroke-width="{width}"/>')

def circle(x,y,r,fill='white',stroke='gray',width=2):
    add(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{col(fill)}" '
        f'stroke="{col(stroke)}" stroke-width="{width}"/>')

def badge(x,y,n,color):
    circle(x,y,25,color,color,1)
    text(x,y+12,str(n),36,'white',True,'middle')

def heading(x,y,n,label,color):
    badge(x+25,y-13,n,color)
    text(x+65,y,label,39,'ink',True)

def image(p,x,y,w,h,crop=None,opacity=1):
    p = Path(p)
    if p not in CACHE:
        CACHE[p] = base64.b64encode(p.read_bytes()).decode()
    if crop:
        cx,cy,cw,ch = crop
        CROPS.append(dict(path=str(p),crop_xywh=crop))
        add(f'<svg x="{x}" y="{y}" width="{w}" height="{h}" '
            f'viewBox="{cx} {cy} {cw} {ch}" overflow="hidden">')
        add(f'<image width="1600" height="1200" href="data:image/png;base64,{CACHE[p]}"/>')
        add('</svg>')
    else:
        add(f'<image x="{x}" y="{y}" width="{w}" height="{h}" '
            f'opacity="{opacity}" href="data:image/png;base64,{CACHE[p]}"/>')

def point(uv):
    return MAP[0]+MAP[2]*uv[0], MAP[1]+MAP[3]*uv[1]

def smooth(points):
    d = f'M{points[0][0]},{points[0][1]}'
    for i in range(len(points)-1):
        p0,p1,p2,p3 = points[max(0,i-1)],points[i],points[i+1],points[min(len(points)-1,i+2)]
        c1 = (p1[0]+.12*(p2[0]-p0[0]),p1[1]+.12*(p2[1]-p0[1]))
        c2 = (p2[0]-.12*(p3[0]-p1[0]),p2[1]-.12*(p3[1]-p1[1]))
        d += f' C{c1[0]},{c1[1]} {c2[0]},{c2[1]} {p2[0]},{p2[1]}'
    return d

def corners(x,y,w,h,color='green',width=4,length=23):
    for a,b,dx,dy in [(x,y,1,1),(x+w,y,-1,1),(x+w,y+h,-1,-1),(x,y+h,1,-1)]:
        path(f'M{a+dx*length},{b} L{a},{b} L{a},{b+dy*length}',color,width)

def node(x,y,w,label,new=False):
    rect(x,y,w,62,'#EDF6F2' if new else 'white','green' if new else 'gray',2.3,3)
    text(x+w/2,y+43,label,36,'green' if new else 'ink',new,'middle')

def build():
    add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
    add('<title>From a missed target to recovery and map update</title>')
    add('<desc>Five numbered actions: inspect the bedroom armchair, retry locally, '
        'search another mapped room after the local limit, verify a cushion, and add '
        'a confirmed novel instance after execution. The same locate-target task remains '
        'active during recovery. Route and initial memory state are illustrative.</desc>')
    add('<defs>')
    for key,value in C.items():
        add(f'<marker id="arrow-{key}" markerWidth="22" markerHeight="22" refX="18" refY="9" '
            f'viewBox="0 0 20 18" orient="auto" markerUnits="userSpaceOnUse">'
            f'<path d="M0,0 L20,9 L0,18 Z" fill="{value}"/></marker>')
    add('</defs><g stroke-linecap="round" stroke-linejoin="round">')
    rect(0,0,W,H,'white','none')
    text(32,55,'Find a cushion.',42,bold=True)
    text(355,55,'Start at the bedroom armchair.',38,'muted')
    line([(32,85),(2368,85)],color='line')

    # The complete apartment supplies spatial context; only relevant places are labelled.
    image(SOURCE/'assets/cutaway.png',*MAP)
    for route,color in [('route_local','amber'),('route_recovery','blue')]:
        d = smooth([point(p) for p in G[route]])
        path(d,'white',14,opacity=.98)
        path(d,color,7,arrow=True)
    # A single viewing cone shows why the final pose can verify the sofa target.
    pts = [point(p) for p in G['frusta']['q3']]
    add('<polygon points="'+' '.join(f'{x},{y}' for x,y in pts)+'" '
        f'fill="{C["green"]}" fill-opacity=".12" stroke="{C["green"]}" stroke-width="2.5"/>')
    for key in ['q1','q2']:
        p = point(G['poses'][key]);a = point(G['anchors']['armchair'])
        line([p,a],color='amber',width=2.5,dash='5 5')
    for key,num,color in [('q1',1,'amber'),('q2',2,'amber'),('q3',4,'green')]:
        x,y = point(G['poses'][key])
        circle(x,y,21,'white',color,3)
        text(x,y+11,str(num),34,color,True,'middle')
    # The endpoint is an object cue, without a detailed projected cuboid.
    target = point(G['anchors']['cushion'])
    circle(*target,17,'white','green',3)
    path(f'M{target[0]-8},{target[1]} l6,6 l12,-15','green',3.5)

    heading(32,155,1,'Inspect armchair','amber')
    image(FRAMES/'obs_000000.png',32,185,500,320,(350,480,1100,704))
    rect(32,185,500,320)
    text(282,551,'No cushion found',36,'amber',anchor='middle')
    line([(282,576),(282,613)],color='amber',width=3,arrow=True)

    heading(32,673,2,'Try another view','amber')
    image(FRAMES/'obs_000001.png',32,703,500,320,(350,480,1100,704))
    rect(32,703,500,320)
    text(282,1069,'Still missing',36,'amber',anchor='middle')
    text(282,1117,'Local limit reached',36,'ink',True,'middle')
    # One concise evidence note, replacing the coverage sample legend.
    text(32,1204,'New coverage lowers belief.',34,'muted')
    text(32,1246,'The armchair stays mapped.',34,'muted')

    heading(650,155,3,'Search another room','blue')
    text(715,201,'Same task · new mapped location',36,'muted')
    # Process arrows are separate from the physical route.
    line([(555,864),(624,864)],color='blue',width=4,arrow=True)
    line([(1787,346),(1845,346)],color='green',width=4,arrow=True)
    # Labels sit on the floor with a white halo; no unrelated room labels.
    text(810,681,'Bedroom',39,'ink',True,'middle',True)
    text(1325,738,'Living room',39,'ink',True,'middle',True)
    arm = point(G['anchors']['armchair'])
    line([(arm[0]-10,arm[1]-15),(arm[0]-50,arm[1]-65)],color='amber',width=2.5)
    text(arm[0]-60,arm[1]-76,'Armchair',36,'amber',True,'end',True)
    line([(target[0]+18,target[1]),(target[0]+68,target[1]+8)],color='green',width=2.5)
    text(target[0]+77,target[1]+20,'Sofa',36,'green',True,halo=True)
    heading(1870,155,4,'Verify cushion','green')
    crop = (730,530,760,486.4)
    image(FRAMES/'obs_000003.png',1870,185,500,320,crop)
    rect(1870,185,500,320)
    bbox = [952.35,603.525,1160.15,793.35]
    sx,sy = 500/crop[2],320/crop[3]
    corners(1870+(bbox[0]-crop[0])*sx,185+(bbox[1]-crop[1])*sy,
            (bbox[2]-bbox[0])*sx,(bbox[3]-bbox[1])*sy)
    text(2120,551,'Found · task resolved',36,'green',True,'middle')

    text(2120,615,'RGB-D views → 3D track',34,'muted',anchor='middle')
    line([(2120,635),(2120,664)],color='green',width=3,arrow=True)
    line([(1870,685),(2370,685)],color='line',width=2,dash='7 7')
    text(1870,735,'AFTER EXECUTION',34,'muted')
    heading(1870,801,5,'Update object map','green')
    text(1870,851,'Insert the confirmed instance',34,'muted')
    # Existing hierarchy retained; only a new object and edge are coloured.
    node(1994,893,252,'Living room')
    line([(2120,955),(2120,987)],color='gray',width=2.5)
    node(2030,987,180,'Sofa')
    line([(2120,1049),(2120,1097)],color='green',width=3,arrow=True)
    node(2015,1097,210,'Cushion',True)
    text(2244,1140,'+ NEW',34,'green',True)
    text(2120,1226,'Available to later tasks',34,'muted',anchor='middle')
    add('</g></svg>')

    svg = '\n'.join(PART)
    name = 'fig8_process_en'
    (OUT/f'{name}.svg').write_text(svg)
    handle = Rsvg.Handle.new_with_flags(Rsvg.HandleFlags(1))
    handle.write(svg.encode()); handle.close()
    scale = 184/25.4*72/W
    surface = cairo.PDFSurface(str(OUT/f'{name}.pdf'),W*scale,H*scale)
    cr = cairo.Context(surface); cr.scale(scale,scale)
    handle.render_cairo(cr); surface.finish()
    preview = cairo.ImageSurface(cairo.FORMAT_ARGB32,W,H)
    handle.render_cairo(cairo.Context(preview))
    preview.write_to_png(str(OUT/f'{name}.png'))
    (OUT/'layout.json').write_text(json.dumps(dict(text=TEXT,crops=CROPS,
        width_px=W,height_px=H,width_mm=184,height_mm=184*H/W,
        minimum_font_pt=min(t['size_px'] for t in TEXT)*scale),indent=2)+'\n')
    print(f'Exported {name}: {len(TEXT)} labels; 184 × {184*H/W:.2f} mm')

if __name__ == '__main__':
    build()
