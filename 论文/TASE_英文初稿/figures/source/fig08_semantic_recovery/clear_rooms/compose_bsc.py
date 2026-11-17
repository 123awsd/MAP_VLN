#!/usr/bin/python3
"""BSC-Nav Fig. 4c layout study, using this project's scene and observations."""
from pathlib import Path
import sys,json,math,html,re
import numpy as np
import cairo
import gi
gi.require_version('Rsvg','2.0')
from gi.repository import Rsvg
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(OUT.parent))
import compose_preview as c

G=json.loads((OUT/'episode_geometry.json').read_text())
OFF=OUT.parent/'assets/hm3d_official'
W,H=2800,1150
C=dict(ink='#222831',muted='#656B71',line='#C4CACD',start='#45ADD1',
       waypoint='#637DB5',miss='#BC8B36',recover='#9365B0',goal='#3B995F',purple='#8750A1')
c.C.update(C);c.S.clear();c.LABELS.clear()
add=c.add;rect=c.rect;path=c.path;line=c.line;circle=c.circle
labels=[]

def text(x,y,s,size=30,color='ink',weight=400,anchor='start',italic=False):
    labels.append(s)
    add(f'<text x="{x}" y="{y}" font-family="TeX Gyre Termes" font-size="{size}" '
        f'font-weight="{weight}" font-style="{"italic" if italic else "normal"}" '
        f'text-anchor="{anchor}" fill="{c.col(color)}">{html.escape(s)}</text>')

def flag(x,y,color,label='',scale=1,check=False):
    add(f'<g transform="translate({x},{y}) scale({scale}) rotate(-12)">')
    path('M0,0 V-52','white',6)
    path('M0,0 V-52','ink',2)
    d='M1,-50 Q13,-56 25,-50 Q37,-44 49,-50 V-21 Q37,-15 25,-21 Q13,-27 1,-21 Z'
    add(f'<path d="{d}" fill="{c.col(color)}" stroke="white" stroke-width="3"/>')
    if check:
        for xx,yy in [(7,-47),(25,-41),(7,-30),(35,-29)]:rect(xx,yy,9,9,'white','none')
    elif label:
        add(f'<text x="25" y="-29" fill="white" font-family="TeX Gyre Heros" font-weight="700" '
            f'font-size="24" text-anchor="middle">{html.escape(label)}</text>')
    add('</g>')

def qwen(x,y,size):
    add(f'<svg x="{x}" y="{y}" width="{size}" height="{size}" viewBox="245 290 1080 1050" overflow="hidden"><use href="#qwen"/></svg>')

def arrow(a,b,color='ink',size=10):
    a=np.array(a);b=np.array(b);d=(b-a)/np.linalg.norm(b-a);n=np.array([-d[1],d[0]])
    c.polygon([b,b-size*d+.45*size*n,b-size*d-.45*size*n],color)

def color_at(t):
    stops=[(0,'#31BFA6'),(.33,'#BDD34B'),(.65,'#EAC64D'),(1,'#E39043')]
    for (u,a),(v,b) in zip(stops,stops[1:]):
        if t<=v:
            f=max(0,min(1,(t-u)/(v-u)))
            rgb=[round(int(a[i:i+2],16)*(1-f)+int(b[i:i+2],16)*f) for i in (1,3,5)]
            return '#'+''.join(f'{i:02x}' for i in rgb)
    return stops[-1][1]

def route(project,key,stop=5,width=7,halo=True):
    total=sum(G['segment_lengths_m']);covered=0
    for i,r in enumerate(G['routes'][:stop]):
        ids=list(range(0,len(r[key])-1,2))+[len(r[key])-1]
        points=[project(r[key][j]) for j in ids]
        distance=np.linalg.norm(np.diff(np.array(r['falcon'])[ids,:2],axis=0),axis=1)
        if halo:line(points,c='white',width=width+2.8,opacity=.95)
        for a,b,dist in zip(points,points[1:],distance):
            subdivisions=max(1,math.ceil(dist*5))
            for j in range(subdivisions):
                u=j/subdivisions;v=(j+1)/subdivisions
                pp=np.array(a)*(1-u)+np.array(b)*u;qq=np.array(a)*(1-v)+np.array(b)*v
                line([pp,qq],c=color_at((covered+dist*(u+v)/2)/total),width=width)
            covered+=float(dist)

def image_view(name,asset,iw,ih,crop,box,border=None):
    v=c.View(name,asset,iw,ih,crop,box);v.image()
    if border:rect(*box,'none',border,1.7)
    return v

def topdown(i,x,y,w,h):
    mh,mw=G['map']['shape']
    v=image_view(f'map-{i}','nav',mw,mh,(0,0,mw,mh),(x,y,w,h))
    def p(q):return v.point((q[0]/mw,q[1]/mh))
    route(p,'grid_pixel',i,width=2.9,halo=False)
    for j in range(i+1):
        pp=p(G['stages'][j]['grid_pixel'])
        circle(*pp,2.5,'white',C['start' if j==0 else 'goal' if j==5 else 'waypoint'],1)
    s=G['stages'][i];pos=np.array(s['grid_pixel'])
    # u=-Falcon Y; v=-Falcon X, same orientation as the official RGB plate.
    aim=pos+np.array([-math.sin(s['yaw']),-math.cos(s['yaw'])])*15
    c.camera(p(pos),p(aim),'start',6.0)
    rect(x,y,w,h,'none','#65696D',1.3)

def cue(name,crop,box):return image_view('cue-'+name,name,1600,1000,crop,box,'purple')

def build(publication=False):
    add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
    add('<title>Long instruction execution with an intermediate semantic recovery</title><defs>')
    c.asset('floor',OUT/'assets/official_l2.png',1280,1116)
    c.asset('qwen',OFF/'qwen3_official.png',3944,1555)
    mh,mw=G['map']['shape'];c.asset('nav',OUT/'assets/habitat_topdown.png',mw,mh)
    for s in G['stages']:c.asset(s['name'],OUT/'assets'/f"{s['name']}.png",1600,1000)
    add('</defs><g stroke-linejoin="round" stroke-linecap="round">')
    rect(0,0,W,H,'white','none')
    text(40,45,'Navigation Instruction',39,weight=700)
    rect(40,65,2720,130,'white','#234754',2.1,15)
    text(68,142,'“Guests are arriving. Check the kettle on the kitchen counter, then look for a cushion on the dining chairs. If it is not there, keep looking nearby. Finally, check the lamp between the beds in the twin bedroom.”',30,weight=700)

    # Left scene panel follows the relative size in the reference composition.
    m=c.View('main','floor',1280,1116,(0,0,1280,1116),(20,226,945,885));m.image(.78)
    def mp(q):return m.point((q[0]/1280,q[1]/1116))
    route(mp,'official_pixel',width=7.2)
    for i,r in enumerate(G['routes']):
        a=np.array(r['official_pixel']);lengths=np.linalg.norm(np.diff(a,axis=0),axis=1)
        if len(lengths)==0:continue
        j=min(len(a)-2,int(np.searchsorted(np.cumsum(lengths),lengths.sum()*.52)))
        p=mp(a[j]);q=mp(a[j+1]);end=np.array(p)*.45+np.array(q)*.55
        c.camera(tuple(end),q,'start',8)
    # The two local observations share one failed subgoal flag.
    flags=[(0,'start','',False),(1,'waypoint','1',False),(3,'miss','2',False),
           (4,'recover','2′',False),(5,'goal','3',True)]
    for i,col,n,check in flags:
        p=mp(G['stages'][i]['official_pixel']);circle(*p,6,'white',col,2)
        flag(*p,col,n,1.0,check)
    target=mp((1215,445))
    a=mp((1194,426));b=mp((1230,463))
    rect(*a,b[0]-a[0],b[1]-a[1],'none','recover',2)
    line([mp(G['stages'][4]['official_pixel']),target],c='recover',width=1.7,dash='5 4')
    # Reference-like target inset sits over an unused corner of the floor plate.
    inset=(705,290,239,166)
    line([target,(932,525),(932,494)],c='recover',width=1.8,dash='5 4')
    image_view('map-inset','sofa',1600,1000,(160,410,720,500),inset,'recover')
    text(824.5,483,'Cushion found',27,'recover',700,'middle')

    xs=[1010+i*248 for i in range(6)];cw=230
    centers=[x+cw/2 for x in xs]
    # Goal descriptions / cues occupy one aligned row above the execution axis.
    rect(xs[0]+16,282,198,75,'#E6F2F6','#588292',1.5,11)
    text(centers[0],313,'Subgoal',30,weight=700,anchor='middle')
    text(centers[0],343,'planning',30,weight=700,anchor='middle')
    rect(xs[0]+16,391,198,75,'#F2EAF5','#9474A2',1.5,11)
    text(centers[0],423,'Semantic',30,weight=700,anchor='middle')
    text(centers[0],453,'recovery',30,weight=700,anchor='middle')
    goals=[(1,'Subgoal 1','Kettle on counter','kitchen',(710,140,390,390)),
           (2,'Subgoal 2','Cushion on chair','chair_first',(0,0,1100,1000)),
           (4,'Revised subgoal','Cushion on sofa','sofa',(175,410,560,560)),
           (5,'Subgoal 3','Bedside lamp','lamp',(650,300,430,430))]
    for i,title,label,name,crop in goals:
        text(centers[i],268,title,31,weight=700,anchor='middle')
        text(centers[i],309,label,28,'purple',anchor='middle',italic=True)
        box=(centers[i]-83,344,166,166)
        # A square target crop ensures uniform visual rhythm in the cue row.
        cx,cy,ww,hh=crop;side=min(ww,hh);crop=(cx+(ww-side)/2,cy+(hh-side)/2,side,side)
        cue(name,crop,box)
        line([(centers[i],510),(centers[i],559)],c='ink',width=1.35,dash='3 4')
    text(centers[3],268,'Recovery',31,weight=700,anchor='middle')
    rect(xs[3]-2,300,cw+4,210,'white','#B5A1C1',1.6,10)
    qwen(centers[3]-25,314,50)
    text(centers[3],393,'Qwen',28,'purple',700,'middle')
    text(centers[3],431,'No cushion seen.',26,anchor='middle')
    text(centers[3],461,'Likely: sofa in the',25,'purple',700,'middle')
    text(centers[3],494,'east living room',27,'purple',700,'middle')
    # Thought dots attach the failed observation to the candidate statement.
    circle(centers[3],530,5,'white','#B5A1C1',1.4)
    circle(centers[3],547,3,'white','#B5A1C1',1.2)
    line([(xs[3]+cw+5,420),(xs[4]+23,420)],c='purple',width=1.8)
    arrow((xs[3]+cw+5,420),(xs[4]+23,420),'purple',8)

    # Same flags repeat on the time axis, the scene and the compact legend.
    yaxis=572
    line([(xs[0]+10,yaxis),(xs[-1]+cw,yaxis)],c='purple',width=2.5)
    arrow((xs[-1]+cw-20,yaxis),(xs[-1]+cw+10,yaxis),'purple',10)
    cols=['start','waypoint','miss','miss','recover','goal']
    nums=['','1','2','','2′','3']
    for i,(x,col,n) in enumerate(zip(centers,cols,nums)):
        circle(x,yaxis,5,'ink','white',1)
        if i==3:qwen(x-16,599,38)
        else:flag(x-12,639,col,n,.65,i==5)
        line([(x,yaxis+7),(x,594)],c='#A7ACB0',width=1)
    text(xs[0],668,'Habitat top-down maps',25,'muted')
    for i,x in enumerate(xs):topdown(i,x,684,cw,207)
    text(xs[0],930,'RGB observations',25,'muted')
    status=['Start','Kettle verified','Not seen','Local recheck','Cushion found','Lamp verified']
    for i,(x,s) in enumerate(zip(xs,G['stages'])):
        view=image_view(f'rgb-{i}',s['name'],1600,1000,(0,0,1600,1000),(x,946,cw,cw*1000/1600),'#565E64')
        text(x+cw/2,1122,status[i],27,cols[i],600,'middle')

    # Compact legend on the far right, matching the reference panel hierarchy.
    lx,lw=2530,232
    for j,(col,label,n,check) in enumerate([('start','Starting','',False),
            ('waypoint','Subgoal','1',False),('miss','Not seen','2',False),
            ('recover','Recovered','2′',False),('goal','Final goal','3',True)]):
        y=330+j*106
        flag(lx+29,y+66,col,n,.67,check)
        text(lx+83,y+54,label,28,weight=700)
    for j in range(24):
        x=lx+17+j*3
        line([(x,918),(x+3,918)],c=color_at(j/23),width=6)
    arrow((lx+80,918),(lx+97,918),color_at(1),9)
    text(lx+112,928,'Path',28,weight=700)
    if not publication:
        text(lx+lw/2,1058,'Illustrative',26,'muted',anchor='middle')
        text(lx+lw/2,1090,'execution',26,'muted',anchor='middle')
    add('</g></svg>')
    return ''.join(c.S)

def render(svg,path,scale=1):
    h=Rsvg.Handle.new_with_flags(Rsvg.HandleFlags(1));h.write(svg.encode());h.close()
    surf=cairo.ImageSurface(cairo.FORMAT_ARGB32,round(W*scale),round(H*scale))
    ctx=cairo.Context(surf);ctx.scale(scale,scale)
    vp=Rsvg.Rectangle();vp.x=vp.y=0;vp.width=W;vp.height=H
    h.render_document(ctx,vp);surf.write_to_png(str(path))

if __name__=='__main__':
    svg=build();render(svg,OUT/'fig8_household_check_preview.png')
    render(svg,Path('/tmp/fig8_household_check_review.png'),.7)
    (OUT/'layout_notes.json').write_text(json.dumps(dict(
        preview_only=True,reference='BSC-Nav Fig. 4c',reference_url='https://www.nature.com/articles/s41467-026-74358-5/figures/4',
        layout='Top instruction band; left RGB floor and route; upper subgoal cues; shared flag axis; aligned native Habitat top-down maps and RGB; right legend.',
        scene='00337-CFVBbU9Rsyb, floor 2',rgb='1600 × 1000 CPU Blender renders of original HM3D mesh and RGB textures at authored poses, 74-degree horizontal FoV; no generative retouching.',
        map=G['map'],route=G['sequence_role'],qwen=G['qwen_role'],
        target_cues='Illustrative crops of original-scene RGB renders, not generated predictions or recorded memory retrieval.',
        official_plate='Crop (0, 1266, 1280, 2382) from PRE_MAP_VLN/docs/assets/system_overview/official_scene/topdown_floors.png; floor 2.',
        curve_checks=G['curve_checks'],registration=G['registration'],
        final_pose_distance_to_earlier_route_m=G['final_pose_distance_to_earlier_route_m'],
        route_length_m=G['route_length_m'],task_semantics=G['task_semantics'],labels=labels
    ),ensure_ascii=False,indent=2)+'\n')
    (OUT/'caption.txt').write_text(
        'Fig. 8. Household object verification with semantic recovery. The agent first checks the kettle on the kitchen counter, then searches the dining chairs for a cushion. After local reinspection does not reveal the cushion, semantic recovery proposes the sofa in the eastern living room as an alternative support surface. The cushion is found on the sofa, and the agent continues to verify a bedside lamp in a previously unvisited bedroom. Flags link subgoals to the smooth trajectory, cumulative path overlays on static Habitat navigability maps, and first-person observations. The official HM3D floor image and original textured scene provide the imagery; first-person views were rendered in Blender. This is an authored mechanism illustration: the task, fixed-time MINCO S4NU trajectory, verification outcomes and Qwen candidate are not a recorded experimental rollout. No color-based target selection is required.\n')
    print(OUT/'fig8_household_check_preview.png')
