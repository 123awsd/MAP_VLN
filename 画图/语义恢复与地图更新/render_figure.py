#!/usr/bin/env python3
"""Visual recovery rules using abstract regions, view frusta and observation symbols."""
from pathlib import Path
import os,json
os.environ.setdefault('MPLCONFIGDIR','/tmp/recovery_rules_mpl')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Circle,Rectangle,FancyArrowPatch,PathPatch
from matplotlib.path import Path as MplPath
from matplotlib.colors import to_rgb
from mechanism_audit import audit

OUT=Path(__file__).resolve().parent
W,H=88,45
INK='#2B404D'; MUTED='#718690'; BLUE='#419DC8'; VIOLET='#9277CF'; GREEN='#109C78'; MISS='#D47860'; GRAY='#A8BAC3'
plt.rcParams.update({'font.family':'Liberation Sans','font.size':6.5,'pdf.fonttype':42,'svg.fonttype':'none','lines.solid_capstyle':'round','lines.solid_joinstyle':'round'})

def tint(c,w=.25):return tuple(1-w+w*v for v in to_rgb(c))
def label(ax,x,y,s,size=6.5,c=INK,ha='center',bold=False,**kw):
    return ax.text(x,y,s,fontsize=size,color=c,ha=ha,va='center',weight='bold' if bold else 'normal',zorder=30,**kw)
def line(ax,pts,c=MUTED,lw=.6,z=10,**kw):return ax.plot(*zip(*pts),color=c,lw=lw,zorder=z,**kw)
def polygon(ax,pts,fc,ec='none',lw=.6,z=3,**kw):
    a=Polygon(pts,closed=True,fc=fc,ec=ec,lw=lw,zorder=z,**kw);ax.add_patch(a);return a
def bezier(ax,pts,c=INK,lw=.85,arrow=True,ls='-'):
    path=MplPath(pts,[MplPath.MOVETO]+[MplPath.CURVE4]*(len(pts)-1))
    if arrow:a=FancyArrowPatch(path=path,arrowstyle='-|>',mutation_scale=4.5,color=c,lw=lw,linestyle=ls,zorder=8)
    else:a=PathPatch(path,fc='none',ec=c,lw=lw,linestyle=ls,zorder=8)
    ax.add_patch(a)
def arrow(ax,a,b,c=MUTED,lw=.7):ax.add_patch(FancyArrowPatch(a,b,arrowstyle='-|>',mutation_scale=4.5,color=c,lw=lw,zorder=10))
def cross(ax,x,y,s=.65):
    line(ax,[(x-s,y-s),(x+s,y+s)],MISS,1.0,20);line(ax,[(x-s,y+s),(x+s,y-s)],MISS,1.0,20)
def check(ax,x,y,s=.75):line(ax,[(x-s,y),(x-.15*s,y-.65*s),(x+1.3*s,y+s)],GREEN,1.1,21)

def frustum(ax,pos,target,c=BLUE,active=True,length=5.8):
    a=np.array(pos,float);d=np.array(target,float)-a;d/=np.linalg.norm(d);n=np.array([-d[1],d[0]])
    far=a+length*d
    # A truncated camera pyramid with an integrated short body, without lens cartoons.
    face=[a-.45*n,a+.45*n,far+1.8*n,far-1.8*n]
    polygon(ax,face,tint(c,.18) if active else 'white',c,.7,14)
    polygon(ax,[a-.55*d-.6*n,a-.55*d+.6*n,a+.45*n,a-.45*n],c if active else 'white',c,.55,15)
    line(ax,[a+.45*n,far-1.8*n],tint(c,.65),.35,15)
    return a

def region(ax,dx,c,checked=False):
    origin=np.array([7+dx,25.]);u=np.array([9.,8.]);v=np.array([14.,-3.])
    corners=[origin,origin+u,origin+u+v,origin+v]
    polygon(ax,corners,tint(c,.10 if checked else .13),tint(c,.6),.7,2)
    # Discrete region samples are an abstract depiction, not a measured occupancy map.
    for s in (.25,.50,.75):
        line(ax,[origin+s*u,origin+s*u+v],tint(c,.17),.4,3)
        line(ax,[origin+s*v,origin+s*v+u],tint(c,.17),.4,3)
    for a in (.22,.5,.78):
        for b in (.22,.5,.78):
            xy=origin+a*u+b*v
            ax.add_patch(Circle(xy,.47,fc=tint(c,.38) if checked else tint(c,.8),ec='white',lw=.3,zorder=5))
    return origin+.5*(u+v)

def detection(ax,x,y,s=1,c=GREEN):
    # Four detector-box corners: a visual observation, not a semantic object icon.
    for sx in (-1,1):
        for sy in (-1,1):
            line(ax,[(x+sx*1.65*s,y+sy*.7*s),(x+sx*1.65*s,y+sy*1.65*s),(x+sx*.7*s,y+sy*1.65*s)],c,.9,19)
    ax.add_patch(Circle((x,y),.48*s,fc=c,ec='white',lw=.3,zorder=20))

def graph_delta(ax):
    root=(79,12.0)
    for pos in [(73.5,7.0),(78.5,5.8),(84.1,7.2)]:
        new=pos[0]>80
        line(ax,[root,pos],GREEN if new else GRAY,.65,8)
        ax.add_patch(Circle(pos,1.0,fc=tint(GREEN,.22) if new else '#E8EDF0',ec=GREEN if new else GRAY,lw=.65,zorder=12))
    ax.add_patch(Circle(root,1.15,fc='#E8EDF0',ec=GRAY,lw=.65,zorder=12))
    label(ax,86,9.5,'+',6.5,GREEN,bold=True)


def render(e):
    fig=plt.figure(figsize=(W/25.4,H/25.4),facecolor='white')
    ax=fig.add_axes([0,0,1,1]);ax.set(xlim=(0,W),ylim=(0,H),aspect='equal');ax.axis('off')
    label(ax,44,42.1,'Same task',6.5,MUTED)
    label(ax,18.5,37.9,'Current region',7.2,INK,bold=True)
    label(ax,62.5,37.9,'Alternative region',7.2,INK,bold=True)
    old=region(ax,0,BLUE,checked=True);new=region(ax,44,VIOLET)
    # Local failed views are shown at one geometric region, connected only by retry arrows.
    poses=[(4.8,17.8),(17,13.7),(30.1,18)]
    for pos in poses:
        frustum(ax,pos,old,BLUE)
        cross(ax,pos[0],pos[1]-1.9,.55)
    bezier(ax,[(6.8,15.2),(9.2,12.4),(12.6,11.8),(15.0,12.9)],BLUE,.85)
    bezier(ax,[(19.0,12.9),(22.9,12.0),(26.3,13.7),(28.0,15.6)],BLUE,.85)
    label(ax,16.8,7.9,'Miss → retry',6.7,BLUE,bold=True)
    # A qualitative dimming of covered samples represents decreased region belief.
    label(ax,17.9,20.7,'belief ↓',5.9,MUTED)
    # The regional transition follows exhaustion, not a single missed observation.
    bezier(ax,[(32,18.1),(39,16.3),(46.6,15.4),(52.8,16.7)],INK,.9)
    label(ax,43.2,22.0,'Exhausted',6.4,INK,bold=True)
    label(ax,43.2,12.1,'Infer + replan',6.0,MUTED)
    # Available views remain outlined; the successful selected view is filled green.
    frustum(ax,(78.8,24.0),new,GRAY,active=False,length=5.3)
    frustum(ax,(73.3,16.5),new,GRAY,active=False,length=5.3)
    frustum(ax,(55.0,17.2),new,GREEN,length=6.4)
    check(ax,55,14.9,.62)
    detection(ax,new[0],new[1],1.05)
    label(ax,66.2,34.8,'Verified',6.2,GREEN,bold=True)
    # Multi-view association and post-execution persistence are a small endpoint annotation.
    detection(ax,63,8.7,.60,c=tint(GREEN,.60))
    detection(ax,65.0,9.7,.60)
    arrow(ax,(67.6,9.6),(71.2,9.6),GREEN,.65)
    graph_delta(ax)
    label(ax,64,4.2,'Fuse views',5.9,MUTED)
    label(ax,77.5,2.3,'After execution',5.9,MUTED)
    # A short connector associates the positive observations with the evidence stream.
    line(ax,[(62.5,24.9),(62.5,13.1),(64.2,11.4)],tint(GREEN,.5),.55,7,linestyle=(0,(1.6,2)))
    fig.canvas.draw();bounds=[(t,t.get_window_extent()) for t in ax.texts]
    for t,bb in bounds:
        assert fig.bbox.contains(bb.x0,bb.y0) and fig.bbox.contains(bb.x1,bb.y1),t.get_text()
    for i,(t,bb) in enumerate(bounds):
        for t2,bb2 in bounds[i+1:]:assert not bb.overlaps(bb2),(t.get_text(),t2.get_text())
    for ext in ('pdf','svg','png'):fig.savefig(OUT/f'semantic_recovery_en.{ext}',dpi=600)
    fig.savefig(OUT/'semantic_recovery_en_preview.png',dpi=300);plt.close(fig)

if __name__=='__main__':
    evidence=audit()
    evidence['figure']={'size_mm':[W,H],'layout':'single abstract region-and-view illustration; no scene construction or stage cards',
        'semantic_object_examples_shown':False,'graphic_elements':['region samples','camera frusta','miss and verification markers','detection boxes','graph insertion'],
        'minimum_text_pt':5.9,'paper_updated_by_renderer':False,'geometry_is_illustrative':True,
        'motion_planner_executed':False,'arrows_show_search_order_not_a_flight_trajectory':True,
        'region_recovery_guard':'location exhausted and recovery permitted with feasible continuation',
        'not_shown':'task-level terminal not-found outcome; optional fallback details; conditional task activation'}
    render(evidence)
    (OUT/'mechanism_evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
