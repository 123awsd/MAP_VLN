from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch, Circle
from matplotlib.lines import Line2D
from matplotlib.path import Path as MplPath
from matplotlib.patches import PathPatch

OUT = Path(__file__).parent
W_MM, H_MM = 89, 98
fig = plt.figure(figsize=(W_MM/25.4, H_MM/25.4), facecolor="white")
gs = fig.add_gridspec(2, 1, height_ratios=[1.45, .78], hspace=.26,
                      left=.08, right=.96, top=.965, bottom=.055)

INK = "#27343B"
MUTED = "#7C8790"
WALL = "#A9B4BB"
GRID = "#E7ECEF"
TEAL = "#2C8192"
ORANGE = "#E58A4A"
LIGHT = "#F6F8F9"

# --- top: compact task-order update map ---
ax = fig.add_subplot(gs[0])
ax.set_aspect("equal")
ax.set_xlim(0, 10); ax.set_ylim(0, 7.2)
ax.axis("off")
ax.text(0, 7.08, "Task-order update", fontsize=8.1, fontweight="bold", color=INK, va="top")
ax.text(10, 7.08, "not found → replan", fontsize=6.0, color=MUTED, ha="right", va="top")

# room layout
rooms = [(0.35, 3.65, 4.35, 3.0, "B"), (4.70, 3.65, 4.95, 3.0, "A"),
         (0.35, 0.45, 4.35, 3.0, "C"), (4.70, 0.45, 4.95, 3.0, "D")]
for x,y,w,h,label in rooms:
    ax.add_patch(Rectangle((x,y),w,h,facecolor="#FAFBFC",edgecolor=WALL,lw=1.0))
    ax.text(x+.20,y+h-.23,f"Room {label}",fontsize=5.8,color="#75838B",va="top")
    # subtle floor grid
    for gx in np.arange(x+.45,x+w,.55): ax.plot([gx,gx],[y+.08,y+h-.08],color=GRID,lw=.35,zorder=0)
    for gy in np.arange(y+.4,y+h,.55): ax.plot([x+.08,x+w-.08],[gy,gy],color=GRID,lw=.35,zorder=0)

anchors = {"A":(7.05,5.05),"B":(2.15,5.05),"C":(2.15,1.85),"D":(7.05,1.85)}
for k,(x,y) in anchors.items():
    col = ORANGE if k=="D" else (MUTED if k=="A" else TEAL)
    fc = "#FFF4EA" if k=="D" else "white"
    ax.add_patch(Circle((x,y),.22,facecolor=fc,edgecolor=col,lw=1.25,zorder=5))
    ax.text(x,y,k,fontsize=6.3,color=col,ha="center",va="center",fontweight="bold",zorder=6)

# Start and route points
start=(8.85,5.85); a=(7.05,5.05); d=(7.05,1.85); c=(2.15,1.85); b=(2.15,5.05)
ax.plot(*start, marker="o", ms=3.5, color=INK, zorder=7)
ax.text(start[0]-.05,start[1]+.25,"Start",fontsize=5.7,color=INK,ha="right")
# old plan, only remaining old segment emphasized
ax.plot([a[0],c[0],b[0]],[a[1],c[1],b[1]],ls=(0,(2.2,2.0)),color="#AAB4BA",lw=1.0,zorder=2)
# executed prefix and updated plan
ax.plot([start[0],a[0]],[start[1],a[1]],color=TEAL,lw=1.8,solid_capstyle="round",zorder=3)
ax.plot([a[0],d[0],c[0],b[0]],[a[1],d[1],c[1],b[1]],color=TEAL,lw=1.8,solid_capstyle="round",zorder=3)
# arrowheads on updated route
for p0,p1 in [(start,a),(a,d),(d,c),(c,b)]:
    ax.add_patch(FancyArrowPatch(p0,p1,arrowstyle="-|>",mutation_scale=7,color=TEAL,lw=0.0,zorder=4))
# checked A and activation note
ax.add_patch(Circle(a,.28,facecolor="white",edgecolor=MUTED,lw=1.1,zorder=6))
ax.text(*a,"A",fontsize=6.3,color=MUTED,ha="center",va="center",fontweight="bold",zorder=7)
ax.text(7.55,3.48,"A: not found",fontsize=6.1,color=ORANGE,fontweight="bold",ha="left")
ax.plot([7.48,7.10],[3.38,2.2],color=ORANGE,lw=.8)
ax.text(5.08,.10,"old: A → C → B",fontsize=5.5,color=MUTED)
ax.text(7.25,.10,"new: D → C → B",fontsize=5.5,color=TEAL)

# --- bottom: path smoothing ---
bx = fig.add_subplot(gs[1])
bx.set_xlim(0,10); bx.set_ylim(0,3.25); bx.axis("off")
bx.text(0,3.15,"Trajectory smoothing",fontsize=8.1,fontweight="bold",color=INK,va="top")
# corridor and axes-like frame
bx.add_patch(Rectangle((.15,.45),9.7,2.15,facecolor=LIGHT,edgecolor="#D5DDE1",lw=.8))
# simple obstacle blocks
for rect in [(3.15,1.05,.58,.85),(5.65,1.55,.62,.75)]:
    bx.add_patch(Rectangle((rect[0],rect[1]),rect[2],rect[3],facecolor="white",edgecolor="#B8C3C9",lw=.75))
# discrete polyline
poly=np.array([[.75,.72],[2.2,.72],[3.15,.72],[3.15,1.95],[4.65,1.95],[5.65,1.95],[5.65,2.35],[8.95,2.35]])
bx.plot(poly[:,0],poly[:,1],color="#8B959C",lw=1.25,ls=(0,(2.2,1.8)),zorder=3)
# smooth curve
verts=[(.75,.72),(1.7,.72),(2.55,.77),(3.35,1.62),(4.4,2.05),(5.35,1.62),(6.15,2.17),(7.45,2.35),(8.95,2.35)]
codes=[MplPath.MOVETO]+[MplPath.CURVE3]*(len(verts)-1)
path=MplPath(verts,codes)
bx.add_patch(PathPatch(path,facecolor="none",edgecolor=TEAL,lw=1.9,capstyle="round",joinstyle="round",zorder=4))
for p,lab,col,ha in [((.75,.72),"S",INK,"center"),((8.95,2.35),"A",TEAL,"center")]:
    bx.plot(*p,marker="o",ms=3.7,color=col,zorder=5)
    bx.text(p[0],p[1]-.27,lab,fontsize=6.0,color=col,ha=ha)
bx.text(1.0,.47,"discrete path",fontsize=5.5,color="#7B858B",va="bottom")
bx.text(7.15,.47,"smooth trajectory",fontsize=5.5,color=TEAL,va="bottom")
bx.text(9.7,2.78,"MINCO-based",fontsize=5.5,color=MUTED,ha="right")

# tiny legend separator
fig.savefig(OUT/"joint_task_viewpoint_planning_halfcol_v1.pdf",bbox_inches="tight",pad_inches=.02)
fig.savefig(OUT/"joint_task_viewpoint_planning_halfcol_v1.svg",bbox_inches="tight",pad_inches=.02)
fig.savefig(OUT/"joint_task_viewpoint_planning_halfcol_v1.png",dpi=500,bbox_inches="tight",pad_inches=.02)
