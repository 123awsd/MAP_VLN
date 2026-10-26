from pathlib import Path
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch, Polygon

out = Path('paper/tase_submission/figures')
out.mkdir(parents=True, exist_ok=True)
fig, ax = plt.subplots(figsize=(13, 8), dpi=180)
ax.set_aspect('equal'); ax.axis('off'); ax.set_xlim(-6.6, 6.6); ax.set_ylim(-4.4, 4.4)

# palette
ink='#243247'; muted='#9AA7B8'; light='#DCE6F2'; teal='#0E7490'; blue='#2563EB'; orange='#F59E0B'; red='#DC2626'; green='#16A34A'

def label(x,y,s,fs=10,c=ink,ha='center',weight='normal'):
    ax.text(x,y,s,fontsize=fs,color=c,ha=ha,va='center',fontweight=weight)
def arrow(x1,y1,x2,y2,c=ink,style='-|>',lw=1.5,alpha=1):
    ax.add_patch(FancyArrowPatch((x1,y1),(x2,y2),arrowstyle=style,mutation_scale=12,lw=lw,color=c,alpha=alpha))

# Title and flow header
label(0,4.05,'Task-conditioned candidate viewpoint generation',18,ink,weight='bold')
label(0,3.62,'One scene shows the complete process: define the region → sample → validate → retain complementary views',10,muted)

# central scene top-down
label(-1.2,3.15,'① Define observation region',11,orange,weight='bold')
# table
ax.add_patch(Rectangle((-3.1,-1.65),3.8,2.5,facecolor='#E7B77B',edgecolor='#8B5E34',lw=2,alpha=.85))
# legs
for x in (-2.75,0.15):
    for y in (-1.42,.55): ax.add_patch(Rectangle((x,y),.18,.18,fc='#7C4A2E',ec='none'))
label(-1.2,-.3,'table anchor',10,'#6B4328')
# region grid under table (display on floor)
for x in (-2.4,-1.2,0.0):
    for y in (-2.35,-2.8,-3.25):
        ax.plot(x,y,'o',ms=5,color=orange)
label(-1.2,-3.62,'below-region samples  Pᵢ  (3×3)',9,orange)
# rings around region center
for r,alpha in [(3.35,.28),(2.65,.38),(2.05,.48)]:
    ax.add_patch(Circle((-1.2,-2.78),r,fill=False,ec=muted,lw=1.2,ls=(0,(4,3)),alpha=alpha))
label(2.25,1.55,'② Sample three\nstand-off rings',11,teal,weight='bold')
# candidate dots around rings, only representative dots
for r,c in [(3.35,muted),(2.65,muted),(2.05,muted)]:
    for deg in range(0,360,45):
        a=math.radians(deg); x=-1.2+r*math.cos(a); y=-2.78+r*math.sin(a)
        ax.plot(x,y,'o',ms=3.3,color=c,alpha=.85)
# chosen height example on right
x,y=-1.2+2.65*math.cos(math.radians(30)),-2.78+2.65*math.sin(math.radians(30))
ax.plot(x,y,'o',ms=8,color=blue); arrow(x,y,x-.2,y+.8,blue,lw=1.2)
label(x+.65,y+.65,'③ FoV gives\nfeasible z',10,blue,weight='bold',ha='left')
# obstacle and rejected candidates
ax.add_patch(Rectangle((1.1,-1.55),.75,1.4,fc='#6B7280',ec=ink,lw=1.2,alpha=.9))
label(1.48,-.05,'obstacle',8,'#4B5563')
# reject near obstacle
ax.plot(1.0,-.9,'o',ms=8,mec=red,mfc='white',mew=2); label(1.0,-.9,'×',14,red,weight='bold')
# blocked LOS example
ax.plot(3.2,-1.2,'o',ms=8,mec=red,mfc='white',mew=2); arrow(3.2,-1.2,.1,-2.45,red,lw=1.2,alpha=.7)
label(3.65,-.45,'④ Reject\ninvalid views',10,red,weight='bold',ha='left')
# selected blue views around perimeter
for deg in (150,225,315):
    a=math.radians(deg); x=-1.2+2.65*math.cos(a); y=-2.78+2.65*math.sin(a)
    ax.plot(x,y,'o',ms=8,color=blue); arrow(x,y,x+(0.25*math.cos(a+math.pi)),y+(0.25*math.sin(a+math.pi)),blue,lw=1.1)
# retain bracket and note
label(4.4,-2.55,'⑤ Keep\nrepresentative\n& diverse views',10,green,weight='bold',ha='left')
for deg in (150,225,315):
    a=math.radians(deg); x=-1.2+2.65*math.cos(a); y=-2.78+2.65*math.sin(a)
    arrow(x,y,4.0,-2.55,green,lw=.8,alpha=.25)
# compact legend bottom
ax.plot([],[],'o',color=orange,label='region samples'); ax.plot([],[],'o',color=muted,label='initial candidates'); ax.plot([],[],'o',color=blue,label='retained views'); ax.plot([],[],'o',mec=red,mfc='white',mew=2,label='rejected')
ax.legend(loc='lower left',bbox_to_anchor=(-.01,-.01),frameon=False,ncol=4,fontsize=8)
# callout equations
label(-5.95,1.8,'Distance rings:',9,ink,ha='left',weight='bold')
label(-5.95,1.48,'D(η)=R / sin(η·αₕ/2)',10,teal,ha='left')
label(-5.95,1.12,'η = 1, 0.75, 0.5',9,muted,ha='left')
label(-5.95,.72,'Each horizontal sample gets\nits own feasible height z.',9,blue,ha='left')
# arrows from callout to ring
arrow(-4.45,1.48,-3.5,.55,teal,lw=1)
fig.savefig(out/'viewpoint_generation_concept_v1.png',bbox_inches='tight',facecolor='white')
fig.savefig(out/'viewpoint_generation_concept_v1.pdf',bbox_inches='tight',facecolor='white')
fig.savefig(out/'viewpoint_generation_concept_v1.svg',bbox_inches='tight',facecolor='white')
print(out/'viewpoint_generation_concept_v1.png')
