from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import math

root=Path('/home/uav/map_VLN')
bg=root/'素材处理后/第一阶段/目标框检测/场景00337_L2_去顶预览.png'
out=root/'素材处理后/第一阶段/目标框检测/场景00337_L2_Fig7叠加预览.png'
img=Image.open(bg).convert('RGBA')
w,h=img.size
d=ImageDraw.Draw(img,'RGBA')
font_paths=['/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf','/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf']
font_path=next(p for p in font_paths if Path(p).exists())
font_bold=ImageFont.truetype(font_path,28)
font_label=ImageFont.truetype(font_path,26)
font_small=ImageFont.truetype(font_path,22)
# image coordinates chosen over four readable rooms in the HM3D top view
S=(1010,185); A=(760,330); D=(780,605); C=(340,570); B=(360,330)
# helper for outlined paths
def line(points, fill, width=12, dash=None):
    if dash is None:
        d.line(points,fill=fill,width=width,joint='curve')
        return
    for p0,p1 in zip(points[:-1],points[1:]):
        x0,y0=p0; x1,y1=p1; L=math.hypot(x1-x0,y1-y0); ux=(x1-x0)/L; uy=(y1-y0)/L
        t=0
        while t<L:
            a=t; b=min(t+dash[0],L)
            d.line([(x0+ux*a,y0+uy*a),(x0+ux*b,y0+uy*b)],fill=fill,width=width)
            t+=sum(dash)
# old route underlay and updated route
old=[A,C,B]
new=[S,A,D,C,B]
line(old,(255,255,255,210),18,(12,10)); line(old,(120,132,140,220),6,(12,10))
line(new,(255,255,255,235),22); line(new,(38,128,146,245),10)
# arrows
def arrow(p0,p1,color,width=10):
    x0,y0=p0; x1,y1=p1; t=.72; x=x0+(x1-x0)*t; y=y0+(y1-y0)*t
    ang=math.atan2(y1-y0,x1-x0); L=22
    q1=(x-L*math.cos(ang-math.pi/6),y-L*math.sin(ang-math.pi/6)); q2=(x-L*math.cos(ang+math.pi/6),y-L*math.sin(ang+math.pi/6))
    d.polygon([(x,y),q1,q2],fill=color)
for p0,p1 in zip(new[:-1],new[1:]): arrow(p0,p1,(38,128,146,245),10)
# points and labels
def node(p,label,color,fill=(255,255,255,235),r=26):
    x,y=p; d.ellipse((x-r,y-r,x+r,y+r),fill=fill,outline=color,width=5)
    bb=d.textbbox((0,0),label,font=font_label); tw=bb[2]-bb[0]; th=bb[3]-bb[1]
    d.text((x-tw/2,y-th/2-3),label,font=font_label,fill=color)
for p,label,color in [(A,'A',(125,137,146,255)),(B,'B',(38,128,146,255)),(C,'C',(38,128,146,255)),(D,'D',(229,138,74,255))]: node(p,label,color, (255,244,234,240) if label=='D' else (255,255,255,238))
d.ellipse((S[0]-13,S[1]-13,S[0]+13,S[1]+13),fill=(39,51,59,255),outline='white',width=4)
d.text((S[0]-88,S[1]-55),'Start',font=font_small,fill=(39,51,59,255))
# callout
box=(560,110,1085,172)
d.rounded_rectangle(box,radius=12,fill=(255,255,255,228),outline=(229,138,74,255),width=3)
d.text((580,126),'A: not found  →  activate D',font=font_bold,fill=(205,107,48,255))
# title and compact legend
panel=(35,25,610,88)
d.rounded_rectangle(panel,radius=12,fill=(255,255,255,220))
d.text((55,45),'Joint task–viewpoint update',font=font_bold,fill=(39,51,59,255))
legend_y=h-58
d.line((45,legend_y,115,legend_y),fill=(120,132,140,230),width=5)
d.text((128,legend_y-16),'old plan',font=font_small,fill=(100,112,120,255))
d.line((295,legend_y,365,legend_y),fill=(38,128,146,245),width=8)
d.text((378,legend_y-16),'updated plan',font=font_small,fill=(38,128,146,255))
img.convert('RGB').save(out,quality=95)
print(out)
