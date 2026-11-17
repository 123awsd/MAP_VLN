from pathlib import Path
from PIL import Image,ImageDraw,ImageFont,ImageOps
import base64,html,io
r=Path(__file__).resolve().parent;o=r/'single_instruction_figure';o.mkdir(exist_ok=True)
W,H=3200,1740;im=Image.new('RGB',(W,H),'white');d=ImageDraw.Draw(im)
svg=[f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="{W}" height="{H}"><rect width="100%" height="100%" fill="white"/>']
font='/usr/share/fonts/truetype/dejavu/DejaVuSerif.ttf'
def text(x,y,s,size=26,color='#222222'):
 d.text((x,y),s,font=ImageFont.truetype(font,size),fill=color);svg.append(f'<text x="{x}" y="{y+size}" font-family="DejaVu Serif" font-size="{size}" fill="{color}">{html.escape(s)}</text>')
def pic(p,x,y,w,h,crop=False):
 a=Image.open(p).convert('RGB')
 if not crop:
  from PIL import ImageChops
  box=ImageChops.difference(a,Image.new('RGB',a.size,'white')).getbbox()
  if box:a=a.crop(box)
 if crop:a=ImageOps.fit(a,(w,h),Image.Resampling.LANCZOS)
 else:
  a.thumbnail((w,h),Image.Resampling.LANCZOS);x+=(w-a.width)//2;y+=(h-a.height)//2
 im.paste(a,(x,y));b=io.BytesIO();a.save(b,format='PNG');enc=base64.b64encode(b.getvalue()).decode();svg.append(f'<image x="{x}" y="{y}" width="{a.width}" height="{a.height}" xlink:href="data:image/png;base64,{enc}"/>')
rows=[('01_laptop','FPV 00:23–00:49',['Check the laptop','on the lounge table.','If absent, find it.'],[24,35,37,44,46]),('02_toolbox','FPV 00:49–00:54',['Verify the toolbox','on the top shelf','in the storage room.'],[49,50,51,52,54]),('03_microwave','FPV 01:00–01:27',['Check whether the','corn in the tea-room','microwave has','finished heating.'],[60,67,74,82,84]),('04_cup','FPV 01:27–01:42',['Check for the cup','under the pool table','in the recreation room.'],[87,93,98,100,102])]
# One instruction spans the entire execution; rows are successive stages.
text(28,12,'SINGLE INSTRUCTION',26,'#187b72')
instruction="I'm heading out soon. Please take care of a few things before I leave. See whether my cup is under the pool table in the recreation room. See if the laptop is on the lounge table. If it's not there, please help me find it. Verify that the toolbox is on the top shelf in the storage room. Determine whether the corn in the tea room microwave has finished heating."
words=instruction.split();wrapped=[];line='';hf=ImageFont.truetype(font,30)
for word in words:
 candidate=(line+' '+word).strip()
 if d.textlength(candidate,font=hf)>3120:wrapped.append(line);line=word
 else:line=candidate
wrapped.append(line)
for j,line in enumerate(wrapped):text(28,53+j*39,line,30)
text(28,205,'Execution of a single instruction',25,'#187b72')
# A single vertical arrow links the four stages in execution order.
d.line([(37,305),(37,1540)],fill='#78a69e',width=4)
d.polygon([(37,1556),(25,1538),(49,1538)],fill='#78a69e')
svg.append('<path d="M37 305 L37 1540" fill="none" stroke="#78a69e" stroke-width="4"/><path d="M37 1556 L25 1538 L49 1538 Z" fill="#78a69e"/>')
stages=[['Laptop search','& recovery'],['Toolbox','verification'],['Microwave','inspection'],['Cup','verification']]
for i,(n,ran,lines,times) in enumerate(rows):
 y=250+i*365
 d.ellipse((29,y+145,45,y+161),fill='#187b72')
 svg.append(f'<circle cx="37" cy="{y+153}" r="8" fill="#187b72"/>')
 for j,line in enumerate(stages[i]):text(67,y+106+j*38,line,27)
 if i==0:text(67,y+202,'Semantic recovery',20,'#187b72')
 pic(r/'complementary_views'/f'{n}_selected.png',330,y,550,330)
 a={'01_laptop':23,'02_toolbox':49,'03_microwave':60,'04_cup':87}[n]
 for k,t in enumerate(times):
  x=900+k*456
  source=r/'map_supported_results'/n
  idx=(t-a)*(4 if n=='02_toolbox' else 2)
  pic(source/f'frame_{idx:03d}.jpg',x,y,448,330,True)
  if i==0 and k>=3:text(x+100,y+334,'Semantic recovery',20,'#187b72')
text(25,1718,'Pink: recorded EKF camera path   |   Green: start   |   Yellow: end   |   Mapping RGB-D + LiDAR-supported flight VGGT depth; EKF path; approximate alignment',17,'#555555')
svg.append('</svg>');im.save(o/'main_figure.png',dpi=(300,300));im.save(o/'main_figure.pdf',resolution=300);(o/'main_figure.svg').write_text('\n'.join(svg));im.resize((1600,870),Image.Resampling.LANCZOS).save(o/'preview.jpg',quality=95)
