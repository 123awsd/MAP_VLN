from pathlib import Path
from PIL import Image,ImageDraw,ImageFont,ImageOps
import base64,html,io
r=Path(__file__).resolve().parent;o=r/'exact_figure';o.mkdir(exist_ok=True)
W,H=3200,1500;im=Image.new('RGB',(W,H),'white');d=ImageDraw.Draw(im)
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
rows=[('01_laptop','00:15–00:53',['Check the laptop','on the lounge table.','If absent, find it.'],[19,31,37,45,52]),('02_toolbox','00:53–01:09',['Verify the toolbox','on the top shelf','in the storage room.'],[53,59,63,66,68]),('03_microwave','01:09–02:09',['Check whether the','corn in the tea-room','microwave has','finished heating.'],[69,87,103,121,128]),('04_cup','02:09–02:43',['Check for the cup','under the pool table','in the recreation room.'],[129,139,149,159,162])]
for i,(n,ran,lines,times) in enumerate(rows):
 y=18+i*365;text(115,y+5,str(i+1),48);text(35,y+69,ran,23,'#666666')
 for j,line in enumerate(lines):text(25,y+123+j*37,line,24)
 if i==0:text(25,y+260,'Recovery at 00:37',22,'#187b72')
 pic(r/'exact_views'/f'{n}_selected.png',330,y,550,330)
 a={'01_laptop':15,'02_toolbox':53,'03_microwave':69,'04_cup':129}[n]
 for k,t in enumerate(times):
  x=900+k*456;pic(r/'exact_results'/n/f'frame_{t-a:03d}.jpg',x,y,448,330,True)
  text(x+7,y+334,f'{t//60:02d}:{t%60:02d}',20)
  if i==0 and k>=2:text(x+100,y+334,'Semantic recovery',20,'#187b72')
text(25,1480,'',12)
svg.append('</svg>');im.save(o/'main_figure.png',dpi=(300,300));im.save(o/'main_figure.pdf',resolution=300);(o/'main_figure.svg').write_text('\n'.join(svg));im.resize((1600,750),Image.Resampling.LANCZOS).save(o/'preview.jpg',quality=95)
