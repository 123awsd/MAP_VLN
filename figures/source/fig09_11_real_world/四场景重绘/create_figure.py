from pathlib import Path
import subprocess, fitz
from PIL import Image
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[4]
VIDEO=ROOT/'视频/视频制作/整理汇总/原子素材/实验视频/第三视角'
FF='/home/uav/.local/lib/python3.13/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
# Coordinates refer to the 480 x 270 inspection frames; boxes indicate only identifiable targets.
rows=[
('Dormitory: four-task mission',['First check whether the kitchen pot is on, then check the bottled water under the bedroom table and the toilet paper by the toilet.','Check for the cup on the living-room refrigerator; if absent, search the living-room table.'], '宿舍第三视角拼接.mp4',[(10.4,(228,112,25),[(0,148,101,254)]),(22.9,(184,134,23),[(220,129,234,142)]),(31.2,(234,61,35),[]),(43.0,(200,77,32),[])]),
('Laboratory: three-task mission',['Check for the cup on the lounge table and the basketball under the chair in the activity room;','inspect whether the trash can next to the fan in the storage room is full.'], '实验室三任务短距离第三视角.mp4',[(20.2,(196,114,21),[]),(28.3,(240,161,23),[(111,103,125,133)]),(60.5,(218,160,21),[(103,182,135,214)]),(84.8,(205,130,20),[(302,128,391,236)])]),
('Laboratory: pool-table mission',['Find the ball on the pool table.'], '实验室台球桌.mp4',[(7.2,(232,118,19),[]),(26.5,(241,121,18),[]),(41.0,(207,127,18),[]),(50.6,(239,153,20),[])]),
('Laboratory: table mission',['Find the cup on the table.'], '实验室桌子.mp4',[(10.5,(241,136,20),[]),(16.5,(255,148,18),[]),(22.5,(247,164,17),[]),(31.5,(273,201,22),[])])]
doc=fitz.open(); page=doc.new_page(width=780,height=824)
black=(.08,.08,.08); green=(0,.72,.27); yellow=(1,.94,0)
for ri,(title,lines,video,panels) in enumerate(rows):
 y=12+ri*202
 page.insert_text((18,y+13),title,fontsize=12,fontname='hebo',color=black)
 for k,line in enumerate(lines): page.insert_text((18,y+27+k*11),'Instruction: '+line if k==0 else line,fontsize=8.5,fontname='helv',color=black)
 for j,(t,(cx,cy,r),boxes) in enumerate(panels):
  img=BASE/'frames'/f'{ri}_{j}_{t}.jpg'
  if not img.exists(): subprocess.run([FF,'-hide_banner','-loglevel','error','-ss',str(t),'-i',str(VIDEO/video),'-frames:v','1','-q:v','2','-y',str(img)],check=True,stdin=subprocess.DEVNULL)
  x=18+j*188; top=y+51; w=180; h=135; s=w/360
  # Preserve the original aspect ratio while removing wide-angle margins.
  left=[[0,60,0,60],[35,45,35,100],[55,60,45,55],[60,65,65,35]][ri][j]
  original=Image.open(img); scale=original.width/480
  crop=original.crop((int(left*scale),0,int((left+360)*scale),int(270*scale)))
  cropped=BASE/'frames'/f'crop_{ri}_{j}_{t}.jpg'; crop.save(cropped,quality=97,subsampling=0)
  page.insert_image(fitz.Rect(x,top,x+w,top+h),filename=str(cropped))
  cx-=left
  shape=page.new_shape(); shape.draw_circle(fitz.Point(x+cx*s,top+cy*s),r*s); shape.finish(color=yellow,width=1.6,dashes='[4 2] 0'); shape.commit()
  for a,b,c,d in boxes: page.draw_rect(fitz.Rect(x+(a-left)*s,top+b*s,x+(c-left)*s,top+d*s),color=green,width=2)
page.insert_text((18,818),'Yellow dashed circle: UAV     Green box: visible target',fontsize=8,fontname='helv',color=black)
out=BASE/'real_world_four_scenes.pdf'; doc.save(out,deflate=True); page.get_pixmap(matrix=fitz.Matrix(2,2),alpha=False).save(BASE/'real_world_four_scenes.png')
print(out)
