"""Local frame sheets for selecting video-time hover windows."""
import argparse
from pathlib import Path
import cv2
import numpy as np

p=argparse.ArgumentParser()
p.add_argument('video')
p.add_argument('output',type=Path)
a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
c=cv2.VideoCapture(a.video)
fps=c.get(cv2.CAP_PROP_FPS)
count=int(c.get(cv2.CAP_PROP_FRAME_COUNT))
sheet=None
for s in range(int(count/fps)+1):
    c.set(cv2.CAP_PROP_POS_FRAMES,round(s*fps))
    ok,f=c.read()
    if not ok:break
    cv2.imwrite(str(a.output/f'frame_{s:03d}.jpg'),f)
    if s%20==0:
        sheet=np.zeros((4*200,5*320,3),np.uint8)
    thumb=cv2.resize(f,(320,180))
    y,x=(s%20)//5*200,(s%5)*320
    sheet[y:y+180,x:x+320]=thumb
    cv2.putText(sheet,f'{s}s',(x+8,y+195),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
    if s%20==19 or s==int(count/fps):
        cv2.imwrite(str(a.output/f'sheet_{s//20}.jpg'),sheet)
c.release()
