"""Export first28seconds of original40s tracks as synchronized40s/30fps."""
from pathlib import Path
import argparse,subprocess,cv2,json
p=argparse.ArgumentParser();p.add_argument('video_dir',type=Path);a=p.parse_args();root=a.video_dir
pairs=[('01_俯视图_无字_30fps.mp4','accepted_top_40s.mp4'),('02_全局点云_无字_30fps.mp4','accepted_global_40s.mp4'),('03_第一视角_无字_30fps.mp4','accepted_rgb_40s.mp4')]
if (root/'full40s.mp4').exists():
 pairs.append(('full40s.mp4','exploration_accepted_baseline_40s.mp4'))
for source,target in pairs:
 cap=cv2.VideoCapture(str(root/source));assert cap.isOpened(),source
 w=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH));h=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT));assert cap.get(cv2.CAP_PROP_FPS)==30
 enc=subprocess.Popen(['gst-launch-1.0','-q','fdsrc','!','videoparse','format=rgb',f'width={w}',f'height={h}','framerate=30/1','!','videoconvert','!','x264enc','speed-preset=veryfast','bitrate=9000','!','h264parse','!','mp4mux','!','filesink','location='+str(root/target)],stdin=subprocess.PIPE)
 current=-1;frame=None
 try:
  for i in range(1200):
   wanted=min(839,i*840//1200)
   while current<wanted:
    ok,frame=cap.read();assert ok;current+=1
   enc.stdin.write(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB).tobytes())
 finally:cap.release();enc.stdin.close()
 assert enc.wait()==0
 print(target,flush=True)
