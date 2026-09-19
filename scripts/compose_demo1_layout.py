"""Compose demo1 with third-person main view and two evidence views."""
from pathlib import Path
import cv2
import numpy as np

root = Path('/workspace/shared/素材处理后/任务demo/demo1')
third_path = root/'01_单独视频/04_第三人称_灯带无人机_1920x1080.mp4'
first_path = root/'01_单独视频/03_第一视角_检测框_1440x1080.mp4'
global_path = root/'01_单独视频/01_全局轨迹旋转_完整轨迹_1920x1080.mp4'
out = root/'02_合并视频/demo1_third_main_layout_lossless.avi'

caps = [cv2.VideoCapture(str(p)) for p in (third_path, first_path, global_path)]
assert all(c.isOpened() for c in caps)
writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'FFV1'), 20, (1920,1080))
assert writer.isOpened()

def fit(frame, size):
    w, h = size
    return cv2.resize(frame, (w, h), interpolation=cv2.INTER_LANCZOS4)

last = None
count = 0
while True:
    frames = []
    for cap in caps:
        ok, frame = cap.read()
        if not ok:
            frames = []
            break
        frames.append(frame)
    if not frames:
        break
    canvas = np.full((1080,1920,3), (24,30,38), np.uint8)
    # Preserve 16:9 for the main and global views, and 4:3 for first person.
    canvas[180:900, 0:1280] = fit(frames[0], (1280,720))
    canvas[60:540, 1280:1920] = fit(frames[1], (640,480))
    canvas[600:960, 1280:1920] = fit(frames[2], (640,360))
    cv2.rectangle(canvas, (0,180), (1279,899), (220,220,220), 2)
    cv2.rectangle(canvas, (1280,60), (1919,539), (220,220,220), 2)
    cv2.rectangle(canvas, (1280,600), (1919,959), (220,220,220), 2)
    cv2.putText(canvas, 'THIRD-PERSON EXECUTION', (24,214), cv2.FONT_HERSHEY_SIMPLEX, .72, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(canvas, 'FIRST-PERSON DETECTION', (1302,92), cv2.FONT_HERSHEY_SIMPLEX, .55, (255,255,255), 1, cv2.LINE_AA)
    cv2.putText(canvas, 'GLOBAL TRAJECTORY', (1302,632), cv2.FONT_HERSHEY_SIMPLEX, .55, (255,255,255), 1, cv2.LINE_AA)
    writer.write(canvas); last = canvas; count += 1

# Hold the completed final state for three seconds so the ending is readable.
for _ in range(60):
    final = last.copy()
    cv2.rectangle(final, (35,930), (690,1010), (24,30,38), -1)
    cv2.putText(final, 'TASK COMPLETE  |  4 TARGETS VERIFIED', (55,982), cv2.FONT_HERSHEY_SIMPLEX, .68, (90,235,150), 2, cv2.LINE_AA)
    writer.write(final)

for cap in caps: cap.release()
writer.release()
print(out, 'frames=', count+60, 'source_frames=', count)
