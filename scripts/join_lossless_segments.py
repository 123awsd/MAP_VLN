"""Join the three isolated RViz captures without timestamp loss at AVI boundaries."""
from pathlib import Path
import cv2

root = Path('/workspace/shared/outputs/video_previews')
inputs = [
    root/'exploration_virtual_seg1b'/'preview_lossless.avi',
    root/'exploration_virtual_seg2b'/'preview_lossless.avi',
    root/'exploration_virtual_seg3b'/'preview_lossless.avi',
]
output = root/'exploration_real_rviz_three_views_20fps_virtual_joined.avi'
writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*'FFV1'), 20, (1920,1080))
assert writer.isOpened()
previous = None
total = 0
counts = []
for segment_index, path in enumerate(inputs):
    cap = cv2.VideoCapture(str(path)); count = 0
    while True:
        ok, frame = cap.read()
        if not ok: break
        # A newly created RViz process needs one frame to populate all displays.
        # Holding the preceding frame for 50 ms makes the join visually continuous.
        if segment_index and count < 4 and previous is not None:
            frame = previous.copy()
        writer.write(frame); previous = frame; count += 1; total += 1
    cap.release(); counts.append(count)
writer.release()
assert counts == [334,333,334], counts
assert total == 1001, total
print(output, counts, total)
