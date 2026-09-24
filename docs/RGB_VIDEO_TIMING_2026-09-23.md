# RGB recording timing correction

Local implementation only; not deployed or tested on NX. No flight processes started.

## Existing laboratory demo

Session: `lab_room_v2/check_cup_basket_trashcan_07/flight_20260923_014428`.
Bag duration: 194.550951 s. MP4 duration: 97.833333 s, 2935 frames at
nominal 30 FPS. Original files remain unchanged.

The old raw-frame pipe assigned time by frame count and ignored ROS image
timestamps. Missing/slow frames therefore compressed playback time. These
durations indicate approximately 2x overall acceleration, not a proven constant
per-frame acceleration. This bag contains camera_info, but no RGB images, and
the old MP4 has no source timestamp sidecar. Exact retrospective reconstruction
is unavailable: camera_info timestamps cannot identify which images survived.
Do not label a uniform slowdown as exact real-time recovery.

## New recordings

`record_ros_rgb_video.py` uses `timestamped_rgb_encoder.py`: camera
`Image.header.stamp` relative to the first accepted frame becomes MP4 PTS.
No frame-rate conversion is applied. Missing frames leave time gaps (the player
holds the preceding image); they do not shorten the timeline. Resolution and
bitrate are unchanged. The last sample has one nominal frame period because the
next source timestamp is unknown. The recording covers first through last received
image, not necessarily the entire bag interval.

`*.mp4.timestamps.csv` stores each submitted frame index, absolute source
timestamp in nanoseconds, and relative PTS. Invalid/duplicate/backward source
timestamps fail recording explicitly. Existing output files are not overwritten.
The sidecar records submissions; encoder EOS must succeed before claiming a sealed
MP4. No recovery of unreceived images is possible.

New dependency: the recording Python interpreter must have PyGObject (`gi`) and
GStreamer introspection plus appsrc, videoconvert, x264enc, h264parse, mp4mux and
filesink. The flight bag wrapper checks these before starting the camera. NX
deployment must include both Python files and the wrapper, check dependencies,
and perform a sensor-only timed recording before flight use. Do not change flight
controls or mission limits for this fix.

## Verification

Run `/usr/bin/python3 -m unittest discover -s tests -p test_rgb_video_timing.py -v`.
Two tests passed locally. The integration test really encodes and parses MP4
sample timing: four irregularly spaced frames spanning 1 s produce 1.033333 s,
not 0.133333 s; sidecar stamps match. Invalid clocks and overwrite refusal are
also checked. This does not certify NX performance or its camera clock accuracy.
