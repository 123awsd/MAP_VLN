#!/usr/bin/env python3
"""Restyle cached detections, with bounded interpolation and offline smoothing."""
import argparse
from collections import Counter
from fractions import Fraction
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np


def smooth_boxes(boxes, max_gap=9, radius=5, lock_size=False):
    """Fill short interior gaps only; smooth each finite segment independently."""
    result = np.array(boxes, dtype=float, copy=True)
    valid = np.flatnonzero(np.isfinite(result).all(axis=1))
    for a, b in zip(valid, valid[1:]):
        if 1 < b-a <= max_gap+1:
            for i in range(a+1, b):
                result[i] = result[a] + (result[b]-result[a]) * ((i-a)/(b-a))
    finite = np.isfinite(result).all(axis=1)
    start = 0
    while start < len(result):
        if not finite[start]:
            start += 1
            continue
        end = start+1
        while end < len(result) and finite[end]:
            end += 1
        raw = result[start:end].copy()
        # Median rejects one-frame box-size spikes; symmetric weighted averaging
        # has no causal lag. Never crosses a long gap or a hover boundary.
        median = np.array([np.median(raw[max(0,i-2):min(len(raw),i+3)], axis=0)
                           for i in range(len(raw))])
        for i in range(len(raw)):
            lo, hi = max(0,i-radius), min(len(raw),i+radius+1)
            weights = np.exp(-0.5*((np.arange(lo,hi)-i)/max(1,radius/2))**2)
            result[start+i] = np.average(median[lo:hi], axis=0, weights=weights)
        start = end
    if lock_size:
        finite = np.isfinite(result).all(axis=1)
        if finite.any():
            centers = (result[finite, :2] + result[finite, 2:]) / 2
            detected = np.isfinite(boxes).all(axis=1)
            sizes = boxes[detected, 2:] - boxes[detected, :2]
            if not len(sizes):
                sizes = result[finite, 2:] - result[finite, :2]
            fixed_size = np.median(sizes, axis=0)
            result[finite, :2] = centers - fixed_size / 2
            result[finite, 2:] = centers + fixed_size / 2
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('detection_dir', type=Path)
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('--raw', action='store_true',
                        help='Render independent per-frame detections without filtering or gap fill')
    args = parser.parse_args()
    metadata = json.loads((args.detection_dir/'report.json').read_text())
    records = [json.loads(s) for s in (args.detection_dir/'detections.jsonl').read_text().splitlines()]
    fps = metadata['fps']
    count = metadata['source_frames']
    assert len(records) == count and all(r['frame'] == i for i,r in enumerate(records))
    overlays = {}
    stats = []
    for window in metadata['hover_window_config']['windows']:
        indices = [i for i in range(count) if window['start_s'] <= i/fps < window['end_s']]
        boxes = np.full((len(indices),4), np.nan)
        labels = []
        for n,i in enumerate(indices):
            detections = [d for d in records[i]['detections'] if d['label'] in window['labels']]
            min_area = float(window.get('min_area_px', 0))
            detections = [d for d in detections if
                          (d['bbox_xyxy'][2]-d['bbox_xyxy'][0]) *
                          (d['bbox_xyxy'][3]-d['bbox_xyxy'][1]) >= min_area]
            if detections:
                if window.get('selection') == 'largest_area':
                    item = max(detections, key=lambda d:
                               (d['bbox_xyxy'][2]-d['bbox_xyxy'][0]) *
                               (d['bbox_xyxy'][3]-d['bbox_xyxy'][1]))
                else:
                    item = max(detections, key=lambda d:d['score'])
                boxes[n] = item['bbox_xyxy']
                labels.append(item['label'])
        if not labels:
            continue
        label = Counter(labels).most_common(1)[0][0]
        smoothed = boxes.copy() if args.raw else smooth_boxes(
            boxes, max_gap=round(0.3*fps), radius=5, lock_size=True)
        for n,i in enumerate(indices):
            if np.isfinite(smoothed[n]).all():
                overlays[i] = dict(label=label, bbox_xyxy=smoothed[n].tolist(),
                                   interpolated=not bool(np.isfinite(boxes[n]).all()))
        common = np.isfinite(boxes).all(axis=1)
        pairs = common[1:] & common[:-1]
        raw_jitter = np.linalg.norm(np.diff(boxes,axis=0)[pairs],axis=1)
        smooth_jitter = np.linalg.norm(np.diff(smoothed,axis=0)[pairs],axis=1)
        raw_sizes = boxes[common, 2:] - boxes[common, :2]
        smooth_sizes = smoothed[np.isfinite(smoothed).all(axis=1), 2:] - smoothed[np.isfinite(smoothed).all(axis=1), :2]
        stats.append(dict(label=label, raw_detected_frames=int(common.sum()),
                          displayed_frames=int(np.isfinite(smoothed).all(axis=1).sum()),
                          raw_box_step_px=float(np.mean(raw_jitter)),
                          smooth_box_step_px=float(np.mean(smooth_jitter)),
                          raw_width_height_std_px=np.std(raw_sizes, axis=0).tolist(),
                          smooth_width_height_std_px=np.std(smooth_sizes, axis=0).tolist()))
    args.output_dir.mkdir(parents=True, exist_ok=False)
    cap = cv2.VideoCapture(metadata['input'])
    width,height = metadata['width'],metadata['height']
    rate = Fraction(fps).limit_denominator(100000)
    output = args.output_dir/'rgb_detected_smooth.mp4'
    encoder = subprocess.Popen([
        '/usr/bin/gst-launch-1.0','-q','fdsrc','fd=0',f'blocksize={width*height*3}','!',
        'videoparse',f'width={width}',f'height={height}','format=bgr',
        f'framerate={rate.numerator}/{rate.denominator}','!','videoconvert','!',
        'video/x-raw,format=I420','!','x264enc','pass=qual','quantizer=18',
        'speed-preset=fast','!','h264parse','!','mp4mux','faststart=true','!',
        'filesink',f'location={output}'],stdin=subprocess.PIPE)
    snapshots = {round((w['start_s']+w['end_s'])*fps/2) for w in metadata['hover_window_config']['windows']}
    try:
        for i in range(count):
            ok,frame = cap.read()
            if not ok:
                raise RuntimeError(f'Source decode failed at {i}')
            if i in overlays:
                d = overlays[i]
                x1,y1,x2,y2 = np.rint(d['bbox_xyxy']).astype(int)
                green=(0,255,0)
                cv2.rectangle(frame,(x1,y1),(x2,y2),green,5,cv2.LINE_AA)
                pos=(x1,max(25,y1-12))
                cv2.putText(frame,d['label'],pos,cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,0),5,cv2.LINE_AA)
                cv2.putText(frame,d['label'],pos,cv2.FONT_HERSHEY_SIMPLEX,0.8,green,2,cv2.LINE_AA)
            if i in snapshots:
                cv2.imwrite(str(args.output_dir/f'preview_{i}.jpg'),frame)
            encoder.stdin.write(frame.tobytes())
    finally:
        cap.release()
        encoder.stdin.close()
        if encoder.wait(timeout=60) != 0:
            raise RuntimeError('Encoder failed')
    check = cv2.VideoCapture(str(output))
    decoded=0
    while check.read()[0]:
        decoded+=1
    check.release()
    assert decoded==count
    for i,d in overlays.items():
        assert any(w['start_s']<=i/fps<w['end_s'] and d['label'] in w['labels']
                   for w in metadata['hover_window_config']['windows'])
    report=dict(source_video=metadata['input'], source_detections=str(args.detection_dir.resolve()),
                color_bgr=[0,255,0],line_width_px=5,confidence_text=False,
                smoothing=('none; independent raw per-frame detector boxes; no interpolation or gap fill'
                           if args.raw else
                           'Fixed robust median width/height per hover; center uses 5-frame median then symmetric 11-frame Gaussian; max 0.3s interior gap fill; no boundary extrapolation'),
                note=('Independent raw model detections; missed frames remain unboxed'
                      if args.raw else
                      'Display-smoothed/interpolated boxes, not new independent detections'),
                fps=fps,frames=decoded,width=width,height=height,duration_s=decoded/fps,windows=stats)
    (args.output_dir/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.output_dir/'display_boxes.json').write_text(json.dumps(overlays)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
