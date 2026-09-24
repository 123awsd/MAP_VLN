#!/usr/bin/env python3
"""Offline per-frame detection overlay; never infer mission completion from boxes."""
import argparse
import collections
import json
from pathlib import Path
import time
import subprocess
from fractions import Fraction

import cv2
import torch
from PIL import Image
from transformers import GroundingDinoForObjectDetection, GroundingDinoProcessor
from torchvision.ops import nms


def target_label(phrase):
    """Allow synonymous drink-container phrases, reject mixed background labels."""
    if any(word in phrase for word in ('chair', 'table', 'fan')):
        return None
    groups = []
    if 'cooker' in phrase or 'steamer' in phrase:
        groups.append('cooker')
    if 'tissue' in phrase or 'toilet paper' in phrase:
        groups.append('tissues')
    if any(word in phrase for word in ('cup', 'mug', 'bottle', 'thermos')):
        groups.append('thermos' if 'thermos' in phrase else 'bottle' if 'bottle' in phrase else 'cup')
    if 'basketball' in phrase:
        groups.append('basketball')
    if 'trash can' in phrase:
        groups.append('trash can')
    return groups[0] if len(groups) == 1 else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=Path)
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('--threshold', type=float, default=0.30)
    parser.add_argument('--model', default='IDEA-Research/grounding-dino-base')
    parser.add_argument('--windows', type=Path,
                        help='Reviewed source-video hover intervals and allowed target labels')
    parser.add_argument('--sample-step', type=int, default=1,
                        help='Use 1 for final video; larger values produce a diagnostic sample only')
    args = parser.parse_args()
    window_config = json.loads(args.windows.read_text()) if args.windows else None
    args.output_dir.mkdir(parents=True, exist_ok=False)
    cap = cv2.VideoCapture(str(args.input))
    if not cap.isOpened():
        raise RuntimeError('Cannot open input video')
    fps = cap.get(cv2.CAP_PROP_FPS)
    width, height = [int(cap.get(p)) for p in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT)]
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    name = args.model
    torch.set_num_threads(4)
    processor = GroundingDinoProcessor.from_pretrained(name, local_files_only=True)
    model = GroundingDinoForObjectDetection.from_pretrained(name, local_files_only=True).to('cuda').eval()
    # Competing background categories prevent forced target-only classification.
    prompts = ['a cup', 'a mug', 'a basketball', 'a trash can',
               'a chair', 'a table', 'a water bottle', 'a thermos', 'a fan']
    if window_config and 'prompts' in window_config:
        prompts = window_config['prompts']
    colors = {'cup': (60, 220, 60), 'basketball': (0, 170, 255), 'trash can': (255, 200, 40),
              'bottle': (60, 220, 60), 'thermos': (60, 220, 60)}
    colors.update({'cooker': (60, 220, 60), 'tissues': (60, 220, 60)})
    output = args.output_dir / 'rgb_detected.mp4'
    writer = None
    if args.sample_step == 1:
        rate = Fraction(fps).limit_denominator(100000)
        writer = subprocess.Popen([
            '/usr/bin/gst-launch-1.0', '-q', 'fdsrc', 'fd=0', f'blocksize={width*height*3}', '!',
            'videoparse', f'width={width}', f'height={height}', 'format=bgr',
            f'framerate={rate.numerator}/{rate.denominator}', '!', 'videoconvert', '!',
            'video/x-raw,format=I420', '!', 'x264enc', 'pass=qual', 'quantizer=18',
            'speed-preset=fast', '!', 'h264parse', '!', 'mp4mux', 'faststart=true', '!',
            'filesink', f'location={output}'], stdin=subprocess.PIPE)
    best = {}
    counts = collections.Counter()
    processed = 0
    started = time.monotonic()
    try:
        with (args.output_dir / 'detections.jsonl').open('x') as report:
            for index in range(count):
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError(f'Input decode failed at frame {index}/{count}')
                if index % args.sample_step:
                    continue
                allowed = None if window_config is None else set()
                if window_config:
                    for window in window_config['windows']:
                        if window['start_s'] <= index/fps < window['end_s']:
                            allowed.update(window['labels'])
                if allowed == set():
                    if writer:
                        writer.stdin.write(frame.tobytes())
                    report.write(json.dumps(dict(frame=index, video_time_s=index/fps,
                                                 inference=False, detections=[])) + '\n')
                    processed += 1
                    continue
                image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                inputs = processor(images=image, text=[prompts], return_tensors='pt').to('cuda')
                with torch.inference_mode(), torch.autocast('cuda', dtype=torch.float16):
                    prediction = model(**inputs)
                result = processor.post_process_grounded_object_detection(
                    prediction, inputs['input_ids'], threshold=args.threshold,
                    text_threshold=0.20, target_sizes=[(height, width)])[0]
                detections = []
                for label in colors:
                    if allowed is not None and label not in allowed:
                        continue
                    selected = [i for i, value in enumerate(result['text_labels']) if target_label(value) == label]
                    if not selected:
                        continue
                    keep = nms(result['boxes'][selected], result['scores'][selected], 0.45)
                    for k in keep.tolist():
                        i = selected[k]
                        box = result['boxes'][i].tolist()
                        x1, y1, x2, y2 = box
                        box = [max(0, min(width-1, x1)), max(0, min(height-1, y1)),
                               max(0, min(width-1, x2)), max(0, min(height-1, y2))]
                        if box[2] <= box[0] or box[3] <= box[1]:
                            continue
                        score = float(result['scores'][i])
                        detections.append(dict(label=label, score=score, bbox_xyxy=box))
                        counts[label] += 1
                for item in detections:
                    x1, y1, x2, y2 = map(int, item['bbox_xyxy'])
                    color = colors[item['label']]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    text = f"{item['label']} {item['score']:.2f}"
                    y = max(20, y1 - 8)
                    cv2.putText(frame, text, (x1, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 4)
                    cv2.putText(frame, text, (x1, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
                for item in detections:
                    label = item['label']
                    if item['score'] > best.get(label, {}).get('score', 0):
                        best[label] = dict(frame=index, time_s=index/fps, score=item['score'])
                        cv2.imwrite(str(args.output_dir / (label.replace(' ', '_') + '_best.jpg')), frame)
                if writer:
                    writer.stdin.write(frame.tobytes())
                else:
                    cv2.imwrite(str(args.output_dir / f'sample_{index:05d}.jpg'), frame)
                report.write(json.dumps(dict(frame=index, video_time_s=index/fps, detections=detections)) + '\n')
                processed += 1
                if processed % 50 == 0:
                    print(f'{index+1}/{count} frames, {time.monotonic()-started:.1f}s, boxes={dict(counts)}', flush=True)
    finally:
        cap.release()
        if writer:
            writer.stdin.close()
            if writer.wait(timeout=60) != 0:
                raise RuntimeError('H.264 encoder failed')
    summary = dict(input=str(args.input.resolve()), model=name, prompts=prompts,
                   threshold=args.threshold, sample_step=args.sample_step, processed_frames=processed,
                   source_frames=count, fps=fps, width=width, height=height,
                   detection_counts=dict(counts), best=best,
                   hover_window_config=window_config,
                   timing='Unchanged source-video timeline; NOT corrected to bag time',
                   interpretation='Offline model predictions, not mission success verification')
    if writer:
        check = cv2.VideoCapture(str(output))
        decoded = 0
        while check.read()[0]:
            decoded += 1
        check.release()
        if decoded != count:
            raise RuntimeError(f'Output verification failed: {decoded} != {count}')
        summary['verified_decoded_output_frames'] = decoded
    (args.output_dir / 'report.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
    main()
