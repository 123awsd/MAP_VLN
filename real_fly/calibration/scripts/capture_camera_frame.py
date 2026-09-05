#!/usr/bin/env python3
"""Capture one D435 color frame and verify the official board's ArUco IDs."""

import argparse
import json
import os
import sys

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image


EXPECTED_K = (
    605.2134399414062,
    0.0,
    330.0566101074219,
    0.0,
    605.4168090820312,
    245.53988647460938,
    0.0,
    0.0,
    1.0,
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--camera-info", required=True)
    parser.add_argument("--timeout", type=float, default=15.0)
    return parser.parse_args()


def refuse_existing(path):
    if os.path.exists(path):
        raise RuntimeError("refusing to overwrite: " + path)


def main():
    args = parse_args()
    refuse_existing(args.image)
    refuse_existing(args.camera_info)
    rospy.init_node("fast_calib_capture_frame", anonymous=True)

    info = rospy.wait_for_message(
        "/camera/color/camera_info", CameraInfo, timeout=args.timeout
    )
    image_msg = rospy.wait_for_message(
        "/camera/color/image_raw", Image, timeout=args.timeout
    )
    if (info.width, info.height) != (640, 480):
        raise RuntimeError(
            "expected D435 color 640x480, got {}x{}".format(info.width, info.height)
        )
    max_k_error = max(abs(a - b) for a, b in zip(info.K, EXPECTED_K))
    if max_k_error > 0.05:
        raise RuntimeError(
            "live CameraInfo differs from the checked config (max K error {:.6f}); "
            "do not calibrate until the config is updated".format(max_k_error)
        )

    frame = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    _, ids, _ = cv2.aruco.detectMarkers(frame, dictionary)
    detected = sorted(int(value) for value in ids.flatten()) if ids is not None else []
    expected = [1, 2, 3, 4]
    if not set(expected).issubset(detected):
        raise RuntimeError(
            "board check failed: expected ArUco IDs {}, detected {}. "
            "Reposition/illuminate the board and retry".format(expected, detected)
        )

    if not cv2.imwrite(args.image, frame):
        raise RuntimeError("failed to write image: " + args.image)
    payload = {
        "topic": "/camera/color/camera_info",
        "image_topic": "/camera/color/image_raw",
        "stamp": info.header.stamp.to_sec(),
        "width": info.width,
        "height": info.height,
        "distortion_model": info.distortion_model,
        "K": list(info.K),
        "D": list(info.D),
        "P": list(info.P),
        "detected_aruco_ids": detected,
    }
    with open(args.camera_info, "x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)
        stream.write("\n")
    print("D435 frame saved; detected ArUco IDs: " + ", ".join(map(str, detected)))


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, rospy.ROSException) as exc:
        print("ERROR: " + str(exc), file=sys.stderr)
        sys.exit(1)
