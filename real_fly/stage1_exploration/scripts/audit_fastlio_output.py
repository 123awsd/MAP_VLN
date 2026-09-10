#!/usr/bin/env python3
"""Audit input lidar coverage and a regenerated FAST-LIO output bag."""

import argparse
import json
import math
import os
import statistics
import sys

import rosbag


def message_stamp(message, bag_stamp):
    header = getattr(message, "header", None)
    if header is not None and not header.stamp.is_zero():
        return header.stamp.to_sec()
    return bag_stamp.to_sec()


def topic_stamps(path, topic):
    stamps = []
    with rosbag.Bag(path, "r") as bag:
        for _, message, bag_stamp in bag.read_messages(topics=[topic]):
            stamps.append(message_stamp(message, bag_stamp))
    return stamps


def stamp_summary(stamps):
    if not stamps:
        return {
            "count": 0,
            "first_header_stamp": None,
            "last_header_stamp": None,
            "duration_sec": 0.0,
            "frequency_hz": 0.0,
            "non_monotonic_count": 0,
        }
    duration = stamps[-1] - stamps[0]
    return {
        "count": len(stamps),
        "first_header_stamp": stamps[0],
        "last_header_stamp": stamps[-1],
        "duration_sec": duration,
        "frequency_hz": (len(stamps) - 1) / duration if duration > 0 else 0.0,
        "non_monotonic_count": sum(b <= a for a, b in zip(stamps, stamps[1:])),
    }


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * fraction))))
    return ordered[index]


def audit_odometry(path, topic):
    stamps = []
    positions = []
    invalid_count = 0
    quaternion_norm_errors = []
    with rosbag.Bag(path, "r") as bag:
        for _, message, bag_stamp in bag.read_messages(topics=[topic]):
            stamp = message_stamp(message, bag_stamp)
            p = message.pose.pose.position
            q = message.pose.pose.orientation
            values = [stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            if not all(math.isfinite(value) for value in values):
                invalid_count += 1
                continue
            stamps.append(stamp)
            positions.append((p.x, p.y, p.z))
            quaternion_norm_errors.append(abs(math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) - 1.0))

    distances = [
        math.sqrt(sum((b[index] - a[index]) ** 2 for index in range(3)))
        for a, b in zip(positions, positions[1:])
    ]
    summary = stamp_summary(stamps)
    summary.update({
        "invalid_message_count": invalid_count,
        "position_min_m": [min(axis) for axis in zip(*positions)] if positions else None,
        "position_max_m": [max(axis) for axis in zip(*positions)] if positions else None,
        "max_step_m": max(distances) if distances else 0.0,
        "p99_step_m": percentile(distances, 0.99),
        "median_step_m": statistics.median(distances) if distances else 0.0,
        "max_quaternion_norm_error": max(quaternion_norm_errors) if quaternion_norm_errors else None,
    })
    return summary


def odometry_samples(path, topic):
    samples = []
    with rosbag.Bag(path, "r") as bag:
        for _, message, bag_stamp in bag.read_messages(topics=[topic]):
            stamp = message_stamp(message, bag_stamp)
            p = message.pose.pose.position
            values = (stamp, p.x, p.y, p.z)
            if all(math.isfinite(value) for value in values):
                samples.append(values)
    return samples


def interpolate_position(samples, stamp, start_index=0):
    index = start_index
    while index + 1 < len(samples) and samples[index + 1][0] < stamp:
        index += 1
    if index + 1 >= len(samples):
        return None, index
    before = samples[index]
    after = samples[index + 1]
    if stamp < before[0] or after[0] <= before[0]:
        return None, index
    fraction = (stamp - before[0]) / (after[0] - before[0])
    position = tuple(
        before[axis] + fraction * (after[axis] - before[axis])
        for axis in range(1, 4)
    )
    return position, index


def compare_odometry(input_bag, output_bag, topic):
    reference = odometry_samples(input_bag, topic)
    regenerated = odometry_samples(output_bag, topic)
    result = {
        "topic": topic,
        "reference_count": len(reference),
        "regenerated_count": len(regenerated),
        "available": False,
    }
    if len(reference) < 2 or len(regenerated) < 2:
        result["reason"] = "reference or regenerated odometry is missing"
        return result

    overlap_start = max(reference[0][0], regenerated[0][0])
    overlap_end = min(reference[-1][0], regenerated[-1][0])
    if overlap_end <= overlap_start:
        result["reason"] = "odometry time ranges do not overlap"
        return result

    reference_origin, reference_index = interpolate_position(reference, overlap_start)
    regenerated_origin, _ = interpolate_position(regenerated, overlap_start)
    if reference_origin is None or regenerated_origin is None:
        result["reason"] = "cannot interpolate odometry at overlap start"
        return result

    errors = []
    vertical_errors = []
    final_delta = None
    # Cap the comparison work while retaining the complete time span.
    stride = max(1, len(regenerated) // 5000)
    for sample in regenerated[::stride]:
        stamp = sample[0]
        if stamp < overlap_start or stamp > overlap_end:
            continue
        reference_position, reference_index = interpolate_position(
            reference, stamp, reference_index
        )
        if reference_position is None:
            continue
        regenerated_relative = tuple(
            sample[axis + 1] - regenerated_origin[axis] for axis in range(3)
        )
        reference_relative = tuple(
            reference_position[axis] - reference_origin[axis] for axis in range(3)
        )
        delta = tuple(
            regenerated_relative[axis] - reference_relative[axis]
            for axis in range(3)
        )
        errors.append(math.sqrt(sum(value * value for value in delta)))
        vertical_errors.append(abs(delta[2]))
        final_delta = delta

    if not errors:
        result["reason"] = "no matched odometry samples"
        return result

    result.update({
        "available": True,
        "overlap_start_sec": overlap_start,
        "overlap_end_sec": overlap_end,
        "matched_count": len(errors),
        "median_relative_position_error_m": statistics.median(errors),
        "p95_relative_position_error_m": percentile(errors, 0.95),
        "max_relative_position_error_m": max(errors),
        "p95_relative_vertical_error_m": percentile(vertical_errors, 0.95),
        "final_relative_delta_xyz_m": list(final_delta),
        "final_relative_position_error_m": errors[-1],
    })
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--output-bag")
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--max-reference-p95-error-m", type=float, default=0.75)
    parser.add_argument("--max-reference-final-error-m", type=float, default=1.0)
    parser.add_argument("--map-pcd")
    parser.add_argument("--json")
    parser.add_argument("--last-input-only", action="store_true")
    args = parser.parse_args()

    lidar = stamp_summary(topic_stamps(args.input_bag, args.lidar_topic))
    if args.last_input_only:
        if lidar["last_header_stamp"] is None:
            return 2
        print("{:.9f}".format(lidar["last_header_stamp"]))
        return 0

    report = {
        "input_bag": os.path.abspath(args.input_bag),
        "input_lidar": lidar,
    }
    passed = lidar["count"] > 0 and lidar["non_monotonic_count"] == 0
    if args.output_bag:
        odometry = audit_odometry(args.output_bag, args.odom_topic)
        final_lag = None
        if lidar["last_header_stamp"] is not None and odometry["last_header_stamp"] is not None:
            final_lag = lidar["last_header_stamp"] - odometry["last_header_stamp"]
        report.update({
            "output_bag": os.path.abspath(args.output_bag),
            "odometry": odometry,
            "last_lidar_minus_last_odometry_sec": final_lag,
        })
        passed = passed and (
            odometry["count"] > 0
            and odometry["non_monotonic_count"] == 0
            and odometry["invalid_message_count"] == 0
            and final_lag is not None
            and abs(final_lag) <= 0.25
            and 15.0 <= odometry["frequency_hz"] <= 25.0
        )
        consistency = compare_odometry(args.input_bag, args.output_bag, args.odom_topic)
        report["online_offline_odometry_consistency"] = consistency
        if consistency["available"]:
            consistency["max_p95_error_m"] = args.max_reference_p95_error_m
            consistency["max_final_error_m"] = args.max_reference_final_error_m
            consistency["passed"] = (
                consistency["p95_relative_position_error_m"]
                <= args.max_reference_p95_error_m
                and consistency["final_relative_position_error_m"]
                <= args.max_reference_final_error_m
            )
            passed = passed and consistency["passed"]
    if args.map_pcd:
        report["map_pcd"] = {
            "path": os.path.abspath(args.map_pcd),
            "exists": os.path.isfile(args.map_pcd),
            "size_bytes": os.path.getsize(args.map_pcd) if os.path.isfile(args.map_pcd) else 0,
        }
        passed = passed and report["map_pcd"]["exists"] and report["map_pcd"]["size_bytes"] > 0
    report["passed"] = passed

    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
    print(rendered)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
