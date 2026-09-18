#!/usr/bin/env python3
import argparse
import csv
import glob
import math
import os
import statistics

DEFAULT_LOG_DIR = "/home/nv/localization_only_ws/logs/eight_tracking"


def read_float(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def finite(values):
    return [v for v in values if math.isfinite(v)]


def rms(values):
    values = finite(values)
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def maximum(values):
    values = finite(values)
    return max(values) if values else float("nan")


def mean_abs(values):
    values = [abs(v) for v in finite(values)]
    return statistics.mean(values) if values else float("nan")


def load_csv(path):
    with open(path, "r", newline="") as f:
        rows = list(csv.DictReader(f))

    keys = [
        "time",
        "des_px",
        "des_py",
        "des_pz",
        "odom_px",
        "odom_py",
        "odom_pz",
        "err_norm",
        "vel_err_norm",
        "yaw_err",
    ]
    data = {key: [] for key in keys}
    for row in rows:
        for key in keys:
            data[key].append(read_float(row, key))
    return data


def latest_log_path(log_dir):
    pattern = os.path.join(log_dir, "eight_tracking_*.csv")
    candidates = glob.glob(pattern)
    if not candidates:
        raise FileNotFoundError("no logs found: %s" % pattern)
    return max(candidates, key=os.path.getmtime)


def print_summary(data):
    count = len(data["time"])
    duration = 0.0
    if count >= 2:
        duration = data["time"][-1] - data["time"][0]

    print("samples:", count)
    print("duration_s: %.3f" % duration)
    print("position_error_rms_m: %.4f" % rms(data["err_norm"]))
    print("position_error_max_m: %.4f" % maximum(data["err_norm"]))
    print("velocity_error_rms_mps: %.4f" % rms(data["vel_err_norm"]))
    print("velocity_error_max_mps: %.4f" % maximum(data["vel_err_norm"]))
    print("yaw_error_mean_abs_rad: %.4f" % mean_abs(data["yaw_err"]))
    print("yaw_error_max_abs_rad: %.4f" % maximum([abs(v) for v in data["yaw_err"]]))


def plot(data, output):
    import matplotlib.pyplot as plt

    if not data["time"]:
        raise RuntimeError("empty log")

    t0 = data["time"][0]
    t = [x - t0 for x in data["time"]]

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

    axes[0].plot(t, data["des_px"], "r--", label="des x")
    axes[0].plot(t, data["odom_px"], "r", label="odom x")
    axes[0].plot(t, data["des_py"], "g--", label="des y")
    axes[0].plot(t, data["odom_py"], "g", label="odom y")
    axes[0].plot(t, data["des_pz"], "b--", label="des z")
    axes[0].plot(t, data["odom_pz"], "b", label="odom z")
    axes[0].set_ylabel("position m")
    axes[0].grid(True)
    axes[0].legend(loc="best", ncol=3)

    axes[1].plot(t, data["err_norm"], "k")
    axes[1].set_ylabel("pos err m")
    axes[1].grid(True)

    axes[2].plot(t, data["vel_err_norm"], "m")
    axes[2].set_ylabel("vel err m/s")
    axes[2].set_xlabel("time s")
    axes[2].grid(True)

    fig.tight_layout()
    if output:
        fig.savefig(output, dpi=160)
        print("saved_plot:", output)
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Analyze eight trajectory tracking CSV.")
    parser.add_argument("csv", nargs="?", help="CSV log path. Defaults to latest project log.")
    parser.add_argument("--log-dir", default=DEFAULT_LOG_DIR,
                        help="directory used when csv is omitted")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--save-plot", default="")
    args = parser.parse_args()

    csv_path = args.csv if args.csv else latest_log_path(args.log_dir)
    print("log:", csv_path)

    data = load_csv(csv_path)
    print_summary(data)
    if args.plot or args.save_plot:
        plot(data, args.save_plot)


if __name__ == "__main__":
    main()
