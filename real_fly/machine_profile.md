# Machine profile

Snapshot collected over SSH from the onboard computer on 2026-09-03.

## Identity

| Field | Value |
|---|---|
| Hostname | `uav26` |
| Login user | `nv` |
| Current IPv4 | `192.168.0.250` on `wlan0` (DHCP; may change) |
| Default gateway | `192.168.0.1` |

## System

| Field | Value |
|---|---|
| Device | NVIDIA Orin NX Developer Kit |
| OS | Ubuntu 20.04.6 LTS (Focal Fossa) |
| Architecture | `aarch64` / `arm64` |
| Kernel | `5.10.216-tegra` |
| NVIDIA L4T | R35.6.4 |
| Root filesystem | 233 GB total, 21 GB used, 201 GB available at snapshot |

## Software

| Field | Value |
|---|---|
| ROS | ROS 1 Noetic |
| ROS installation | `/opt/ros/noetic` |
| `ROS_PACKAGE_PATH` | `/opt/ros/noetic/share` |
| Python | `/usr/bin/python3`, Python 3.8.10 |
| Jetson monitoring | `/usr/bin/tegrastats` available |
| SSH server | Installed and active |
| NoMachine | Not detected in the package list |
| Mihomo/Clash | Not detected in the package list |

## Network snapshot

| Interface | State | Address |
|---|---|---|
| `wlan0` | UP | `192.168.0.250/24` |
| `eth0` | DOWN | — |

This file contains no password, private key, or subscription URL.  The IP
address is only a connection snapshot and must not be treated as permanent.
