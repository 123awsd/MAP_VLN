# Vendored Livox-SDK2

This directory is a source snapshot of Livox-SDK2 commit
`f5d9375f84efe2b15bc0a052d3e18482ed13adf4` (2026-04-15), which adds
MID-360S support.

Two local portability fixes are included: the top-level and `sdk_core` CMake
minimum version is 3.5, and two headers explicitly include `<cstdint>`.

`livox_ros_driver2` builds this bundled SDK by default, so a clean workspace
does not require a system-wide SDK installation. To install it separately:

```bash
cmake -S src/SUPER/drivers/livox_ros_driver2/3rdparty/Livox-SDK2 -B /tmp/livox-sdk2-build
cmake --build /tmp/livox-sdk2-build --target livox_lidar_sdk_static
sudo cmake --install /tmp/livox-sdk2-build
```
