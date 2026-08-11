# particle_filter_cpp

Native C++17 IFAC particle-filter localization runtime. It implements the MCL
update, RangeLib ray casting, scan matching, state tracking, TF, and status
interfaces in a compiled ROS 2 node. There is no Python PF runtime fallback.

## Supported configuration

The launch layer provides these C++ profiles:

- `variant0_no_ground_no_deskew`
- `variant0_no_ground_no_deskew_no_icp`

Ground filtering and scan deskew are disabled because the 0809 regression
showed worse behavior with the unmeasured IMU/LiDAR geometry. The node fails at
startup if either is enabled instead of silently changing behavior.

## Build and test

```bash
cd ~/IFAC_ws
./setup_deps.sh
./build_all.sh

cd localization_ws
colcon test --packages-select particle_filter_cpp
colcon test-result --verbose
```

The CMake build expects the RangeLib source checkout at `~/range_libc`.

## Offline replay

```bash
cd ~/IFAC_ws
tools/hbl python3 tools/run_pf_ab_replay.py \
  --profiles variant0_no_ground_no_deskew \
  --base-to-laser-z-m 0.1374
```

This uses an isolated ROS domain and launches no hardware or control nodes.
Run it on the Jetson before any real-car drive.
