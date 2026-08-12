# particle_filter_cpp

Native C++17 IFAC particle-filter localization package. It contains the MCL
runtime together with its launch file, parameter profiles, maps, and RViz
configuration. RangeLib ray casting, scan matching, state tracking, TF, and
status interfaces all run in the compiled ROS 2 node; there is no Python PF
runtime or separate asset-only package.

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

The CMake build needs a RangeLib source checkout (headers only). It is looked up in
this order, first hit wins:

1. `-DRANGELIBC_ROOT=<path>`
2. `$RANGE_LIBC`
3. `~/range_libc`
4. `<workspace>/range_libc`, `<workspace sibling>/range_libc`
5. `/opt/range_libc`

`setup_deps.sh` and `build_all.sh` search the same list and export `RANGE_LIBC` so
CMake resolves to the identical checkout.

## Real-car launch

The normal launch selects the C++ package automatically:

```bash
rc my_track0809 rviz:=false
```

Use the no-ICP profile only for an intentional comparison:

```bash
rc my_track0809 \
  pf_profile:=variant0_no_ground_no_deskew_no_icp \
  rviz:=false
```

Do not pass `pf_engine`; the only runtime is C++.

## Offline replay

```bash
cd ~/IFAC_ws
tools/hbl python3 tools/run_pf_ab_replay.py \
  --profiles variant0_no_ground_no_deskew \
  --base-to-laser-z-m 0.1374
```

This uses an isolated ROS domain and launches no hardware or control nodes.
Run it on the Jetson before any real-car drive.

## Status interfaces

The node publishes `/pf/initialpose_received`, `/pf/initialpose_accepted`,
`/pf/localization_quality`, `/pf/localization_ready`, `/pf/pose/odom`, and
`map -> odom` TF. Autonomy readiness requires an accepted initial pose and
consecutive scan-map quality updates.
