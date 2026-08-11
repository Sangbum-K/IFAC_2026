# C++ Particle Filter Localization

The vehicle localization runtime is the native C++17 node from
`particle_filter_cpp`. This `particle_filter` package only installs maps,
parameter profiles, RViz configuration, and the ROS 2 launch file. It does not
contain a Python localization node or Python MCL implementation.

## Runtime

The default profile is `variant0_no_ground_no_deskew`, which enables C++ MCL,
RangeLib GiantLUT ray casting, beam skipping, scan-to-scan ICP, slip detection,
quality/readiness tracking, and map-to-odom TF publication. The optional
`variant0_no_ground_no_deskew_no_icp` profile disables only ICP and its
dependent slip evidence.

Ground filtering and scan deskew are disabled. They regressed the 0809 bag and
depended on IMU/LiDAR geometry that had not been physically calibrated after
the vehicle height changed. The C++ node rejects a configuration that enables
either feature instead of silently running a different path.

## Build

```bash
cd ~/IFAC_ws
./setup_deps.sh   # once per machine; provides ~/range_libc
./build_all.sh
```

The default real-car launch starts the C++ node automatically:

```bash
rc my_track0809 rviz:=false
```

Select the no-ICP diagnostic profile only when intentionally testing it:

```bash
rc my_track0809 \
  pf_profile:=variant0_no_ground_no_deskew_no_icp \
  rviz:=false
```

Do not pass `pf_engine`; the runtime is always C++.

## Safe rosbag replay

Run the exact recorded map, scan, odometry, IMU, static TF, and initial pose in
an isolated ROS domain without hardware, planning, control, mux, or actuation:

```bash
cd ~/IFAC_ws
tools/hbl python3 tools/run_pf_ab_replay.py \
  --profiles variant0_no_ground_no_deskew \
  --base-to-laser-z-m 0.1374
```

Compare ICP on/off:

```bash
tools/hbl python3 tools/run_pf_ab_replay.py \
  --profiles \
  variant0_no_ground_no_deskew \
  variant0_no_ground_no_deskew_no_icp \
  --base-to-laser-z-m 0.1374
```

Runtime acceptance is PF correction rate at least 15 Hz, correction-age p99 at
most 100 ms, and zero stale/worker errors. The `Auto*` quality columns use only
moving samples where the final mux output exactly matches a fresh autonomy
command. A bag has no independent pose ground truth, so this is a regression
and timing check rather than proof of absolute localization accuracy.

## Status interfaces

The C++ PF publishes transient-local status topics:

- `/pf/initialpose_received`
- `/pf/initialpose_accepted`
- `/pf/localization_quality`
- `/pf/localization_ready`

Autonomy readiness requires an accepted RViz initial pose and consecutive
scan-map quality updates. The node also publishes `/pf/pose/odom`, visualization
topics, and `map -> odom` TF with stale-output protection.
