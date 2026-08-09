# Particle Filter Localization

This code implements the MCL algorithm for the RACECAR. 

[![YouTube Demo](./media/thumb.jpg)](https://www.youtube.com/watch?v=-c_0hSjgLYw)

For high efficiency in Python, it uses Numpy arrays and [RangeLibc](https://github.com/f1tenth/range_libc) for fast 2D ray casting.

# Installation

To run this, you need to ensure that both the map_server ROS package, and the python wrappers for RangeLibc are installed.

For the map server:
```
sudo apt-get update
rosdep install -r --from-paths src --ignore-src --rosdistro kinetic -y
```

For [RangeLibc](https://github.com/f1tenth/range_libc):

```
sudo pip install cython
git clone http://github.com/kctess5/range_libc
cd range_libc/pywrappers
# on VM
./compile.sh
# on car - compiles GPU ray casting methods
./compile_with_cuda.sh
```

# Usage

The majority of parameters you might want to tweak are in the launch/localize.launch file. You may have to modify the "odometry_topic" or "scan_topic" parameters to match your environment.

```
roslaunch particle_filter localize.launch
```

Once the particle filter is running, you can visualize the map and other particle filter visualization message in RViz. Use the "2D Pose Estimate" tool from the RViz toolbar to initialize the particle locations.

See [launch/localize.launch](/particle_filter/launch/localize.launch) for docs on available parameters and arguments.

The "range_method" parameter determines which RangeLibc ray casting method to use. The default is cddt because it is fast and has a low initialization time. The fastest option on the CPU is "glt" but it has a slow startup. The fastest version if you have can compile RangeLibc with CUDA enabled is "rmgpu". See this performance comparison chart:

![Range Method Performance Comparison](./media/comparison.png)

# Docs

This code is the staff solution to the lab guide found in the [/docs](/particle_filter/docs) folder. A mathematical derivation of MCL is available in that guide.

There is also documentation on RangeLibc in the [/docs](/particle_filter/docs) folder.

The code itself also contains comments describing purpose of each method.

# Slip-Aware Stationary Hold

The production update is synchronized at the LiDAR timestamp and runs in this
order:

1. Interpolate wheel odometry at `t_scan` and integrate the bounded IMU gyro
   deque exactly between the previous and current scan timestamps.
2. Estimate independent scan-to-scan motion with robust ICP on the same 73
   downsampled beams used by the PF.
3. Compute a hysteretic slip score from scan/odom translation, scan/odom yaw,
   odom/IMU yaw, and scan/IMU yaw innovations.
4. During slip, disable stationary hold, multiply process noise by 2.5, and
   permit a jump-guard bypass only with scan-map fit at least 0.75 and a
   correction inside 0.80 m / 0.80 rad.
5. Freeze the filter only when wheel speed/action, IMU motion, and scan motion
   all independently indicate stationary.

The current VESC driver publishes gyro in deg/s and acceleration in g, so
`localize.yaml` explicitly converts them to rad/s and m/s^2. Also note that the
current `/odom` yaw already comes from the same VESC IMU: scan-vs-odom
translation is therefore the independent evidence for wheelspin, while the IMU
mainly protects yaw and stationary classification.

Missing or gapped IMU data never blocks MCL output. It disables stationary hold
for that interval, and the periodic `PF sync:` line reports IMU misses, scan
matcher validity, slip score/state, event/recovery/bypass counts, event age,
last duration, and maximum correction. Exact scan-time entry and recovery are
also emitted as `PF slip detected` and `PF slip recovered` log lines.

# Autonomy Readiness Gate

The PF publishes transient-local `/pf/initialpose_received`,
`/pf/initialpose_accepted`, `/pf/localization_quality`, and
`/pf/localization_ready` status. `received` means the RViz message reached the
PF, while `accepted` means the LiDAR/base transform was resolved and the local
particle cloud was actually seeded. A received pose is cached and retried at
10 Hz while the fixed transform is unavailable. Readiness then requires
`accepted` plus the configured five consecutive scan-map quality-OK updates.
The topic is refreshed on every processed scan, so a stalled worker becomes a
stale lease rather than leaving a fresh-looking `true` behind.

`wait_for_localization.py` does not start the real-car autonomy stack before the
lease is ready. Independently, real-car `control_node` publishes only zero when
the lease is false or older than 0.5 s. A single DEGRADED update does not cause
an abrupt race-speed stop after arming, but LOST revokes readiness immediately
and full consecutive-OK recovery is required to re-arm. Joystick and estop are
higher-priority mux inputs and are not blocked.

`initialize_global()` still spreads particles over the full map, but it is not
treated as a safe automatic initial-pose solution. The normal MCL scan update
does refine an `/initialpose` seed locally; there is no robust, validated global
startup pose detector in this repository.

# LiDAR Deskew At The Scan Midpoint

`LaserScan.header.stamp` is treated as the acquisition time of the first
delivered beam. The PF keeps each selected beam's original array index and
computes:

```text
t_beam = header.stamp + selected_index * time_increment
t_ref  = header.stamp + 0.5 * (beam_count - 1) * time_increment
```

For the recorded 1081-beam UST-10LX scans this span is 18.75 ms, so `t_ref` is
`header.stamp + 9.375 ms`. Only the 73 beams selected by `angle_step=15` are
deskewed and raycast. One bounded-buffer lookup interpolates odometry at all beam times,
and one trapezoidal IMU pass integrates gyro yaw from `t_ref` to every beam.
Each valid endpoint is then transformed by
`T_laser(ref)_laser(beam)` before RangeLib and scan-to-scan ICP use it.

The wheel translation scale is normally 1.0. While the hysteretic slip state
is active it uses `deskew_slip_translation_scale` (0.0 by default), while IMU
rotation and rotation-induced motion from the LiDAR lever arm stay enabled.
Missing IMU falls back to odom yaw; missing per-beam odom disables wheel
translation but still permits IMU rotational deskew; invalid scan timing falls
back to the original scan unchanged. None of these fallbacks blocks MCL.

The periodic `PF sync:` line reports deskew source, applied count,
timing/odom/IMU fallback counts, scan span, translation scale, maximum motion,
and changed-scan-shape drops. Configure the behavior in `config/localize.yaml`
with `deskew_enabled`, `deskew_max_scan_duration_sec`, and
`deskew_slip_translation_scale`. Keep `rangelib_variant=5` (the configured
default); radial variant 4 assumes uniformly spaced angles and is rejected
when deskew is enabled.

# 3D Scan Geometry Gate

UST invalid/no-return values, including 65.533 m, become invalid before any
median or geometry operation. Ground continuity is evaluated on all 1081 raw
beams, then projected onto the fixed 73-beam ICP/MCL mask, so the RangeLib
raycast budget does not grow. Dynamic IMU tilt is trusted only when a continuous
expected-floor component inside the forward evidence sector satisfies the same
beam, angle, and XY-span tests, or when valid returns collapse like a sky frame.
Otherwise the scan falls back to the measured static LiDAR extrinsic. Frame-drop
ratios also use the forward sector; side/rear beams are still individually
masked.

Inspect these decisions directly on a real-driving bag:

```bash
cd ~/IFAC_ws
python3 tools/analyze_scan_geometry.py \
  bags/rosbag0808/run_0808_222948 \
  --sample-hz 0 --pitch-deg 0 \
  --inspect-sec 47.6,53.46,147.5,273.08
```

`--sample-hz 0` checks every scan. The report includes AHRS tilt, fallback and
degraded fractions, longest dropped interval, requested-time IMU/odom values,
and raw component radii before map/track filters. A pitch sweep is diagnostic
only: set `base_link -> laser` and `base_link -> vesc_imu` in
`hardware_ws/src/f1tenth_stack/config/sensors.yaml` from physical measurements,
not from the sweep's best-looking result.

# MCL Regression Tests (ROS 2 Humble)

The ESS, systematic-resampling, beam-skipping, stable log-likelihood,
scan-motion estimator, slip detector, and LOST/recovery state machine are kept
in a ROS-independent production module.
Run the exact same functions used by the node with:

```bash
cd ~/IFAC_ws
PYTHONPATH=$PWD/localization_ws/src/particle_filter \
  python3 -m pytest -q \
  localization_ws/src/particle_filter/test/test_mcl_core.py \
  localization_ws/src/particle_filter/test/test_synchronization.py
```

Measure Python-side cost and verify the configured raycast budget on either
the x86 development PC or Jetson:

```bash
python3 tools/benchmark_pf_mcl.py
```

After a replay recording, `tools/compare_pf_replay.py` reports prior/posterior
ESS, resampling count, scan fit, skipped beams, skip fallbacks, and
DEGRADED/LOST samples together with the existing map-fit and runtime metrics.
The candidate default uses `angle_step=15`; pass the real A/B steps explicitly
if either configuration differs:

```bash
python3 tools/compare_pf_replay.py \
  --old baseline.db3 --new candidate.db3 \
  --old-angle-step 10 --new-angle-step 15
```

Summarize existing real-driving bags without replaying the node:

```bash
python3 tools/analyze_pf_bags.py \
  bags/run_0806_233816 bags/run_0807_153600
```

The `Auto*` columns and worst intervals use only samples where the final
`/ackermann_cmd` is an exact ackermann_mux copy of
`/autonomy/ackermann_cmd`, the command is fresh, and odometry speed is at
least 0.5 m/s. Manual, estop, transition-stop, stationary, and unclassified
mux output are excluded. `MuxMatch` should be near 100%; otherwise mode-only
metrics are not trustworthy for that bag.

Run a deterministic baseline/candidate MCL A/B directly on recorded scans,
scan-time interpolated odometry, and integrated IMU. The default
`candidate-hold-stop` follows the production slip-aware path:

```bash
OMP_NUM_THREADS=8 python3 tools/offline_pf_replay.py \
  bags/rosbag0806/run_0806_235014 \
  --start-sec 10 --duration-sec 45 --rate-hz 15
```

Isolate deskew from every other MCL/slip setting with a deterministic A/B:

```bash
OMP_NUM_THREADS=8 python3 tools/offline_pf_replay.py \
  bags/rosbag0807/run_0807_134045 \
  --start-sec 10 --duration-sec 30 \
  --configs candidate-hold-stop-no-deskew candidate-hold-stop
```

The offline tool first compares its occupancy-grid DDA raycaster against the
RangeLib fake scans recorded on the car. It is suitable for algorithm and
held-out map-fit regression, but Jetson compute/turnaround acceptance must use
an actual Humble replay because the validation raycaster is not RangeLib.

# Cite

This library accompanies the following [publication](http://arxiv.org/abs/1705.01167).

    @article{walsh17,
        author = {Corey Walsh and 
                  Sertac Karaman},
        title  = {CDDT: Fast Approximate 2D Ray Casting for Accelerated Localization},
        volume = {abs/1705.01167},
        url    = {http://arxiv.org/abs/1705.01167},
        year   = {2017}}
