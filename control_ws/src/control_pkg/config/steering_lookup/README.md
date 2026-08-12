# Steering lookup table

The environment profiles select one of the temporary ETH-format LUTs in this
directory:

- `SIM_linear_lookup_table.csv`: simulator
- `NUC2_hangar_pacejka_lookup_table.csv`: initial low-speed real-car testing
- `NUC6_glc_pacejka_lookup_table.csv`: ForzaETH NUC6 GLC calibration
  ([source](https://github.com/ForzaETH/race_stack/blob/ros2-humble/system_identification/steering_lookup/cfg/NUC6_glc_pacejka_lookup_table.csv))
- `UNICORN2-hj-0331-faster-more-data_pacejka_lookup_table.csv`: candidate for
  the current IFAC car and post-2026-08-10 tire configuration
  ([source](https://github.com/HMCL-UNIST/unicorn-racing-stack/blob/ros1/system_identification/steering_lookup/cfg/UNICORN2-hj-0331-faster-more-data_pacejka_lookup_table.csv))

Replace the real-car table after identifying the IFAC vehicle and update
`map/steering_lut_file` in the relevant environment profile.

The first row contains velocities in m/s, the first column contains steering
angles in radians, and each body cell contains lateral acceleration in m/s^2.
