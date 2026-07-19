# Steering lookup table

The environment profiles select one of the temporary ETH-format LUTs in this
directory:

- `SIM_linear_lookup_table.csv`: simulator
- `NUC2_hangar_pacejka_lookup_table.csv`: initial low-speed real-car testing

Replace the real-car table after identifying the IFAC vehicle and update
`map/steering_lut_file` in the relevant environment profile.

The first row contains velocities in m/s, the first column contains steering
angles in radians, and each body cell contains lateral acceleration in m/s^2.
