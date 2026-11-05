# GrandTour ETH-1 Hesai/CPT7 GLIM bootstrap profile

This is the first isolated GLIM profile for GrandTour mission
`2024-10-01-11-29-55` (ETH-1). It deliberately changes only dataset-facing
configuration while retaining the validated CUDA GLIM settings from the
Newer College optimized profile.

- LiDAR: `/boxi/hesai/points` (`hesai_lidar`, approximately 10 Hz)
- IMU: `/boxi/cpt7/imu` (`cpt7_imu`, approximately 100 Hz)
- Point fields: `intensity`, `ring`, and auto-detected float64 `timestamp`
- Odometry: CUDA multi-resolution VGICP, 5 s fixed-lag smoother
- Submapping/global mapping: CUDA VGICP; global optimization enabled
- Initialization/noise: provisional NAIVE 0.75 s and generic GLIM noise

The mission TF bag records `T_box_base_hesai_lidar`. The `cpt7_imu` transform
is identity relative to `box_base`, so GLIM's required transform is the inverse:

`T_hesai_lidar_cpt7_imu = [-0.044611849471, 0.302238142205,
-0.012539945511, -0.709330179038, 0.704851604038, 0.003921407290,
-0.004419949690]` in TUM `[x y z qx qy qz qw]` order.

This profile is a bootstrap configuration, not a final tuned result. First
validate timestamp decoding, initialization, pose continuity, and metric scale
against the mission reference. Only then freeze an accuracy profile and add
CBS experiments.

For official GrandTour/COMFORT evaluation, convert GLIM's sensor trajectory to
the prism frame before alignment:
`T_world_prism(t) = T_world_source(t) * T_source_prism`.

The CPT7 IMU is present in this training mission, but it is redacted in the
benchmark test missions. Results using CPT7 therefore characterize our CBS
pipeline but are not directly deployable as a benchmark submission method.
