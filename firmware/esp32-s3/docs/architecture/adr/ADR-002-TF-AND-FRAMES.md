# ADR-002: REP-105 TF contract

`map -> odom` is owned by exactly one global localization/mapping component.
`odom -> base_link` is owned by the local estimator. `base_link ->
base_footprint` is a deterministic planar projection published by one V2 frame
adapter only when required by Nav2; body/sensor static edges come from
`robot_state_publisher`. ZED, LiDAR, and IMU are children of `base_link`.

`base_link` is the vehicle semantic root; `base_footprint` is not a camera
substitute. Dynamic TF freshness and duplicate-authority detection are health
requirements. On loss: autonomy disarms; no static transform is invented to
mask it. Rollback: recovered mapping TF topology remains isolated.
