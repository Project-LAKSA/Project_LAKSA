# ADR-002: REP-105 TF contract

`map -> odom` is owned by exactly one global localization/mapping component.
`odom -> base_link` is owned by the local estimator. A V2 planar-frame adapter
may publish the derived `base_link -> base_footprint` projection for Nav2; it
is deliberately **not** a fixed URDF joint because ramps/body attitude make a
projection dynamic. Body/sensor static edges come from `robot_state_publisher`.
ZED, LiDAR, and IMU are children of `base_link`.

`base_link` is the vehicle semantic root; `base_footprint` is not a camera
substitute. Dynamic TF freshness and duplicate-authority detection are health
requirements. On loss: autonomy disarms; no static transform is invented to
mask it. Rollback: recovered mapping TF topology remains isolated.
