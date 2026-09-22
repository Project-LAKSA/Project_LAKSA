# ADR-003: local state estimation

## Context and requirements

G2 needs continuous local motion without a map, planner, controller, or
hardware-specific authority. G1 fixes the frame contract as
`odom -> base_footprint -> base_link`; only the first edge belongs to the
local navigation estimator. The output must be standard `nav_msgs/Odometry`
and a single fresh `odom -> base_footprint` TF edge.

## Alternatives considered

1. A LAKSA EKF/UKF: rejected. `robot_localization` already provides a maintained
   ROS 2 estimator, covariance handling, diagnostics, frame semantics, and
   innovation rejection.
2. `robot_localization` UKF: deferred. Synthetic G2 trajectories do not show a
   non-linearity that justifies its extra tuning and runtime cost.
3. ZED as the `odom -> base` TF authority: rejected. It makes a camera the
   vehicle root and conflicts with the canonical estimator contract.
4. Fusing raw ZED IMU together with ZED VIO: rejected for G2. ZED VIO already
   incorporates the camera IMU when `imu_fusion=true`; this would double-count
   correlated information.

## Decision

Use one official `robot_localization` `ekf_node`, configured in
`config/ekf_local_odom.yaml`:

```text
world_frame=odom, odom_frame=odom, base_link_frame=base_footprint
two_d_mode=true, publish_tf=true, use_control=false
```

The filter fuses only these G2-current variables:

- `/laksa/vio/odom` (`nav_msgs/Odometry`): relative X, Y, yaw.
- `/laksa/vehicle/speed` (`geometry_msgs/TwistWithCovarianceStamped`): body Vx.

ZED is a raw VIO measurement source, configured for positional tracking,
`GEN_3`, internal IMU fusion, no area memory, no ZED TF, no ZED map TF, and
camera 3D mode. The VESC adapter is observational: actual telemetry is a
measurement, never a command. It will not publish physical measurements until
repeated physical telemetry characterizes its variance.

The BNO08X is excluded because its extrinsic is `UNKNOWN`. Vy pseudo-measurement
is disabled pending slip/replay evidence. Z, roll, pitch, and their derivatives
are intentionally not part of the planar filter.

## Body attitude gap

`robot_localization` can publish one estimator output edge, and
`robot_state_publisher` publishes mechanical static/joint transforms. Neither
creates the required dynamic planar-projection relation
`base_footprint -> base_link` while preserving the independently estimated
planar `odom -> base_footprint` edge. `base_link_output_frame` changes the
EKF's child frame; it does not provide both states. Therefore
`OFFICIAL_BODY_ATTITUDE_PROJECTION_GAP=PROVEN`. A narrowly scoped future glue
node is deferred; it is not required for G2 local-odometry qualification.

## Degraded behavior and consequences

VIO loss permits only a short prediction interval; uncertainty/freshness must
later disarm autonomy. Speed loss retains VIO pose with reduced redundancy. If
all inputs time out, the filter output must not be treated as healthy
indefinitely. G2 exposes filter diagnostics, output age, TF age, finite-value
checks, and covariance for G7 rather than implementing a custom health manager.

The global mapper/localizer remains the sole owner of `map -> odom` in later
modes. G2 starts none. This configuration has synthetic/replay readiness, but
physical sensor covariance calibration, BNO08X fusion, and body-attitude glue
remain hardware-gated.

## Rollback

Disable the V2 G2 launch (it is disabled by default) and retain the frozen
legacy runtime. No legacy mapping, Nav2, or actuation files are changed.
