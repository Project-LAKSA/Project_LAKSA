# ADR-002: REP-105 frames and authority contract

## Context

LAKSA needs planar Nav2 costmaps on floors and ramps, while VIO and 3D
reconstruction must retain body roll, pitch, height, and the measured +4-degree
ZED mount. REP-105 requires the `map -> odom -> base` chain to remain a tree
with one publisher per child. [Nav2's robot-localization guidance](https://docs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_gps/navigation2_with_gps/)
uses `base_footprint` for planar navigation; `robot_localization` can publish
the local transform with `world_frame=odom`.

## Alternatives

- **A — `odom -> base_link`, derived `base_link -> base_footprint`:** retains
  six-degree-of-freedom body state on the local-estimator edge, but makes the
  Nav2 frame a child projection and complicates the planar estimator contract.
- **B — `odom -> base_footprint`, derived `base_footprint -> base_link`:**
  makes the Nav2-facing state explicitly planar while preserving physical body
  attitude as a child residual. Sensor mounts remain attached to `base_link`.

## Decision

Choose **B**.

```text
map --GLOBAL_LOCALIZATION_MODE_SELECTOR--> odom
odom --LOCAL_STATE_ESTIMATOR, planar--> base_footprint
base_footprint --BODY_ATTITUDE_PROJECTION_ADAPTER--> base_link
base_link --robot_state_publisher, static--> chassis / axles / ZED / LiDAR
base_link -> imu_link: absent until measured
```

`base_footprint` is the ground-plane navigation origin. `base_link` is the
semantic rigid vehicle-body root. The future body-attitude adapter is the only
publisher allowed to derive their dynamic relationship. It is not an URDF fixed
joint. `robot_state_publisher` owns only mechanical static edges and cannot
publish `map`, `odom`, or a navigation-state edge.

Mode ownership is explicit: mapping has one mapping-system `map -> odom`
authority; navigation has one localization-system authority; 3D reconstruction
has no Nav2 `map -> odom` authority. ZED is a VIO measurement source, never the
vehicle root or a V2 odometry TF authority.

## Consequences and rollback

Synthetic ±10-degree pitch and small-roll tests require Nav2's frame to remain
zero-roll/zero-pitch while the body residual and sensor mount are preserved.
Loss or duplication of any dynamic authority disarms autonomy. This ADR does
not alter the recovered legacy mapping TF topology; rollback is achieved by not
launching V2 publishers.
