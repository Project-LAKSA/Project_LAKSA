# LAKSA Navigation V2 — G1 implementation handoff

## Scope and result

G1 establishes one human-authored vehicle contract:
`jetson/laksa_navigation_v2/config/vehicle_contract.yaml`. The file uses JSON
syntax, which is a valid YAML subset, so its dependency-free compiler can use
the Python standard library on CI and the Jetson. Its companion formal schema
is `config/vehicle_contract.schema.json`.

`generate_contract_artifacts.py` compiles the contract deterministically into
Nav2 canonical/padded footprints, directional kinematics, State Lattice/MPPI,
ros2_control, Gazebo V004 inputs, URDF Xacro properties, TF authority contract,
the generated manifest, and `VEHICLE_GEOMETRY_SOURCE_AUDIT.json`.

## Steering and radius evidence

The historical ESP32 Kconfig calls the 0.523-rad quantity a road-wheel limit.
The recovered Jetson control math derives `atan(wheelbase * curvature)` before
clipping to asymmetric `left_road_wheel_limit_rad` / `right_road_wheel_limit_rad`,
then normalizes it to servo travel. G1 therefore records 0.523 rad left and
0.288 rad right as **equivalent bicycle steering angles**, not individual
Ackermann wheel angles or servo-horn angles.

With the 0.324-m wheelbase, G1 derives:

- left geometric radius: 0.5619612805670269 m;
- right geometric radius: 1.0937226373133722 m;
- planner-safe symmetric radius: 1.0937226373133722 m.

The selection policy is the maximum directional geometric radius until actual
effective radii are measured. The historical 0.90-m value remains explicitly
classified as `LEGACY_ASSUMPTION_NOT_ACTIVE_V2`; it is not a V2 planner fact.
No replay artifact in the checked-in V004/characterization material contained a
jointly meaningful speed, yaw-rate, steering, and direction sample suitable to
estimate `abs(v / yaw_rate)`. Both measured effective radii remain pending
physical measurement.

## TF decision

G1 selects Option B, documented in ADR-002:

```text
map -> odom -> base_footprint -> base_link -> {ZED, LiDAR, mechanical links}
```

The local estimator will own the planar `odom -> base_footprint` edge in G2.
A future body-attitude projection adapter will own the dynamic
`base_footprint -> base_link` residual. `robot_state_publisher` owns only
static mechanical/sensor edges. The IMU mounting transform remains absent and
explicitly unknown. Mapping/localization mode owns exactly one `map -> odom`;
3D reconstruction has no Nav2 `map -> odom` authority.

## Offline qualification

The suite includes contract/schema semantics, generated-artifact consistency,
URDF property verification, sensor extrinsics, duplicate/loop/root/missing-edge
negative TF tests, and synthetic ±10-degree pitch plus roll projection tests.
`launch/g1_offline_tf_harness.launch.py` is test-only and disabled by default;
it has no sensors, Nav2, or motion publisher.

Read-only audit commands:

```bash
git switch --detach github/architecture/navigation-v2
PYTHONPATH=firmware/esp32-s3/jetson/laksa_navigation_v2 \
  python3 -m unittest discover -s firmware/esp32-s3/jetson/laksa_navigation_v2/test -v
PYTHONPATH=firmware/esp32-s3/jetson/laksa_navigation_v2 \
  python3 -m laksa_navigation_v2.generate_contract_artifacts --check
git diff --exit-code
```

## Known limits and next gate

No hardware was connected and no sensor, actuator, planner, or controller was
started. G1 creates no Nav2 patches and does not modify frozen legacy branches.
The exact next gate is **G2_LOCAL_STATE_ESTIMATION**: configure and qualify the
local estimator against offline data while preserving the G1 TF contract.
