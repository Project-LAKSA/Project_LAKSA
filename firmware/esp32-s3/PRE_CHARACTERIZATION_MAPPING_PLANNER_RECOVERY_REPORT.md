# LAKSA pre-characterization mapping + planner recovery

Status: forensic composite reconstructed; deployment and stationary runtime
acceptance remain pending. No physical motion, traction, steering excitation,
characterization, VESC testing, or direct actuator publication was performed.

## Executive summary

The strongest reproducible mapping state is not a single post-baseline commit.
The evidence supports a composite built from `b178a91` plus pre-characterization
planner/Field Lab work present in the Jetson checkout before 2026-09-17.

The recovered mapping authority is intentionally the earlier coherent chain:

```text
ZED 2i GEN_3 + IMU fusion -> /zed/zed_node/odom
        -> odom -> zed_camera_link -> base_footprint
        -> RTAB-Map map -> odom
        -> /map and Field Lab -> Nav2 costmaps -> SmacPlannerHybrid -> Path
```

The composite does not promote the later fused-EKF launch, `laksa_odom` frame
family, `publish_tf=false`, `area_memory=true`, or AUTO tracking state into the
historical recovery target. Those are separately identified later states.

## Timeline and characterization boundary

| Window | Evidence | Interpretation |
|---|---|---|
| 2026-09-05/06 | `3f8aa2d`, `b8ed0a3`, `19bbe45`, `b178a91` | A028 visualization, mapping lifecycle, dashboard and cockpit |
| 2026-09-07 | Jetson ZED/URDF timestamps | GEN_3, IMU fusion, no area memory, measured camera pitch |
| 2026-09-09 | `laksa_planning_lab`, A029 planning preview, REEDS_SHEPP configs | planner work before characterization |
| 2026-09-11/14 | mapping/dashboard/session/test files | pre-characterization mapping and planning improvements |
| 2026-09-17 | `drive_supervisor`, `characterization_*`, supervised runners | first capability to alter normal production runtime |

`CHARACTERIZATION_BOUNDARY=2026-09-17`: the first Jetson source state with
production-installed characterization request/enable paths and runners. This
is a capability boundary, not merely the first filename containing the word.

## Mapping reconstruction

The exact b178 files are retained for the mapping core:

- `jetson/laksa_mapping/config/indoor_live_zed.yaml`
- `jetson/laksa_mapping/config/rtabmap_common.yaml`
- `jetson/laksa_mapping/launch/mapping_stack.launch.py`
- `jetson/laksa_description/urdf/laksa_visualization.urdf`

Their fingerprints match the historical clues: ZED `GEN_3`, IMU fusion,
gravity as origin, `area_memory=false`, `floor_alignment=false`,
`two_d_mode=false`, ZED TF enabled, RTAB `map->odom`, direct ZED odometry input,
and the measured `0.06981317007977318` rad mount pitch. Direct IMU-to-RTAB was
not added.

The later `rtabmap_fused.yaml`, `flat_ground_fusion.yaml`, base-pose adapter,
and fused-EKF launch remain forensic evidence, not the recovery path. They
introduce a second odometry authority and require separate review.

## Planner reconstruction

The branch includes the pre-characterization `laksa_planning_lab` package,
planning preview configuration, Nav2 Ackermann configuration, and planning
action tree. Evidence supports `SmacPlannerHybrid`, `REEDS_SHEPP`, minimum
turning radius `1.09 m`, wheelbase `0.324 m`, and footprint approximately
front `0.419 m`, rear `0.149 m`, left/right `0.148 m`.

This is the recovery planner state. A forward-biased DUBIN-primary change is
explicitly not included.

## Startup and safety

The branch includes a manual-control service contract and a single
`drive_supervisor` command arbiter. It starts disarmed with actuation disabled;
all Nav2 nodes are opt-in. The supervisor is the only final command publisher.
Safety predicates are named `production_safety.py`; characterization requests
are rejected by this recovery build, and runners, campaign runners, stage
runners, and deployers are not included.

## Validation and blockers

- Git source reconstruction: PASS locally.
- Static YAML/launch review: PASS for included files.
- Current Jetson source/install hashes and runtime evidence are preserved under
  `/private/tmp/laksa-forensic-20260921` and in the accompanying JSON.
- Current Jetson source was dirty; it was not overwritten.
- Remote ROS graph/rate capture was inconclusive because the shell did not join
  the running discovery context. No runtime claim is made from that capture.
- No deployment, service restart, repeated reset test, or planner action test
  has been performed.

The VESC is physically disconnected and is `EXPECTED_DISCONNECTED`, not a
software acceptance failure. Xbox is optional for stationary validation.

The next safe gate is an isolated Jetson build/install, source/build/install
hash comparison, five software-only restart cycles, and planner-only action
tests. Only after those pass should the single physical mapping procedure be
given to the operator. Physical quality remains pending until then.

## Architecture snapshot

```text
sensors: ZED / LiDAR / IMU
    -> localization and odometry
    -> unique TF graph
    -> RTAB-Map
       +-> occupancy/map -> Field Lab
       +-> map -> Nav2 costmaps
             -> SmacPlannerHybrid -> planned Path
```

## Final status snapshot

```text
CHARACTERIZATION_BOUNDARY=2026-09-17/source capability boundary
HISTORICAL_GOOD_STATE_EXACT_COMMIT=NONE
RECOVERY_TARGET_TYPE=COMPOSITE_RECONSTRUCTION
RECOVERY_BRANCH=recovery/pre-characterization-mapping-planner-good
RECOVERY_HEAD=TO_BE_FILLED_AFTER_FINAL_SAFETY_COMMIT
JETSON_DEPLOYED_HEAD=NONE
JETSON_HASH_MATCH=NO
MAPPING_RUNTIME_RESTORED=NO
ZED_TRACKING=FAIL
RTAB_RUNTIME=FAIL
TF_TOPOLOGY=FAIL
REPEATED_SESSION_RESET=FAIL
PLANNER_RUNTIME_RESTORED=NO
SMAC_PLANNER=PASS
COMPUTE_PATH_TO_POSE=FAIL
IS_PATH_VALID=FAIL
CONTROLLER_VALIDATION=OUT_OF_SCOPE
CHARACTERIZATION_ENABLED=NO
VESC_STATUS=EXPECTED_DISCONNECTED
SAFE_NO_MOTION_VALIDATION=FAIL
CHARACTERIZATION_MAPPING_REGRESSION=INCONCLUSIVE
PHYSICAL_MAPPING_QUALITY_REVALIDATED=PENDING
BLOCKER=No isolated Jetson deployment/runtime validation has been performed; current installed workspace is dirty and the remote ROS shell did not observe the running discovery context.
```
