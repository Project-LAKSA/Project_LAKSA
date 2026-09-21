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
- Isolated Jetson `colcon build`: PASS, 8 packages.
- Source-to-isolated-install SHA-256: PASS for the ZED config, RTAB config,
  mapping launch, Nav2 planner config, and supervisor config.
- Static mapping launch resolution: PASS when the existing `/home/ubuntu/zed_ws`
  install is sourced; no node was started.
- Static planner and Field Lab launch-argument resolution: PASS; no node was
  started.
- Current Jetson source/install hashes and runtime evidence are preserved under
  `/private/tmp/laksa-forensic-20260921` and in the accompanying JSON.
- Current Jetson source was dirty; it was not overwritten.
- Remote ROS graph/rate capture was inconclusive because the shell did not join
  the running discovery context. No runtime claim is made from that capture.
- No production deployment or service restart, repeated reset test, or planner
  action test has been performed. The isolated build/install is a deployment
  rehearsal, not a production activation.

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
RECOVERY_HEAD=ab82b0cba7b320f26986b94b3d6175791b735e8a
JETSON_DEPLOYED_HEAD=ab82b0cba7b320f26986b94b3d6175791b735e8a (isolated workspace only)
JETSON_HASH_MATCH=YES (isolated source/install subset)
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
SAFE_NO_MOTION_VALIDATION=PASS (isolated build only; no runtime nodes started)
CHARACTERIZATION_MAPPING_REGRESSION=INCONCLUSIVE
PHYSICAL_MAPPING_QUALITY_REVALIDATED=PENDING
BLOCKER=Production runtime was not switched; five reset cycles and planner action tests remain pending, and current live sensor state still cannot prove mapping quality.
```

## Phase 2 controlled production runtime qualification (2026-09-21)

The recovered composite was deployed by source/install file identity, with a
rollback snapshot at `/home/ubuntu/laksa_phase2_rollback_20260921T100000Z`.
The prior live checkout was preserved rather than reset. Two narrow,
post-deployment recovery compatibility commits were necessary because the
historical interface intentionally lacks later fields: the supervisor no
longer accesses `DriveCommand.brake`, the VESC freshness test uses the
historical `telemetry_fresh`, and Health reports unavailable historical
`brake_active` telemetry as `null`. A separate service contract commit aligns
the cockpit with the same CycloneDDS environment already used by control and
LiDAR. None modifies mapping parameters, planner behavior, characterization,
or actuator authority.

The canonical stationary manager ran five clean sessions:
`20260921T112746Z`, `20260921T112816Z`, `20260921T112846Z`,
`20260921T112916Z`, and `20260921T112946Z`. Every session reached `COMPLETE`,
used a new database path, presented fresh ZED RGB/odometry to Field Lab, and
had no remaining `/zed/zed_node` or `/zed_rtabmap/rtabmap` graph nodes before
the next session. Evidence is retained on Jetson at
`/home/ubuntu/laksa_phase2_rollback_20260921T100000Z/reset_cycles_final_20260921T1127Z.log`.
This establishes reset behavior only; it does not establish mapping quality
without physical manual driving.

ZED and RTAB were live in each cycle. The live mapping launch process used
the session copies of `indoor_live_zed.yaml` (SHA-256
`aae8a161...141ab15`) and `rtabmap.yaml`, with RTAB explicitly remapped to
`/zed/zed_node/odom`. RTAB processed frames at its configured 1 Hz mapping
rate. The Field Lab health/API observed fresh RGB and odometry. Numerical
`tf2_echo` and live parameter-service requests from the CLI discovery context
timed out, so TF numerical authority is explicitly **BLOCKED**, not passed.

The isolated `ROS_DOMAIN_ID=71` planner lab loaded `SmacPlannerHybrid` with
the recovered `REEDS_SHEPP` configuration and returned a `ComputePathToPose`
action result for all five deterministic cases. Independent path validation
accepted one case and rejected four (three collision failures, one kinematic
violation). The official `IsPathValid` interface was not invoked and therefore
is `NOT_RUN`. These are recovery results, not controller execution; no path
was sent to the vehicle.

Machine-readable evidence is in `PRODUCTION_RUNTIME_QUALIFICATION.json`.
The authoritative current qualification conclusion is:

```text
MAPPING_RUNTIME_RESTORED=YES
ZED_TRACKING=PASS
ZED_ODOMETRY=PASS
RTAB_RUNTIME=PASS
FIELD_LAB_RUNTIME=PASS
REPEATED_SESSION_RESET=PASS
SMAC_PLANNER=PASS
COMPUTE_PATH_TO_POSE=PASS
IS_PATH_VALID=NOT_RUN
TF_TOPOLOGY=BLOCKED
PLANNER_RUNTIME_RESTORED=YES
PHYSICAL_MAPPING_QUALITY_REVALIDATED=PENDING
PRODUCTION_RUNTIME_RESTORED=NO
BLOCKER=TF numerical verification is blocked; official IsPathValid is not run; only 1/5 planner paths passes independent collision/kinematic validation.
```

## Final TF and planner-validity forensics (2026-09-21)

The attempted persistent read-only TF observer did not alter TF or mapping. A
new stationary mapping session (`20260921T120201Z`) could not open the ZED
stream: ZED SDK 5.4.1 reported `CAMERA STREAM FAILED TO START` and the
manager therefore received no camera odometry or RTAB input. This is a
hardware/runtime availability blocker, not evidence of a TF error. The
preserved manager metadata is `TF_OBSERVER_BLOCKER_20260921T120201Z.json`.

The official installed Humble interface is `nav2_msgs/srv/IsPathValid`, owned
by `nav2_planner/planner_server`. Five previously preserved action results
were replayed in an isolated `ROS_DOMAIN_ID=71`, localhost-only planner lab.
Each exact returned `nav_msgs/Path` was passed to that service and to the
independent validator. The lab anchors its sole local `map -> base_footprint`
transform at the requested start pose before calling the official service;
this matters because Humble validates only from the point nearest the current
robot pose.

Results are preserved in `PLANNER_VALIDITY_FORENSICS.json`:

| Case | ComputePathToPose | Official IsPathValid | Independent result | Primary finding |
|---|---|---:|---:|---|
| S000000 | success | pass | pass | valid |
| S000001 | success | fail | fail | actual footprint collision |
| S000002 | success | pass | fail | collision exists between discrete path poses; Humble service samples returned poses only |
| S000003 | success | fail | fail | actual footprint collision |
| S000004 | success | pass | fail | curvature below the 1.09 m independent kinematic limit; IsPathValid is collision-only |

The installed Humble source semantics explain the intentional disagreement:
the service checks returned poses from the closest one onward and uses either
the costmap center/radius or the polygon footprint; it does not check
continuous interpolation or Ackermann curvature. The initial test harness
used a false `(0,0,0)` robot pose and could skip invalid prefixes. Anchoring it
at each case's start corrected that harness defect and reduced official passes
from 4/5 to 3/5. `invalid_pose_indices` remains empty because this Humble
implementation does not populate that response field.

An isolated, non-production `smooth_path=false` experiment did not repair
S000001 or S000003; both remained official collision failures. It is preserved
as `PLANNER_VALIDITY_FORENSICS_NO_SMOOTH_EXPERIMENT.json`. Consequently no
planner, mapping, TF, or controller production configuration was changed.
The verified root cause is `PLANNER_ACTUAL_INVALID_PATH` for S000001 and
S000003, with additional validator-semantic differences for S000002 and
S000004. A 5/5 official qualification requires a separately evidenced planner
or costmap configuration correction; speculative tuning is intentionally out
of scope for this forensic recovery.

## Final closure evidence — production-aligned planner laboratory (2026-09-21)

The original planner laboratory was itself a test-harness mismatch: it used a
0.588 x 0.316 m footprint with implicit 0.01 m padding, no static-costmap
inflation, `allow_unknown=false`, and several non-production Smac values.
Only the isolated `laksa_planning_lab` was corrected. It now mirrors the
recovered production `nav2_ackermann.yaml` planner, static costmap footprint
(`[-0.15,-0.18]..[0.42,0.18]`), 0.02 m padding, 0.55 m inflation, and planner
parameters. The live published footprint measures 0.61 x 0.40 m after padding.
No production mapping or planner source was changed.

The five preserved cases were replayed in the localhost-only lab with both
Smac internal `smooth_path=true` and `smooth_path=false`. Both variants return
five paths, but each has only 2/5 official Nav2 discrete-footprint passes. The
continuous LAKSA collision layer accepts 2/5 and the independent Ackermann
layer accepts 3/5. Consequently the planner is **not qualified** for the
five-case acceptance corpus; no production planner fix was committed.

- **S000001:** start and goal are individually valid. The raw path first exits
  the map between poses 6 and 7, at 0.6181 m (`x=-0.6151, y=-2.0728,
  yaw=2.0508`), with footprint cells `(28,-1)` and `(29,-1)` outside the map.
  It remains invalid without smoothing. The isolated `allow_unknown=false`
  experiment also returned that same invalid route, so that switch is not a
  demonstrated fix.
- **S000003:** start and goal are individually valid. The raw path first
  intersects occupied cells `(31,102)` and `(32,102)` at pose 51, 4.8496 m
  (`x=-0.2409, y=2.6299, yaw=2.4435`). It is invalid before smoothing.
  Deferring analytic expansion changes the path but produces another
  out-of-map collision, so it is not a demonstrated fix either.
- **S000002:** became Nav2-invalid when the lab adopted the actual larger,
  padded production footprint; it leaves the map between returned poses.
- **S000004:** Nav2 and continuous collision checks pass, while the independent
  Ackermann check reports 0.7530 m implied radius, below the 1.09 m constraint.

Full preserved raw paths, returned paths, costmap identity, footprint data,
and experiment results are referenced in
`PLANNER_FINAL_CLOSURE_FORENSICS.json`. This separates Nav2's discrete
footprint predicate from LAKSA continuous-collision and kinematic predicates;
the independent validator was not weakened to make it agree.

The latest ZED diagnostic does not prove a TF defect. The USB identities and
`/dev/video0`/`/dev/video1` enumerate, but ZED SDK 5.4.1 reports no camera and
the wrapper reports `CAMERA STREAM FAILED TO START` followed by camera
detection timeout. No holder or permissions conflict was observed. This is
classified `CAMERA_PHYSICALLY_UNAVAILABLE`; the persistent numerical TF
observer and TF qualification remain **BLOCKED**, with evidence in
`ZED_STREAM_TF_BLOCKER_20260921T205000Z.json`. Mapping configuration and
extrinsics remain frozen.

## Smac internal collision-checker closure (2026-09-21)

`PLANNER_INTERNAL_COLLISION_FORENSICS.json` records an isolated source-level
replay against Humble Navigation2 commit `3c3db59d`. It did not build, install,
or load a Nav2 library in the production workspace.

The recovered polygon footprint is 0.61 x 0.40 m after padding. Its inscribed
and circumscribed radii are 0.1700 m and 0.4833 m. With a 0.55 m / 3.0
inflation layer, Humble `findCircumscribedCost()` returns 98.

Two distinct mechanisms explain why Smac accepted the two official-invalid
raw poses:

| Case | Exact raw pose | Smac branch | Internal result | Official result | Proven mechanism |
|---|---|---|---|---|---|
| S000001 | index 7; grid `(30.837685, 3.200006, bin 24)` | center-cost fast accept | free (`center=0 < 98`) | collision | The fast branch returns before the polygon check. The full footprint cost is 254 and the footprint leaves the map. |
| S000003 | index 51; grid `(35.682771, 92.898621, bin 28)` | full polygon check | free (`cost=253`) | collision | `Costmap2D::mapToWorld` takes unsigned cells. Smac truncates the continuous state to `(-0.275, 2.585)` for checking, while it publishes `(-0.240861, 2.629931)`; the published footprint cost is 254. |

The raw yaws equal their 72-bin orientations, so angle quantization is not
causal. Smoothing is not causal. The evidence does not require a mapping,
TF, ZED, or RTAB explanation. S000001 is a map-boundary collision; S000003 is
an occupied-cell collision at `(31,102)` and `(32,102)`.

An isolated candidate correction retained the continuous grid position during
footprint placement and suppressed the center-cost fast return for polygon
footprints. It converted S000001 and S000003 to official and continuous
passes, proving both mechanisms. It did **not** qualify the corpus: S000002
had no collision-free path, and the independent Ackermann gate still rejected
S000003 and S000004. Therefore no production planner change, configuration
tuning, mapping change, or 25-case promotion was made. The required planner
safety gate remains: official `IsPathValid`, then LAKSA continuous collision,
then LAKSA Ackermann validation; only all-pass paths may reach a future
controller.
