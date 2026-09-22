# LAKSA 3D mapping validation record

Status: **physical replay dataset and user-present validation still required**
Last updated: 2026-09-13 (America/Chicago)

## Frozen product contracts

- Canonical mapping remains one ZED RGB-D stream plus validated A2M12 LaserScan and `/laksa/odometry/fused` into one RTAB-Map instance.
- The accepted 2D occupancy path is untouched. `Grid/NoiseFilteringRadius` remains `0.0` in `rtabmap_fused.yaml`.
- `Reg/Force3DoF` is not set in the active fused profile; ZED `two_d_mode` is not enabled. Legacy/shadow profiles that contain planar settings are not canonical runtime inputs.
- CycloneDDS, `map -> odom -> base_footprint`, wheelbase `0.324 m`, and minimum turning radius `1.09 m` remain unchanged.
- Manual Xbox authority remains above autonomy. No physical autonomous movement was performed in this work.

The architecture is consistent with RTAB-Map's official RGB-D plus LaserScan robot-mapping example. RTAB-Map's official `MapsManager` also confirms that assembled map products are subscriber-driven and that graph changes can cause cloud regeneration: [robot mapping demo](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/robot_mapping_demo.launch.py), [MapsManager source](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_util/src/MapsManager.cpp).

## Immutable evidence available before the golden bag

| Artifact | Early snapshot | Mature snapshot |
|---|---:|---:|
| Snapshot | `20260913T154554Z` | `20260913T223925Z` |
| Duration | 86.4 s | 239.4 s |
| Database nodes | 65 | 103 |
| Exported PLY points | 11,648 | 398,752 |
| Live cloud points at final metadata sample | 3,318 | 126,313 |
| RTAB processing mean | 323.317 ms | 1,740.662 ms |
| RTAB processing max | 440.384 ms | 3,686.966 ms |
| Proximity-by-time latest | 0.036 ms | 1,774.638 ms |
| PGM SHA-256 | `8e82fa34...55d326` | `dc277e61...d39a26` |
| PLY SHA-256 | `7a7f0cdd...924ae` | `51772ef7...9caee` |
| DB SHA-256 | `4e4f4635...1d658` | `53c67d8b...64d585a` |

These numbers prove that the multi-second slowdown reaches RTAB processing; it is not only browser rendering. They do not distinguish map-size, revisit/proximity, graph rebuild, or scene/tracking effects because the sessions are not replayable from source data.

The mature log contains:

- ZED: `Gravity alignment issues detected. Recomputing alignment...`
- RTAB MapsManager: `Graph has changed! The whole cloud is regenerated.`
- repeated `Memory.cpp::forget(): Less signatures transferred...`
- rejected loop constraints `98 <-> 18` and `100 <-> 21`. Existing `RGBD/OptimizeMaxError=3` rejected them at maximum error ratios approximately 11.96 and 11.63. This is evidence that the current official guard worked; it is not evidence for tightening or enabling robust optimization.

No global loop-closure, optimizer, gravity, cloud-subtraction, node-filter, ZED generation, or 2D occupancy parameter was changed.

## Frozen-cloud sampling experiment

Reproduction command:

```bash
python3 jetson/tools/analyze_ply_sampling.py \
  /home/ubuntu/laksa_maps/snapshots/20260913T223925Z/map.ply \
  --package-root /home/ubuntu/laksa_ws/src/laksa_dashboard \
  --output /home/ubuntu/laksa_diagnostics/mapping_3d/20260913T223925Z_sampling.json
```

The input is the immutable 398,752-point mature PLY. All methods are deterministic. The JSON payload estimate models six rounded numeric coordinates per point; it is comparative, not a measured websocket byte count.

| Strategy / budget | Sampling time | Occupied 5 cm voxels | Full-cloud nearest-distance p95 | Estimated compact JSON |
|---|---:|---:|---:|---:|
| Current order-spanning / 15k | 0.292 ms | 13,188 | 0.1378 m | 0.63 MB |
| Current / 30k | 0.153 ms | 23,678 | 0.1039 m | 1.26 MB |
| Current / 60k | 0.241 ms | 40,237 | 0.0766 m | 2.52 MB |
| Adaptive voxel / 15k | 368.072 ms | 14,778 | 0.1167 m | 0.63 MB |
| Adaptive voxel / 30k | 381.890 ms | 29,111 | 0.0864 m | 1.26 MB |
| Morton stratified / 15k | 94.397 ms | 13,545 | 0.1288 m | 0.63 MB |
| Morton stratified / 30k | 96.638 ms | 24,405 | 0.0970 m | 1.26 MB |

Whole-cloud RANSAC bands are intentionally recorded as provisional, not labeled walls: the frozen artifact has no scene annotation/crops and the three strongest models were near-horizontal. For the strongest band, the full source RMS was `0.073888 m`, p95 absolute residual `0.134370 m`, and robust 5–95% thickness `0.242222 m`. The current deterministic 15k sample measured `0.073669 m`, `0.133516 m`, and `0.241411 m`, respectively. The sample therefore preserves the source band thickness closely; it is not manufacturing that thickness.

Decision: keep the 15k current sampler until live browser gates are available. Adaptive voxel is rejected for runtime because 368 ms of Python work is incompatible with the source-to-render pose p95 limit of 150 ms. Morton is not selected: ~94 ms cost buys only modest coverage and has no demonstrated plane-quality benefit. Browser FPS/CPU/render time and post-change pose p95 are `NOT_RUN` because there is no active mapping replay or connected browser.

## Visualization critical-path separation

Before this work, Field Lab permanently subscribed to `/laksa/fused_mapping/cloud_map`, even with zero Field Lab clients. In the installed architecture that subscription can request assembled-cloud work inside RTAB MapsManager.

Field Lab now creates exactly one preview subscription only when all are true:

1. a Field Lab websocket client exists;
2. mapping is `STARTING`, `MAPPING`, or `DEGRADED`;
3. the selected preview source matches the mapping mode.

It destroys the subscription otherwise. Messages remain throttled, latest-only, coalesced, and droppable. Pose uses its separate websocket, latest-value path, and callback group. This eliminates known idle coupling; quantitative subscriber OFF/ON replay remains `NOT_RUN` until the golden bag exists.

Official MapCloud uses map data with decimation, voxel, and node filtering controls and remains the required independent renderer for the early/mid/late comparison: [official RViz MapCloud configuration](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_launch/launch/config/rgbd.rviz).

## Installed cloud-subtraction semantics

Installed packages are `rtabmap_ros`, `rtabmap_slam`, and `rtabmap_util` version `0.22.1`. The installed default is `cloud_subtract_filtering=false` with minimum-neighbor semantics `2`. In upstream `MapsManager`, the subtract filter operates on assembled ground/obstacle cloud contributions, while occupancy-grid update occurs separately. That is enough to define a replay candidate, not enough to deploy it: byte identity and occupied/free/unknown equality of `/map` must still be proven on the same input. `map_filter_radius` and `map_filter_angle` remain zero/disabled on the canonical process for the same reason.

## Gravity and ramp status

- `Reg/Force3DoF`: false/not set in the active fused profile.
- ZED `two_d_mode`: false/not enabled.
- ZED `set_gravity_as_origin`: true in the mature session log.
- `Optimizer/GravitySigma`: not explicitly set in the active captured config; effective runtime/default and graph-link count have not been proven.
- `Mem/UseOdomGravity`: not explicitly active.
- Incoming RTAB odometry is `/laksa/odometry/fused`, incorporating ZED VIO, but gravity alignment suitability and actual gravity constraints are not yet proven.
- Controlled ramp recovery: `NOT_RUN`.

The gravity warning makes VIO/gravity a live hypothesis, not a tuning authorization. Do not enable `Mem/UseOdomGravity`, ZED `two_d_mode`, or `Reg/Force3DoF` without the recorded ramp A/B.

## Snapshot, planner, Xbox, Follow, and LiDAR closure

The failed mature snapshot navigation log shows lifecycle manager configuration of `planner_server` failing immediately while the planner took about 7.7 s to finish configuration. At the same time the noncanonical `laksa-planning-preview.service` was active and owned duplicate planner/controller node names. That service is now stopped and disabled. Cockpit also refuses to start saved navigation if it becomes active again.

Snapshot transition now:

- rejects a pre-shutdown map pose older than 0.5 s;
- seeds official `/initialpose` once with 5 cm / 2 degree covariance scales;
- requires three fresh AMCL samples within 0.05 m and 2 degrees before `READY`;
- fails closed for a jump over 0.15 m or 5 degrees;
- uses no custom `map -> odom` shim.

Physical stationary delta validation is `NOT_RUN`.

Follow Route now presents `START AUTONOMOUS ROUTE?`, maximum speed `0.15 m/s`, and `Xbox input immediately overrides.` The backend requires the product confirmation, performs internal supervisor arming/preflight, and disarms on failure/cancel/completion. No separate ARM concept is exposed.

The displayed lidar is the actual `/laksa/lidar/scan_validated`, transformed at its timestamp into `map`, latest scan only at 5 Hz, now bright fuchsia (`0xd946ef`) with a distinct point size. Physical visual/latency validation is `NOT_RUN`.

Xbox was paired/trusted but physically disconnected during this work. The reconnect service, joy path, supervisor, and hardware adapter remained running; end-to-end manual actuation in mapping and navigation is `NOT_RUN` and is mandatory before any autonomous route test.

## Golden dataset procedure

No qualifying source rosbag exists. With Leo physically present and Xbox manual control proven, start normal mapping and run:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/laksa_ws/install/setup.bash
python3 /home/ubuntu/src/Project_LAKSA/firmware/esp32-s3/jetson/tools/record_mapping_golden.py
```

Mark `early`, `mid`, `kitchen`, `ramp_start`, `ramp_end`, and `return`; type `stop` to finalize. The script records source RGB/depth/camera info, ZED and fused odometry, ZED IMU, validated LaserScan, TF, RTAB info/mapData, occupancy, state/safety context, active parameter dumps, versions, git state, event timestamps, and SHA-256 sums under `/home/ubuntu/laksa_datasets/mapping_golden_<UTC>/`.

Afterward, replay the identical bag for subscriber OFF/ON, official MapCloud/source/Field-Lab screenshots at the same checkpoints, manually cropped floor/wall A/wall B plane metrics, cloud subtraction false/true, and only evidence-justified gravity or graph candidates.

## Acceptance matrix

| Requirement | Result |
|---|---|
| Accepted 2D implementation/config unchanged | PASS (no mapping-path edits) |
| 2D replay byte/quality equivalence | NOT_RUN |
| Early 3D sharp | PASS (user-observed; early artifact frozen) |
| Late 3D close to early, no unexplained duplicates | NOT_RUN / unresolved |
| Large-map pose source-to-render p95 <=150 ms | NOT_RUN (last accepted baseline 124.86 ms) |
| No unhandled corrupting graph correction | PARTIAL: two bad constraints rejected; full timeline pending |
| Ramp pitch recovery | NOT_RUN |
| Snapshot stationary delta <=0.05 m / <=2 degrees | NOT_RUN; gate implemented |
| Xbox manual in mapping and navigation | NOT_RUN; controller disconnected |
| Follow UX and internal preflight | PASS (27 changed-component tests); physical route NOT_RUN |
| Violet validated lidar visible with latency gate | code PASS; live visual/latency NOT_RUN |

The immutability fixture had stale hashes for the accepted ZED profile and active mapping launch. It was rebased to the current artifacts (`4e465...` and `a6bee...`) after confirming all other protected hashes matched and both frozen snapshots recorded the current ZED hash. Only the test oracle changed; mapping inputs did not.

Final runtime observed after deployment: mapping `IDLE`, vehicle velocity `0.0`, VESC brake active, Xbox disconnected, noncanonical preview service inactive/disabled, and no planner/controller preview process. Physical validation must begin with manual Xbox recovery, never with autonomy.
