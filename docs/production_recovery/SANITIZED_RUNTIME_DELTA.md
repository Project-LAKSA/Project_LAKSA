# Sanitized Jetson runtime divergence delta

## Security boundary

This report is a semantic audit aid, not a source backup. The compared Jetson
workspace and install files remain private evidence. Their raw bodies, diffs,
comments, environment values, and unrelated strings are not in this repository.
Hashes identify the exact private evidence without disclosing it. `REDACTED` in
this report means no additional raw detail is safe or needed for the stated
audit question.

Comparison was read-only against a private, disposable local analysis copy.
`<JETSON_WORKSPACE_SRC>` and `<JETSON_INSTALL_SHARE>` are logical locations;
no host-specific path is needed to interpret the findings. File timestamps were
not used to infer causal order because source checkout, workspace, and install
copy times are not a reliable deployment chronology.

## Summary

Nineteen source/configuration or package-metadata files differ across four
LAKSA packages. Three source-like installed artifacts (`laksa_mapping` RTAB
configuration and launch plus dashboard launch) match the divergent workspace
versions, proving that divergence reached the installed runtime for those
specific artifacts. The remaining installed equivalents were either not
source-like share artifacts or were unavailable for direct source comparison.

The critical semantic delta is a topology change, not formatting: the private
workspace mapping stack is a compact ZED RGB-D/VIO RTAB-Map topology, while the
production-mainline source describes a fused topology that includes validated
LiDAR scan input and a fused odometry path. This report does **not** decide
which topology was physically qualified.

## File findings

### RUNTIME_DIFF_01 — RTAB-Map common configuration

- Component: RTAB-Map configuration.
- Evidence: repository `60a10a0b…0946ae`; workspace/install
  `041e1fab…0b96b2`.
- Semantic delta: the declared RTAB body frame differs; the workspace uses the
  ZED camera frame whereas production-mainline uses the planar vehicle frame.
  The workspace also uses a coarser depth decimation and omits the explicit
  3-DoF registration constraint present in production-mainline.
- Relevance: mapping **CRITICAL**, odometry **HIGH**, TF **HIGH**.
- Confidence: **CONFIRMED**. Raw content published: **NO**.

### RUNTIME_DIFF_02 — Mapping launch topology

- Component: mapping launch.
- Evidence: repository `d6d60c79…439b3b`; workspace/install
  `e624b4d6…a1b2`.
- Semantic delta: production-mainline declares a validated LiDAR scan input,
  RGB-D synchronization, fused local odometry, a vehicle-pose adapter, and an
  EKF participant before RTAB-Map. The workspace launch starts a ZED RGB-D
  synchronizer and RTAB-Map in a ZED-specific namespace and consumes raw ZED
  odometry; it has no corresponding validated-scan remap, fused-odometry
  remap, EKF launch, or vehicle-pose adapter.
- Relevant interfaces: `/laksa/lidar/scan_validated`,
  `/laksa/odometry/fused`, `/zed/zed_node/odom`, RGB-D image/depth/camera-info
  streams, and the two RTAB namespaces described above.
- Relevance: mapping **CRITICAL**, odometry **CRITICAL**, TF **HIGH**.
- Confidence: **CONFIRMED**. This is the strongest private clue to the lost
  multimodal integration; it is not proof that either version was the final
  good physical runtime.

### RUNTIME_DIFF_03 — Mapping session manager

- Component: mapping session lifecycle and Field Lab mapping telemetry.
- Evidence: repository `ddf689d9…d27c2a`; workspace `b8ffc67a…f687f`.
- Semantic delta: production-mainline observes validated LiDAR, RGB-D, raw ZED
  odometry, fused odometry, RTAB progress, map and cloud outputs across a
  fused mapping namespace. The workspace manager observes only the ZED RGB-D,
  raw ZED odometry and ZED-RTAB map/cloud path. Its map saving and readiness
  behavior follow that ZED-specific namespace rather than the fused policy.
- Relevance: mapping **CRITICAL**, odometry **HIGH**, Field Lab **HIGH**,
  session lifecycle **HIGH**.
- Confidence: **CONFIRMED**. Raw content published: **NO**.

### RUNTIME_DIFF_04 — Drive supervisor configuration and implementation

- Component: supervised command boundary.
- Evidence: config repository `c75a48f…40b09`, workspace `8bead6d8…76fc`;
  implementation repository `3dba9b05…ebfc`, workspace `2d4cce2a…c5ea`.
- Semantic delta: the private workspace retains a disabled-by-default,
  lower-priority characterization request/enable interface with bounded
  limits and timeout/arm handling. Production-mainline has that interface
  removed. No conclusion about mapping follows from this control difference.
- Relevance: characterization **HIGH**, safety **HIGH**, mapping **NONE**.
- Confidence: **CONFIRMED**. This proves characterization-related code existed
  in the divergent workspace source; it does not prove that a characterization
  deployment overwrote the mapping or install artifacts.

### RUNTIME_DIFF_05 — Dashboard / Field Lab server and launch

- Component: Field Lab data model and launch surface.
- Evidence: server repository `2a2515bb…c249`, workspace
  `71f1cb16…4f88`; launch repository `cc554cb1…95d23`, workspace/install
  `77965317…ff0d`.
- Semantic delta: production-mainline implements session-aware Field Lab
  telemetry, canonical fused odometry checks, map/session consistency checks,
  demand-driven cloud handling, final-map loading, and live-navigation gateway
  behavior. The workspace server is a smaller ZED-RTAB map/cloud observer
  using `map -> base_footprint` lookup. The installed dashboard launch matches
  that smaller workspace launch and uses a different preview point cap.
- Relevance: Field Lab **CRITICAL**, mapping **HIGH**, TF **HIGH**,
  navigation **MEDIUM**.
- Confidence: **CONFIRMED**. This may explain a Field Lab capability/latency
  regression, but performance causality remains **UNKNOWN** without runtime
  timing evidence.

### RUNTIME_DIFF_06 — Xbox/teleoperation and drivetrain configuration

- Files: `config/xbox_drive.yaml`, `launch/xbox_drive.launch.py`, and
  `scripts/drivetrain_conversion.py` in `laksa_bringup`.
- Semantic delta: workspace uses a generic joystick-to-`/laksa/command`
  configuration and a differing launch/configuration surface; production-mainline
  has LAKSA-specific teleoperation wiring. The small drivetrain-conversion
  difference is control-path relevant but was not exercised or changed here.
- Relevance: safety **MEDIUM**, navigation **NONE**, mapping **NONE**.
- Confidence: **CONFIRMED** for divergence; **UNKNOWN** for runtime use.

### RUNTIME_DIFF_07 — Package/build/documentation metadata

- Files: `CMakeLists.txt`, `package.xml`, `setup.py`, and README/third-party
  notices across `laksa_bringup`, `laksa_dashboard`, `laksa_interfaces`, and
  `laksa_mapping`.
- Semantic delta: dependency/export/install metadata differs consistently with
  the runtime topology changes above. These files do not independently prove a
  runtime graph; they tell the auditor to check package availability and entry
  points in the frozen runtime evidence.
- Relevance: startup **MEDIUM**, mapping **MEDIUM**, all other runtime roles
  **LOW**.
- Confidence: **CONFIRMED** for hash divergence.

## Findings requested for the next audit

| Question | Sanitized result | Confidence |
| --- | --- | --- |
| LiDAR input to RTAB differs? | Yes: the workspace launch lacks the validated LaserScan remap retained by production-mainline. | CONFIRMED |
| `Grid/Sensor` differs? | No difference was found in the divergent private files. Historical final-state value remains unresolved. | CONFIRMED / UNKNOWN historical state |
| RF2O differs? | No private file delta references RF2O. Historical runtime use remains unresolved. | CONFIRMED / UNKNOWN historical state |
| EKF differs? | Yes: production-mainline mapping launch includes an EKF participant and fused odometry; workspace launch does not. | CONFIRMED |
| RTAB odometry source differs? | Yes: fused odometry versus raw ZED odometry. | CONFIRMED |
| TF authority differs? | Yes: declared RTAB body frame and vehicle-pose/EKF topology differ. Actual runtime publisher authority still needs ros-graph/TF evidence. | CONFIRMED / UNKNOWN runtime authority |
| Field Lab differs? | Yes: the data sources and lifecycle/consistency behavior differ. | CONFIRMED |
| Characterization overwrote mapping? | Inconclusive. Characterization remnants exist in the divergent supervisor, but no causal deployment record ties them to mapping replacement. | INCONCLUSIVE |

## Audit use

The auditor should combine this delta with `PRE_CHARACTERIZATION_EVIDENCE_MATRIX.md`,
the September 2 runtime captures, the September freeze manifest/hashes, and
the recovery branch. Do not restore a private file by hash or assumption. The
next decision must be an evidence-backed, minimal mapping/odometry/Field Lab
restoration plan.
