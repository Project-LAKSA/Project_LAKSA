# Pre-characterization evidence matrix

This is an evidence matrix, not a restoration plan. “Current” refers to
`production/laksa-mainline`; “divergent runtime” refers only to private
hash-identified evidence summarized in `SANITIZED_RUNTIME_DELTA.*`.

| Subsystem | Current production-mainline | Divergent runtime evidence | Sep-02 / Sep-13 / Sep-21 evidence | Confidence | Unresolved question |
| --- | --- | --- | --- | --- | --- |
| RPLIDAR driver | Present in LAKSA source. | No private driver-source delta. | Sep-02 runtime includes scan evidence; Sep-13 freeze has service/runtime captures. | PROBABLE | Which driver configuration fed the qualifying run? |
| LiDAR filtering | Validated-scan route declared. | Workspace mapping launch omits its RTAB remap. | Sep-02 has LiDAR mapping/filter captures. | CONFIRMED delta | Which scan topic was final? |
| LiDAR topic routing | `/laksa/lidar/scan_validated` is current mapping input. | Workspace RTAB topology has no equivalent remap. | Sep-02 graph records `/scan`; recovery contains validated-scan references. | CONFIRMED delta | Was current route ever physically qualified? |
| RF2O | Source/dependency evidence exists. | No private delta. | Sep-02 graph includes RF2O odometry; Sep-13 capture available. | PROBABLE | Was RF2O active in the best session? |
| ZED wrapper | ZED RGB-D/odometry sources declared. | Workspace is ZED-centric. | Sep-02 graph records ZED odometry and point cloud. | CONFIRMED topology | Exact wrapper configuration by date? |
| ZED VIO | Current fused path observes it. | Workspace feeds raw ZED odometry to RTAB. | Sep-02 capture records ZED odometry. | CONFIRMED delta | Was it transformed/fused before RTAB in the good run? |
| RGB-D synchronization | Current fused RGB-D sync. | Workspace has ZED RGB-D sync. | Sep-02 3D launch evidence exists. | CONFIRMED | Which namespace/process form was deployed? |
| robot_localization | Current mapping launch includes EKF. | Workspace mapping launch omits EKF. | Sep-02 has EKF parameter snapshot; recovery references it. | CONFIRMED delta | Was EKF active in final physical mapping? |
| RTAB-Map | Fused LiDAR + RGB-D intent. | ZED RGB-D/VIO-only intent. | Sep-02 has RTAB map/cloud graph; Sep-13 freeze has mapping hashes. | CONFIRMED delta | Which topology was the historically best one? |
| Occupancy generation | Current fused policy output. | Workspace saves/observes ZED-RTAB occupancy. | Sep-02 has RTAB occupancy/cloud topics. | CONFIRMED delta | Which map output drove Field Lab/Nav2? |
| 3D mapping | Current has dense/session-aware behavior. | Workspace is smaller ZED-RTAB observer. | Sep-02 point-cloud graph and Sep-13 Field Lab capture exist. | STRONG | What was release-quality versus live-preview output? |
| TF ownership | Vehicle-frame contract intended. | RTAB frame differs; adapter/EKF topology absent. | Sep-02 TF capture exists. | CONFIRMED delta | Actual publishers/ownership in good run? |
| Field Lab | Session-aware fused telemetry. | Simpler ZED-RTAB map/cloud observer. | Sep-02 dashboard artifacts and Sep-13 state capture exist. | CONFIRMED delta | What produced historically low latency? |
| Mapping session lifecycle | Fused readiness/reset policy. | ZED-only readiness/save policy. | Sep-13 session/runtime artifacts available. | CONFIRMED delta | Which reset policy avoided regressions? |
| Nav2 costmaps | Present but not changed here. | No private semantic delta. | Sep-02 parameters captured. | UNKNOWN | Which map/odom contract was consumed? |
| Planner | Present but untouched. | No private semantic delta. | Sep-02 parameters/recovery forensics exist. | UNKNOWN | No decision in this audit. |
| Controller | Present but untouched. | No private semantic delta. | Sep-02 parameters/recovery forensics exist. | UNKNOWN | No decision in this audit. |
| Drive supervisor | Active production excludes characterization. | Private workspace retains disabled characterization interface. | Sep-02 supervisor parameter capture; Sep-13 capture available. | CONFIRMED delta | Did any later deployment activate it? |
| Startup/systemd | Current service definitions preserved. | No private unit-file delta collected. | Sep-02 and Sep-13 systemd captures exist. | PROBABLE | Which service mode selected each topology? |
| DDS/runtime environment | Current source has environment guidance. | No private semantic delta collected. | Sep-13 process environments and package list exist. | UNKNOWN | Did RMW/domain settings affect latency? |

## Local-only evidence availability

The Sep-13 freeze was present during this analysis and contains manifest,
mapping-hash, runtime process/environment, systemd, ROS parameter, Field Lab,
source snapshot, and install snapshot artifacts. The Sep-21 temporary
forensic directory was not present. Neither absence nor presence selects a
mapping configuration.
