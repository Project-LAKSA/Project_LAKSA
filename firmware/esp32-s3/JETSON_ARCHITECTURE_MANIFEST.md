# LAKSA Jetson production architecture manifest

Baseline: `b178a91f3a87f90ae289f62588744bd7936a43f4`  
Recovery branch: `recovery/jetson-production-stable`

This manifest describes the committed production source baseline. Runtime
observations are labelled separately; a live observation does not silently
promote an installed or dirty checkout into source authority.

## Baseline versus installed runtime

The recovery checkout was created directly from the Jetson's committed
`b178a91f3a87f90ae289f62588744bd7936a43f4`. The Jetson source checkout and
installed workspace were dirty during the audit. In particular, the installed
workspace contained a later `drive_supervisor.yaml` and later mapping files
whose SHA-256 values did not match this clean checkout; the clean baseline
contains the older `xbox_drive.yaml` contract instead. Those later files were
not copied into this branch without an evidence-backed review. No deployment
or production restart was performed as part of recovery staging.

This distinction is intentional: this branch is a reproducible committed
baseline and architecture-review target, while runtime acceptance remains
open until the later installed changes are adjudicated and the live sensor and
actuator health gates pass.

## Package inventory

| Package / area | Source path | Responsibility |
|---|---|---|
| `laksa_bringup` | `jetson/laksa_bringup` | Xbox input, drive supervisor, manual/control launch contracts |
| `laksa_mapping` | `jetson/laksa_mapping` | Mapping session ownership and mapping launch |
| `laksa_dashboard` | `jetson/laksa_dashboard` | Field Lab HTTP/WebSocket UI and session API |
| `laksa_description` | `jetson/laksa_description` | Visualization URDF and vehicle dimensions |
| `laksa_health` | `jetson/laksa_health` | Health aggregation and diagnostics |
| `laksa_interfaces` | `extra_ros_packages/laksa_interfaces` | ROS messages/services shared with the ESP32 |
| ESP32 firmware | `src`, `components`, `sdkconfig.defaults` | VESC, PCA9685, BNO08X and micro-ROS transport |

Generated `build/`, `install/`, `log/`, bags, maps, and physical evidence are
not production source and are intentionally absent from this branch.

## Production runtime modes

The committed architecture has one manual mapping/control path. Field Lab owns
mapping session lifecycle; it does not publish actuator commands. The canonical
motion path is:

```text
Xbox / joy/game_controller_node
        -> /joy
        -> /drive_supervisor
        -> /laksa/command + /laksa/brake
        -> micro-ROS Agent / ESP32-S3
        -> VESC + PCA9685 steering
```

Mapping uses the ZED 2i, validated LiDAR, robot-localization, RGB-D sync and
RTAB-Map. The intended authority boundary is:

```text
ZED odometry -> fusion odometry -> RTAB-Map -> /map -> /tf map->odom
robot state publisher -> odom/base and sensor static transforms
```

The exact installed launch/config must be hash-checked against this branch
before deployment. This branch does not authorize automatic deployment.

## Topic ownership

| Topic | Owner | Role |
|---|---|---|
| `/joy` | `joy_node` | Operator input |
| `/laksa/command` | `drive_supervisor` | Sole final drive command |
| `/laksa/brake` | `drive_supervisor` | Brake/e-stop output |
| `/laksa/state` | ESP32 micro-ROS bridge | Vehicle state |
| `/laksa/vesc/state` | ESP32 micro-ROS bridge | VESC telemetry/fault state |
| `/laksa/pca9685/state` | ESP32 micro-ROS bridge | Steering driver state |
| `/laksa/imu/data` | ESP32 micro-ROS bridge | IMU data |
| `/laksa/lidar/scan_validated` | LiDAR validation boundary | Mapping scan input |
| `/zed/zed_node/odom` | ZED wrapper | Visual-inertial odometry input |
| `/laksa/odometry/fused` | `robot_localization` EKF | Fused odometry input to RTAB |
| `/map` | RTAB-Map | Mapping output |

No characterization package, deployer, or analyzer is a production publisher
in this baseline. Any later characterization tooling must remain observer-only
or use the explicitly gated supervisor request interface.

## TF ownership

- RTAB-Map owns `map -> odom` during mapping.
- The production robot-description/state-publisher path owns the fixed sensor
  and vehicle transforms.
- The fused odometry path owns `odom -> base_footprint`.
- ZED internal camera transforms are owned by the ZED wrapper's URDF publisher.
- There must be exactly one publisher for each dynamic edge and one consistent
  frame family: `map`, `odom`, `base_footprint`.

## Startup dependencies

1. Device rules claim `/dev/laksa_microros` and `/dev/laksa_lidar`.
2. micro-ROS Agent establishes the ESP32 session.
3. `laksa-control-navigation` / manual control starts the supervisor and Xbox
   input path.
4. LiDAR validation/guard starts and exposes the validated scan.
5. Field Lab starts the mapping stack only for an explicit mapping session.
6. ZED, fusion, RGB-D sync and RTAB-Map become active under the session owner.
7. Dashboard and health monitor expose state; they do not become motion owners.

## Safety ownership

`drive_supervisor` is the only final command arbiter. E-stop, stale Xbox,
stale ESP32/VESC data, VESC fault and disabled characterization gate must force
neutral/brake behavior. The installed live audit on 2026-09-21 observed vehicle
velocity 0, brake active and autonomy disarmed; it also observed Xbox
disconnected, ZED stale/standby and VESC health `ERROR`, so production was not
declared accepted from that observation.

## Reproducibility

Source branch HEAD is the baseline commit above. The exact source/config paths
must be deployed through an operator-reviewed process and then compared by
SHA-256 with the installed workspace. Do not commit generated install/build
trees or physical characterization evidence into this production branch.
