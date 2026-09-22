# LAKSA Navigation Architecture V2

Status: proposed architecture; no V2 node is production-qualified.

V2 has one robot contract, one TF authority per edge, one local odometry
contract, and a fail-closed path gate. Hardware drivers, simulation, and
offline replay supply the same ROS interfaces; they do not fork navigation
logic.

```text
ZED VIO / measured speed -> robot_localization EKF -> odom -> base_footprint
body attitude adapter  -> base_footprint -> base_link
SLAM/localization      -> one authority       -> map  -> odom
LiDAR / depth          -> Nav2 layered costmaps
costmaps -> State Lattice primary / Hybrid comparison -> candidate Path
candidate -> Nav2 IsPathValid -> swept collision -> Ackermann gate
accepted Path -> MPPI Ackermann -> Collision Monitor -> authority supervisor
                                                    -> ESP32 safety boundary
Xbox manual ------------------------------------------------> supervisor
RTAB-Map RGB-D -> 3D reconstruction -> Field Lab (observer only)
```

The legacy recovered mapping runtime stays frozen. V2 adapts it behind
contracts; it does not overwrite its launch/configuration files.

## Contracts at a glance

```mermaid
flowchart TB
  map --> odom --> base_footprint --> base_link
  base_link --> zed_camera_link
  base_link --> lidar_link
  base_link --> imu_link
```

```mermaid
flowchart LR
  ZED[ZED VIO] --> EKF[robot_localization EKF]
  IMU -. pending calibrated extrinsic .-> EKF
  Speed[vehicle speed] --> EKF
  EKF --> Odom[odom to base_footprint]
  SLAM[one SLAM/localizer] --> Map[map to odom]
```

```mermaid
flowchart LR
  Lidar --> Local[local obstacle layer]
  Depth[ZED depth optional] --> Local
  Static[static map] --> Global[global costmap]
  Global --> Planner
  Local --> Controller
```

```mermaid
flowchart LR
  Planner --> Candidate[Candidate Path]
  Candidate --> Official[Nav2 IsPathValid]
  Official --> Sweep[continuous sweep]
  Sweep --> Kinematic[Ackermann validation]
  Kinematic -->|pass| MPPI
  Official -->|fail| Reject[PATH_REJECTED]
  Sweep -->|fail| Reject
  Kinematic -->|fail| Reject
```

```mermaid
flowchart TB
  Estop[E-stop / hardware] --> Supervisor
  Xbox[Xbox manual] --> Supervisor
  Autonomy[approved autonomy only] --> Supervisor
  FieldLab[Field Lab] -. observer only .-> Supervisor
  Supervisor --> ESP32 --> Actuators
```

```mermaid
stateDiagram-v2
  [*] --> BOOT
  BOOT --> SENSOR_WAIT
  SENSOR_WAIT --> LOCALIZATION_READY
  LOCALIZATION_READY --> MAP_READY
  MAP_READY --> PATH_READY
  PATH_READY --> AUTONOMY_ARMED
  AUTONOMY_ARMED --> EXECUTING
  EXECUTING --> DEGRADED: stale / invalid / lost health
  DEGRADED --> PATH_READY: health restored
  BOOT --> E_STOP: hardware safety
  EXECUTING --> E_STOP: emergency stop
```

```mermaid
flowchart LR
  Contract[vehicle_contract.yaml] --> Loopback[Nav2 Loopback]
  Contract --> Gazebo[ros_gz V004 twin]
  Contract --> Hardware[hardware adapter]
  Loopback --> Same[Same Nav2 / BT / validators]
  Gazebo --> Same
  Hardware --> Same
```

See `docs/architecture/adr/` for decisions, `NAVIGATION_V2_INTERFACE_CONTRACTS.md`
for interfaces, and `NAVIGATION_V2_TEST_PLAN.md` for evidence gates.

## Gate order

G1 canonical robot / TF is complete. G2 supplies only local planar odometry:
one `robot_localization` EKF, raw ZED VIO pose, measured VESC Vx when
calibrated, and no global TF. G3 compares 2D mapping/localization approaches;
only then do costmaps, planner, and controller gates begin.
