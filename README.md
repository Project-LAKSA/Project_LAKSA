# Project_LAKSA
Autonomous Robot challenge

# Autonomous RC Car Design Review Document

**Document version:** v1.0
**Prepared on:** 2026-05-13
**Primary design intent:** Build a safe, testable, competition-ready autonomous 1/10-scale RC car platform that can start with simple line/wall-following behavior and grow into LiDAR/vision-based path following, obstacle avoidance, and higher-speed autonomous racing algorithms.

---

## Design Assumptions

Because the exact competition rulebook, track type, and competition date have not been provided yet, this document uses the following assumptions. These should be updated once the official rules are available.

| Item | Assumption Used for This Design Review |
|---|---|
| Competition format | Single-vehicle autonomous run or time-trial style competition. |
| Track | Indoor or controlled outdoor course; may be line-marked, wall-bounded, cone-bounded, or a small closed loop. |
| Vehicle class | 1/10-scale RC car with Ackermann steering. |
| Operating speed | Conservative initial testing below 1.0 m/s; competition tuning target 2.0–5.0 m/s depending on track size, safety rules, and sensor reliability. |
| Autonomy level | Fully autonomous after start command, with manual override and emergency stop. |
| Primary objective | Complete the course reliably before optimizing speed. |
| Expansion objective | Support future F1TENTH-style algorithms: Follow-the-Gap, Pure Pursuit, SLAM/localization, speed profiling, MPC/MPCC, and learning-based methods. |

---

# 1. Robot Design Overview

## 1.1 Overall Concept

The proposed robot is a **1/10-scale autonomous RC car** built on a proven short-course truck chassis with an upper electronics deck. The design follows the philosophy used in F1TENTH-style research platforms: use a commercially available RC chassis for drivetrain, suspension, and steering, then add sensors, compute, power regulation, and software modules for autonomy.

The recommended first build should **not** attempt to custom-design the full mechanical chassis. Instead, it should convert a reliable RC platform into an autonomous platform.

### Recommended Architecture

The car is split into three physical layers:

| Layer | Purpose |
|---|---|
| **Lower chassis** | RC drivetrain, steering servo, suspension, motor, battery compartment, wheels, base RC receiver. |
| **Upper control deck** | Main compute (optional for high-level autonomy), auxiliary MCU, power regulation, motor driver, servo driver, wiring harness. |
| **Sensor layer** | Front LiDAR (RPLIDAR C1), IMU (BNO085), optional camera/line sensor for future expansion. |

This structure mirrors the F1TENTH build approach, where the car is organized into lower chassis, autonomy elements, and upper chassis/electronics deck. The official F1TENTH build documentation uses a Traxxas Slash 4x4-style chassis and NVIDIA Jetson compute platform as the base reference design. [W1]

## 1.2 Concept Sketch

### Top View Layout

```text
                           FRONT / DRIVING DIRECTION
        ┌──────────────────────────────────────────────────────┐
        │                                                      │
        │           ┌─────────────────────────┐                │
        │           │  RPLIDAR C1 (12 m)      │                │
        │           │  UART TTL / USB         │                │
        │           └────────────┬────────────┘                │
        │                        │                             │
        │           ┌────────────┴────────────┐                │
        │           │ Steering Servo Linkage  │                │
        │           │ (MG996R PWM)            │                │
        │           └─────────────────────────┘                │
        │                                                      │
        │   ┌──────────────────────────────────────────────┐   │
        │   │          Upper Control Deck                   │   │
        │   │                                              │   │
        │   │  ┌──────────┐ ┌──────────┐ ┌──────────────┐ │   │
        │   │  │ Aux MCU  │ │ BNO085   │ │ Power Dist / │ │   │
        │   │  │ (TBD)    │ │ IMU 9DOF │ │ RC Rx        │ │   │
        │   │  └──────────┘ └──────────┘ └──────────────┘ │   │
        │   │                                              │   │
        │   │  ┌──────────────────┐  ┌────────────────┐   │   │
        │   │  │ Cytron MDD10A    │  │ CANUDUINO      │   │   │
        │   │  │ Dual Motor Drv   │  │ 16-Ch Servo    │   │   │
        │   │  │ (PWM control)    │  │ Driver (I2C)   │   │   │
        │   │  └──────────────────┘  └────────────────┘   │   │
        │   └──────────────────────────────────────────────┘   │
        │                                                      │
        │  [Battery tray low in chassis for center of gravity] │
        │                                                      │
        └──────────────────────────────────────────────────────┘
        ◆ Motor 1                                    ◆ Motor 2
        (JGB37-520 left)                    (JGB37-520 right)
                           REAR / MOTOR SIDE
```

### Side View Layout

```text
                    Optional camera mast for future expansion
                         ┌─────────────┐
                         │Optional Cam │
                         └──────┬──────┘
           │
    ┌───────┐       ┌──────┴────────────────────────┐
    │LiDAR  │       │ Upper control deck            │
    └───────┘       │ MCU + MDD10A + servo driver   │
        ────────────────┴─────────────────────────────────
        ┌────────────────────────────────────────────────┐
    │ Lower RC chassis: battery, motors, steering, Rx │
        └────────────────────────────────────────────────┘
             O                                           O
          Front wheel                                 Rear wheel
```

### CAD / Mechanical Modeling Plan

The first CAD package should focus only on add-on parts:

| CAD Part | Purpose |
|---|---|
| Upper control deck | Mount MCU/main compute, MDD10A, servo driver, power board, cable tie points, and standoffs. |
| LiDAR front bracket | Keep LiDAR level and forward-facing, with clear scan field for RPLIDAR C1 (or future compatible 2D LiDAR). |
| IMU bracket | Rigid mount for BNO085 near centerline, with known axis orientation. |
| Camera mast (optional) | Adjustable height and tilt for lane/track visibility in future upgrades. |
| Battery retainer | Prevent LiPo movement under acceleration/braking. |
| Kill-switch bracket | Externally reachable emergency stop or power cutoff. |
| Cable routing clips | Keep wires away from driveshafts, wheels, and steering linkages. |

## 1.3 Estimated Size and Weight

### Base Chassis Estimate

A practical chassis choice is the **Traxxas Slash 4x4 VXL HD / Slash 4x4-style 1/10 short-course truck**, because it is durable, widely supported, and close to the F1TENTH reference design.

| Dimension | Estimate |
|---|---:|
| Length | ~568 mm / 22.36 in |
| Width | ~296 mm / 11.65 in |
| Height before autonomy hardware | ~193 mm / 7.60 in |
| Wheelbase | ~324 mm / 12.75 in |
| Ground clearance | ~72 mm / 2.83 in |
| Base chassis weight | ~2.6 kg / 5.8 lb |

These dimensions are based on current Slash 4x4 VXL HD retailer specifications. [W2]

### Finished Robot Estimate

| Configuration | Estimated Finished Size | Estimated Finished Weight |
|---|---|---:|
| **Camera/line-sensor low-cost build** | ~568 mm L × 296 mm W × 230–300 mm H | ~3.0–3.5 kg |
| **LiDAR-first style build** | ~568 mm L × 296 mm W × 250–330 mm H | ~3.4–4.2 kg |
| **LiDAR + depth camera + extra sensors** | ~568 mm L × 296 mm W × 280–350 mm H | ~3.7–4.6 kg |

Weight should be measured after assembly. The design target should keep the **battery low**, mount **control electronics near the centerline**, and avoid placing heavy sensors high or far forward unless required for visibility.

## 1.4 Parts List With Estimated Costs

The table below reflects the **current prototype parts actually selected by the team**.

### Current Prototype Build (Confirmed Components)

| Subsystem | Selected Part | Purpose | Estimated Cost |
|---|---|---|---:|
| Primary LiDAR | RPLIDAR C1 (12 m) | Front scanning for wall following, gap finding, obstacle detection | $70.00 |
| IMU | 7semi BNO085 (9-DOF) | Yaw/acceleration support for control and state estimation | $25.40 |
| Traction motor (x2) | DC Motor JGB37-520 | Main traction actuation | $9.86 each (2x) |
| Motor driver | Cytron MDD10A Dual Channel 10A | PWM-based dual channel traction motor control | TBD |
| Steering servo | AZDelivery MG996R Digital Servo | Steering actuation (PWM) | $10.47 |
| Servo controller | CANUDUINO 16-Channel 12-bit | Multi-servo PWM generation over I2C | $14.84 |
| Auxiliary controller | Dedicated MCU (model TBD) | PID loops, encoder readout, IMU readout, watchdog, PWM supervision | TBD |

### Estimated Core Electronics Cost (Current Selection)

| Cost Summary | Estimated Total |
|---|---:|
| Known subtotal (without MDD10A and MCU) | **$140.43** |

Future upgrades (high-performance compute, additional vision stack, or premium LiDAR) can be evaluated after baseline reliability is demonstrated.

## 1.5 Build and Testing Timeline Before Competition Day

The timeline below assumes roughly **12 weeks**. If the team has less time, preserve the order of the milestones and compress scope, not safety.

### T-12 to T-10 Weeks: Requirements and Procurement

| Goal | Deliverables |
|---|---|
| Lock competition rules | Track type, allowed sensors, allowed compute, max size/weight, required safety systems. |
| Finalize architecture | Decide line/camera-first vs LiDAR-first vs hybrid. |
| Order parts | Chassis, MCU/main compute, MDD10A, sensors, batteries, charger, mechanical hardware. |
| Set up repository | Git repo, issue tracker, design folder, BOM spreadsheet, experiment logs. |

**Gate:** Parts ordered; competition constraints documented.

### T-9 to T-8 Weeks: Mechanical and Electrical Bring-Up

| Goal | Deliverables |
|---|---|
| Assemble base RC chassis | Manual RC driving works reliably. |
| Mount upper deck | MCU/main compute, MDD10A, servo driver, power board physically secured. |
| Power distribution | Safe power to compute and sensors; voltage checks completed. |
| Battery safety | Charging, storage, and inspection process documented. |

**Gate:** Manual driving and safe power-on/power-off procedure verified.

### T-7 to T-6 Weeks: Software Bring-Up and Teleoperation

| Goal | Deliverables |
|---|---|
| Controller setup | Firmware setup for MCU and optional high-level compute integration. |
| Motor integration | MDD10A PWM tuning, motor calibration, steering calibration, encoder readout validation. |
| Sensor integration | LiDAR/IMU/(optional camera or line sensor) data visible and logged. |
| Manual override | RC or gamepad override tested. |

**Gate:** Robot can be manually driven while logging sensor and control data.

### T-5 Weeks: Baseline Autonomy

| Track Type | Baseline Algorithm |
|---|---|
| Line-marked track | PID line-following using line sensor or camera lane center error. |
| Wall-bounded track | Wall-following PID or Follow-the-Gap using LiDAR/distance sensors. |
| Known closed-loop track | Pure Pursuit on pre-defined waypoints at very low speed. |

**Gate:** Robot completes a slow lap/route without manual steering for at least 3 consecutive runs.

### T-4 Weeks: Safety and Reliability Layer

| Goal | Deliverables |
|---|---|
| Emergency stop | Physical and remote stop tested. |
| Watchdog | Lost heartbeat or stale sensor data stops throttle. |
| Speed cap | Software and controller-level speed limits. |
| Obstacle stop | Minimum distance threshold triggers braking/neutral throttle. |
| Logging | Every run logs timestamp, mode, sensor status, steering, throttle, battery. |

**Gate:** Safety tests pass before faster driving is allowed.

### T-3 Weeks: Path Planning and Speed Profiling

| Goal | Deliverables |
|---|---|
| Track map or centerline | Map, waypoints, or line model created. |
| Pure Pursuit | Path follower tuned at low speed. |
| Speed profile | Slower in curves, faster on straights, capped by perception confidence. |
| Recovery tests | Robot recovers from small starting offsets. |

**Gate:** 5 consecutive low-speed successful laps/routes.

### T-2 Weeks: Mock Competition Testing

| Goal | Deliverables |
|---|---|
| Competition simulation | Same start procedure, same scoring, same run duration. |
| Failure drills | Sensor unplug, lost Wi-Fi, low battery, obstacle on track, manual override. |
| Spare parts kit | Batteries, tools, cables, adapters, tires, fasteners, charger. |
| Freeze major changes | Only bug fixes and calibration changes allowed. |

**Gate:** Robot completes full mock competition procedure with safety systems active.

### T-1 Week to Competition Day

| Goal | Deliverables |
|---|---|
| Code freeze | Tag release branch and store backup image. |
| Calibration sheet | Steering center, speed limit, sensor transforms, battery thresholds. |
| Battery rotation plan | Charged packs labeled; fireproof bag and charger packed. |
| Final inspection | Wheels, steering, wires, mounts, sensor field of view, kill switch. |

**Gate:** Competition-ready checklist signed off.

---

# 2. Component Selection

## 2.0 Selected Components for Current Prototype (Team-Confirmed)

The following components are the specific parts selected for the current prototype build and should be treated as the baseline integration targets.

| Subsystem | Selected Model | Qty | Interface / Control | Unit Cost (USD) | Subtotal (USD) |
|---|---|---:|---|---:|---:|
| LiDAR | RPLIDAR C1 (12 m) | 1 | UART TTL / USB | 70.00 | 70.00 |
| IMU (9-DOF) | 7semi BNO085 | 1 | I2C / SPI / UART | 25.40 | 25.40 |
| Traction motor | DC Motor JGB37-520 | 2 | DC motor (driven via PWM motor driver) | 9.86 | 19.72 |
| Traction motor driver | Cytron MDD10A Dual Channel 10A DC Motor Driver | 1 | PWM control input | TBD | TBD |
| Steering servo | AZDelivery MG996R Digital Servo Motor | 1 | PWM | 10.47 | 10.47 |
| Servo controller | CANUDUINO 16-Channel 12-bit Servo Driver | 1 | I2C | 14.84 | 14.84 |

**Known subtotal (excluding MDD10A): 140.43 USD**

### Auxiliary Microcontroller Scope

An auxiliary microcontroller will be used to execute low-latency control and safety loops:

- PID control loops for traction motors.
- Encoder acquisition and speed estimation.
- IMU acquisition and filtering.
- PWM output generation for motor/servo actuation.
- Watchdog supervision and fail-safe response.

**Open item:** finalize the specific MCU model and document its required peripherals (timers/PWM channels, encoder interfaces, I2C/SPI/UART availability, and watchdog features).

## 2.1 Motors

### Recommended Motor/Drivetrain Choice

| Item | Selection |
|---|---|
| Motor type | Brushed DC geared motor |
| Example | JGB37-520 |
| Quantity | 2 |
| Drive layout | Differential traction drive with two DC traction motors |
| Steering type | Ackermann steering with standard RC steering servo |
| Controller | Cytron MDD10A dual-channel 10A motor driver (PWM input) |

### Justification

A geared DC motor pair was selected for this prototype due to lower cost, simpler integration, and easier availability. Closed-loop behavior is provided by the auxiliary MCU + encoder feedback + PWM command to MDD10A.

For autonomous control, the selected architecture supports:

- motor RPM control,
- encoder telemetry,
- speed estimation,
- repeatable throttle behavior,
- programmable safety limits.

### Motor/Speed Strategy

The motor is capable of speeds far above what the autonomous software should initially use. The design should enforce software and controller-level speed limits.

| Phase | Max Speed Recommendation |
|---|---:|
| Bench test / wheels lifted | 0.0–0.2 m/s equivalent |
| First autonomous tests | 0.3–0.7 m/s |
| Stable baseline testing | 0.8–1.8 m/s |
| Competition tuning | 1.5–3.0 m/s only after safety gates pass |
| Advanced upgrade path | Higher speeds only after drivetrain, sensing, and safety redesign |

## 2.2 Battery

### Recommended Battery

| Item | Recommendation |
|---|---|
| Chemistry | Lithium Polymer (LiPo) |
| Voltage | 3S, 11.1 V nominal |
| Capacity | 5000–6000 mAh |
| Discharge rating | 25C or higher |
| Quantity | Minimum 3 packs for testing/competition |
| Connector | Match main power bus, MDD10A, and regulator connectors; use properly rated adapters |
| Charger | Balance charger with storage-charge mode |

### Runtime Estimate

A 3S 5000 mAh LiPo stores approximately:

```text
Energy = 11.1 V × 5.0 Ah = 55.5 Wh
```

Runtime depends heavily on driving style.

| Use Case | Expected Runtime per 5000 mAh Pack |
|---|---:|
| Sensor/computer idle only | >1 hour theoretically, but not a useful competition estimate |
| Slow testing with frequent stops | ~25–45 minutes |
| Moderate autonomous driving | ~15–30 minutes |
| Aggressive high-speed testing | ~8–15 minutes |

For competition planning, assume **15–20 minutes of useful runtime per pack** and bring multiple charged batteries.

### Battery Justification

LiPo is the practical choice for 1/10-scale RC platforms because it provides high power density and can support motor bursts. However, LiPo safety must be treated as a design requirement, not an afterthought. The official F1TENTH build documentation explicitly warns that LiPo batteries store a large amount of energy in a small space and can damage the car or cause fire if used improperly. [W1]

## 2.3 Sensors

The sensor set is anchored to the current selected hardware and keeps optional room for later expansion.

### Recommended Sensor Stack

| Sensor | Required? | Purpose | Recommendation |
|---|---|---|---|
| **2D LiDAR** | Required (selected) | Wall following, obstacle detection, Follow-the-Gap baseline | RPLIDAR C1 (12 m), UART TTL / USB |
| **Camera** | Optional (future) | Lane/line detection, data collection, future vision ML | USB/CSI camera as future expansion |
| **Distance sensors** | Optional safety backup | Near-field stop zones and side clearance | ToF sensors preferred over ultrasonic for short-range repeatability |
| **Encoders / odometry** | Required | Speed and distance estimation | Wheel encoder feedback processed by auxiliary MCU |
| **IMU** | Required (selected) | Yaw rate, acceleration, orientation support | 7semi BNO085, I2C/SPI/UART |
| **Battery telemetry** | Required | Voltage, current, low-battery shutdown | Power monitor + MCU software checks |
| **RC receiver / gamepad** | Required | Manual override and testing | RC transmitter or gamepad bridge |

### Track-Type-Specific Sensor Choice

| Track Type | Minimum Sensor Set | Preferred Algorithm |
|---|---|---|
| Black/white line track | Line sensor array + IMU/odometry | PID line-following |
| Lane-marked camera track | Camera + odometry | Vision lane center PID / behavioral cloning later |
| Wall-bounded track | 2D LiDAR or side ToF sensors | Wall-following PID / Follow-the-Gap |
| Known mapped track | 2D LiDAR + wheel encoder odometry + IMU | SLAM/localization + Pure Pursuit |
| Obstacle course | 2D LiDAR + camera/depth + odometry | Follow-the-Gap + AEB + local avoidance |

### Sensor Mounting Requirements

| Sensor | Mounting Requirement |
|---|---|
| LiDAR | Level, rigid, front/center or high enough for unobstructed scan; avoid wheel/body occlusion. |
| Camera | Adjustable tilt; view should include 1–3 m of track ahead, not mostly horizon/body shell. |
| Line sensor | Low to ground, fixed height, protected from impacts, shielded from ambient light if needed. |
| IMU | Near vehicle centerline; rigid mount; known axis orientation. |
| Distance sensors | Front and/or side-facing; protected from wheel spray and impacts. |

---

# 3. Software and Controls

## 3.1 High-Level Control Strategy

The recommended software strategy is a **progressive autonomy stack**:

```text
Manual Mode
   ↓
Safe Baseline Mode
   ↓
Line / Wall / Gap Following Mode
   ↓
Mapped Path-Following Mode
   ↓
Advanced Planning / Learning Mode
```

The design should first prove that the car can be safely driven and stopped. Only then should the team optimize lap time.

## 3.2 Software Architecture

```text
Sensors
  ├── LiDAR (RPLIDAR C1)
  ├── IMU (BNO085)
  ├── Encoder odometry
  ├── Optional camera / line sensor
  ├── Optional distance sensors
  └── Battery telemetry

Perception / State Estimation
  ├── Line position or lane center estimate
  ├── Wall distance / gap estimate
  ├── Obstacle distance
  ├── Vehicle speed / odometry
  └── Optional localization on map

Planning
  ├── Baseline: line center / wall target / largest gap
  ├── Intermediate: waypoints and Pure Pursuit target
  ├── Speed profile based on curvature and confidence
  └── Future: MPC / MPCC / RL

Control
  ├── Steering command
  ├── Throttle / speed command
  ├── Brake / neutral command
  └── Command limiter

Safety Supervisor
  ├── Manual override
  ├── Watchdog timeout
  ├── Speed cap
  ├── Obstacle stop
  ├── Sensor stale-data check
  └── Low-battery stop

Actuation
  ├── MDD10A motor control (PWM)
  ├── Servo control via 16-ch I2C driver
  └── Auxiliary MCU watchdog/supervision
```

This structure follows the common autonomous racing decomposition into perception, planning, and control. The autonomous racing survey uses this perception-planning-control pipeline to categorize autonomous racing research. [P5]

## 3.3 Baseline Algorithms

### 3.3.1 PID Line Following

Use this if the competition track has a visible line.

| Input | Line position error from line sensor or camera |
| Output | Steering angle |
| Throttle | Constant low speed initially; later speed based on line confidence and curve estimate |
| Pros | Simple, explainable, easy to debug |
| Cons | Track-dependent; fails if line is missing, dirty, reflective, or poorly lit |

Control law:

```text
steering = Kp * error + Ki * integral(error) + Kd * derivative(error)
```

Recommended starting behavior:

- reduce speed when line confidence is low,
- stop if line is lost for more than a fixed timeout,
- ramp steering commands to avoid oscillation,
- log line position, steering, throttle, and confidence every run.

### 3.3.2 Wall-Following PID

Use this if the track is bounded by walls or rails.

| Input | Left/right wall distance from LiDAR or ToF sensors |
| Output | Steering angle |
| Target | Maintain desired offset from wall or center of corridor |
| Pros | Simple and good for controlled indoor tracks |
| Cons | Can fail in open areas, corners, or with discontinuous boundaries |

### 3.3.3 Follow-the-Gap

Use this for obstacle avoidance and mapless LiDAR driving.

Follow-the-Gap uses LiDAR to find the largest unobstructed region and steers toward it.  It acts as a simple, computationally efficient geometric obstacle-avoidance method that uses LiDAR data to find the largest gap and drive toward its midpoint. It is robust in dynamic environments but can choose suboptimal paths and can zigzag when similar gaps appear on both sides. [P4]

**Design decision:** Follow-the-Gap should be implemented as a **safety/local avoidance mode**, not as the primary racing-line optimizer.

### 3.3.4 Pure Pursuit Path Following

Use this as the first planned path-following controller once a centerline or waypoint path exists.

Pure Pursuit is recommended for the first mapped-path controller because it is:

- simple,
- explainable,
- computationally light,
- widely used in F1TENTH,
- easier to transfer to physical hardware than MPCC/RL.

The F1TENTH survey notes that Pure Pursuit remains the most common control algorithm because of robustness and simplicity. [P3]

The adaptive lookahead Pure Pursuit paper improves the standard Pure Pursuit approach by assigning lookahead distance per waypoint and reports improved racing metrics in simulation and on a scaled F1/10 testbed. [P7]

## 3.4 Planned Algorithms by Development Stage

| Stage | Algorithm | Purpose | Competition Readiness |
|---|---|---|---|
| Stage 1 | Manual teleoperation + logging | Verify hardware and collect data | Required |
| Stage 2A | PID line-following | Baseline for line track | High if track has line |
| Stage 2B | Wall-following PID | Baseline for walled track | High if walls are reliable |
| Stage 2C | Follow-the-Gap | Mapless obstacle avoidance | Medium; good fallback |
| Stage 3 | SLAM / map creation | Create track map | Required for mapped autonomy |
| Stage 4 | Pure Pursuit | Follow centerline/racing line | High |
| Stage 5 | Adaptive lookahead Pure Pursuit | Improve lap time and stability | Medium-high |
| Stage 6 | Speed profiling | Slow for turns, fast on straights | High once Pure Pursuit is stable |
| Stage 7 | MPC | More accurate path tracking | Medium; requires tuning |
| Stage 8 | MPCC | High-performance racing | Research/future |
| Stage 9 | DRL / end-to-end learning | Learning-based autonomy | Research/future |

## 3.5 Speed Profiling

Speed profiling should be conservative and rule-based first.

### Initial Speed Rule

```text
if safety_stop:
    speed = 0
elif sensor_confidence_low:
    speed = low_speed
elif high_curvature_ahead:
    speed = curve_speed
elif obstacle_near:
    speed = slow_speed
else:
    speed = straight_speed
```

### Recommended Parameters

| Condition | Speed Behavior |
|---|---|
| Startup / first lap | 0.3–0.7 m/s |
| Tight turn | Reduce by 40–70% |
| Low line/camera confidence | Reduce to crawl speed or stop |
| Obstacle inside safety zone | Stop |
| Long straight with good confidence | Gradually increase |
| Low battery | Reduce speed and return/stop |
| High steering oscillation | Reduce speed automatically |

## 3.6 Testing and Validation Approach

The team should use repeatable tests before competition day.

| Test | Purpose | Pass Criteria |
|---|---|---|
| Manual drive test | Verify drivetrain and steering | No binding, no unexpected throttle |
| Sensor visibility test | Confirm sensor frames/data | Camera/LiDAR/line values stable |
| Static safety test | Confirm stop logic without movement | All stop triggers produce zero throttle |
| Slow autonomous test | Verify baseline algorithm | Complete short segment without intervention |
| Full lap test | Verify route completion | 3 consecutive successful laps |
| Disturbance test | Check robustness | Offset start, lighting change, obstacle added |
| Battery runtime test | Estimate run duration | Runtime logged for each pack |
| Mock competition | Full procedure rehearsal | Complete run under frozen code |

Studies like GRAIC experience report highlights why autonomy testing is challenging: autonomy pipelines combine heterogeneous perception, planning, decision-making, and control modules; tests can be flaky and long; and deterministic simulation/closed-loop testing is difficult but important. It recommends automated testing concepts such as repeatable scenarios, score/log/video collection, and deterministic setup where possible. [P6]

---

# 4. Safety Systems

## 4.1 Safety Philosophy

The car should be designed so that **loss of confidence leads to stopping**, not guessing.

Safety systems must cover:

1. physical power safety,
2. battery safety,
3. manual override,
4. software watchdogs,
5. perception failure,
6. obstacle avoidance,
7. testing procedure.

## 4.2 Required Safety Systems

### 4.2.1 Physical Emergency Stop

| Requirement | Design |
|---|---|
| External access | A physical switch or removable loop must be reachable without touching wheels/drivetrain. |
| Function | Cut motor power or force MCU outputs to neutral/disabled PWM state. |
| Labeling | Red/yellow or clearly marked “E-STOP / POWER”. |
| Verification | Test before every autonomous run. |

### 4.2.2 Remote Manual Override

| Requirement | Design |
|---|---|
| RC override | Operator can immediately take steering/throttle control. |
| Mode switch | Autonomous mode enabled only when a dedicated switch/button is active. |
| Failsafe | Loss of RC signal should command neutral throttle/brake. |
| Test | Verify at low speed before each test session. |

### 4.2.3 Software Watchdog

| Failure | Required Response |
|---|---|
| No autonomy command for >100–250 ms | Throttle = 0, steering hold or center |
| Sensor timestamp stale | Slow or stop |
| MCU-motor driver communication error | Stop command and alert |
| CPU overload / process crash | Stop command |
| Low battery voltage | Reduce speed, then stop |
| Invalid steering/throttle command | Clamp or reject command |

### 4.2.4 Autonomous Emergency Braking / Obstacle Stop

A simple automatic emergency braking layer should run independently of the main planner.

| Input | Trigger |
|---|---|
| LiDAR minimum distance | Stop if obstacle inside distance threshold |
| ToF front sensor | Stop if object very near |
| Camera/depth optional | Stop if obstacle detected in path |
| Line sensor optional | Stop if line lost too long |

### 4.2.5 Speed and Acceleration Limits

| Phase | Max Speed | Acceleration |
|---|---:|---|
| Bring-up | 0.3–0.5 m/s | Very low |
| First autonomous | 0.5–1.0 m/s | Low |
| Stable autonomy | 1.0–2.0 m/s | Moderate |
| Competition tuning | Rule-dependent | Ramped |
| Advanced racing | Only after review | Ramped and logged |

Speed limit should exist in at least two places:

1. software command limiter,
2. MCU/PWM driver output limits.

### 4.2.6 LiPo Battery Safety

Design requirements:

| Battery Safety Item | Requirement |
|---|---|
| Charging | Balance charge only, attended, in fireproof bag. |
| Storage | Storage charge when not used for more than a day. |
| Physical retention | Battery must be strapped so it cannot eject. |
| Polarity | Use keyed connectors and label polarity. |
| Low-voltage cutoff | Enforce via power monitor/MCU and software warning. |
| Inspection | Check puffing, heat, damaged wires before each run. |

### 4.2.7 Mechanical Safety

| Risk | Mitigation |
|---|---|
| Exposed wheels/gears | Keep hands clear; use wheels-lifted stand for bench tests. |
| Sharp brackets | Round edges and cover exposed screw tips. |
| Sensor damage | Use guards and breakaway mounts where possible. |
| Wire entanglement | Route wires away from driveshafts, wheels, servo linkage. |
| High center of gravity | Keep battery low and heavy electronics centered. |
| Collision damage | Add foam bumper and keep spare mounts. |

## 4.3 Pre-Run Safety Checklist

Before every autonomous run:

```text
[ ] Battery visually inspected: no puffing, no heat, no damaged wires.
[ ] Battery voltage checked.
[ ] Wheels, suspension, steering linkage checked.
[ ] Sensor mounts tight.
[ ] LiDAR/camera/line sensor data visible.
[ ] Motor driver, encoder, and IMU telemetry visible.
[ ] Manual RC override tested.
[ ] Physical E-stop tested.
[ ] Software stop command tested.
[ ] Speed cap set for current test phase.
[ ] Track is clear of people, loose objects, and cables.
[ ] Logger running.
```

## 4.4 Stop Conditions

The robot must stop automatically if any of the following are true:

```text
manual_override_active == true
e_stop_pressed == true
autonomy_heartbeat_stale == true
sensor_data_stale == true
obstacle_distance < emergency_stop_distance
line_lost_timeout_exceeded == true
battery_voltage < safe_threshold
steering_command_invalid == true
throttle_command_invalid == true
robot_outside_track_boundary == true
```

## 4.5 Safety Verification Tests

| Test | Method | Expected Result |
|---|---|---|
| E-stop test | Press physical stop during low-speed roll | Motor stops immediately |
| RC override | Toggle autonomous/manual switch | Manual control takes priority |
| Heartbeat loss | Kill autonomy process | Motor command goes neutral |
| LiDAR stale | Disconnect or block LiDAR topic | Car slows/stops |
| Line lost | Remove line/cover sensor | Car stops after timeout |
| Obstacle stop | Place object in front at threshold | Car stops before contact |
| Low battery | Simulate low voltage threshold | Speed reduced/stopped |
| Command clamp | Send max steering/throttle command | Command limited to safe range |

---

# Review Decision Summary

| Area | Recommended Decision |
|---|---|
| Chassis | Use 1/10-scale 4WD short-course RC platform, preferably Traxxas Slash 4x4-style. |
| Compute | Use auxiliary MCU for real-time control; optional high-level compute board can be added later. |
| Motor control | Use Cytron MDD10A with closed-loop control implemented on auxiliary MCU. |
| Battery | Use 3S 11.1 V 5000–6000 mAh LiPo; bring at least 3 packs. |
| Primary sensor | Use RPLIDAR C1 as primary ranging sensor and BNO085 as primary IMU. |
| First autonomy algorithm | PID line-following for line track or wall-following/Follow-the-Gap for walled track. |
| First planned controller | Pure Pursuit with conservative speed profile. |
| Future algorithms | Adaptive Pure Pursuit, MPC, MPCC, DRL after baseline and safety pass. |
| Safety | Physical E-stop, remote override, watchdog, speed cap, obstacle stop, LiPo procedure. |
| Testing | Require repeatable logs, mock competition, and no major changes inside final week. |

---

# Source Basis

## Uploaded papers

| ID | Source |
|---|---|
| P1 | O’Kelly et al., **F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning** |
| P2 | Agnihotri et al., **Teaching Autonomous Systems at 1/10th-scale: Design of the F1/10 Racecar, Simulators and Curriculum** |
| P3 | Evans et al., **Unifying F1TENTH Autonomous Racing: Survey, Methods and Benchmarks** |
| P4 | Kong, **From Simulation to Reality: Assessing the Efficacy of Pure Pursuit, MPC, and MPCC on the F1Tenth Platform** |
| P5 | Betz et al., **Autonomous Vehicles on the Edge: A Survey on Autonomous Vehicle Racing** |
| P6 | Jiang et al., **Continuous Integration and Testing for Autonomous Racing Software: An Experience Report from GRAIC** |
| P7 | Sukhil & Behl, **Adaptive Lookahead Pure-Pursuit for Autonomous Racing** |
| P8 | Evans et al., **Comparing Deep Reinforcement Learning Architectures for Autonomous Racing** |
| P9 | Aurandt et al., **Multimodal Model Predictive Runtime Verification for Safety of Autonomous Cyber-Physical Systems** |

## External component/current-cost references

| ID | Source |
|---|---|
| W1 | F1TENTH official build documentation and bill of materials pages |
| W2 | Current retailer specifications for Traxxas Slash 4x4 VXL HD / 68386-4-style chassis |
| W3 | RPLIDAR C1 product specifications and current pricing reference |
| W4 | 7semi BNO085 9-DOF module specifications and current pricing reference |
| W5 | Cytron MDD10A specifications and current pricing reference |
| W6 | AZDelivery MG996R and CANUDUINO 16-Channel 12-bit controller references |
| W7 | Traxxas 3S 5000 mAh LiPo battery pricing reference |

---

# Items to Finalize After Competition Rules Are Available

| Question | Why It Matters |
|---|---|
| Is the track line-following, wall-following, cone-bounded, or open obstacle course? | Determines primary sensor and baseline algorithm. |
| Are LiDAR/depth cameras allowed? | Determines whether F1TENTH-style autonomy is legal. |
| Are there size/weight limits? | Determines mounting deck and battery choice. |
| Is there a required emergency stop standard? | May require specific hardware kill switch or wireless E-stop. |
| Is Wi-Fi allowed during autonomous run? | Determines whether remote telemetry/commands can be used. |
| Is manual override allowed or required? | Determines RC receiver and failsafe design. |
| What is the competition date? | Allows final schedule to be converted from T-minus plan to calendar dates. |
| Are there obstacles or other robots? | Determines whether Follow-the-Gap/AEB is mandatory. |
| Is scoring based on time, completion, penalties, or accuracy? | Determines speed profile and risk appetite. |
