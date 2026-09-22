# V2 test plan

Levels 0-5 run without hardware.

1. **G1 static**: canonical YAML/JSON schema, deterministic contract compiler,
   generated footprint equivalence, URDF property inclusion, REP-105 authority,
   negative TF cases, and synthetic ±10-degree ramp projection.
2. **G2.1 local odometry**: actual Humble `robot_localization` EKF in an isolated
   ROS domain with deterministic standard VIO/speed fixtures. Fifteen cases
   cover motion, directional radii, dropouts, outliers, timestamp faults, and a
   ramp. A reproducible test-only `Vy=0` A/B experiment is retained as negative
   evidence; it must not mask all-input timeout. Assert a single planar
   `odom -> base_footprint` TF, monotonic finite output, no map TF, and no
   actuator publishers. Production intent is separately launched without the
   synthetic publisher and uses a raw-frame/output-frame diagnostic contract.
   G2.1 adds deterministic restart/time/frame/NaN/stale-speed policy fixtures;
   physical VIO reset and covariance evidence remain Level C.
3. **Unit**: geometry, eRPM conversion, footprint construction, gate logic.
4. **Planner fixtures**: original five cases plus 100 seeded adversarial cases:
   fractional cells, outside map, corners, unknown, inflation, narrow passages,
   reverse/cusps, and impossible starts/goals.
5. **Rosbag replay**: recorded map/odom/scan inputs and deterministic actions.
6. **Loopback**: planner/BT/controller lifecycle and timeout/preemption.
7. **Gazebo**: V004 Ackermann dynamics, sensor latency/noise, Speed Course.
8. **Hardware**: stationary, manual mapping, low-speed autonomy, then speed.

Primary planner metric: `INVALID_PATH_ESCAPE_COUNT=0`. `NO_SAFE_PATH` on an
impossible fixture is PASS. A path must independently pass official discrete,
continuous collision, and Ackermann validation. Hardware levels are
`BLOCKED_BY_HARDWARE` while devices are absent. The G1 test-only TF harness is
disabled by default and contains no sensors, Nav2, or motion publishers.
