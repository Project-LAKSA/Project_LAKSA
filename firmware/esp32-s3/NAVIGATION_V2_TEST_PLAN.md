# V2 test plan

Levels 0-5 run without hardware.

1. **G1 static**: canonical YAML/JSON schema, deterministic contract compiler,
   generated footprint equivalence, URDF property inclusion, REP-105 authority,
   negative TF cases, and synthetic ±10-degree ramp projection.
2. **Unit**: geometry, eRPM conversion, footprint construction, gate logic.
3. **Planner fixtures**: original five cases plus 100 seeded adversarial cases:
   fractional cells, outside map, corners, unknown, inflation, narrow passages,
   reverse/cusps, and impossible starts/goals.
4. **Rosbag replay**: recorded map/odom/scan inputs and deterministic actions.
5. **Loopback**: planner/BT/controller lifecycle and timeout/preemption.
6. **Gazebo**: V004 Ackermann dynamics, sensor latency/noise, Speed Course.
7. **Hardware**: stationary, manual mapping, low-speed autonomy, then speed.

Primary planner metric: `INVALID_PATH_ESCAPE_COUNT=0`. `NO_SAFE_PATH` on an
impossible fixture is PASS. A path must independently pass official discrete,
continuous collision, and Ackermann validation. Hardware levels are
`BLOCKED_BY_HARDWARE` while devices are absent. The G1 test-only TF harness is
disabled by default and contains no sensors, Nav2, or motion publishers.
