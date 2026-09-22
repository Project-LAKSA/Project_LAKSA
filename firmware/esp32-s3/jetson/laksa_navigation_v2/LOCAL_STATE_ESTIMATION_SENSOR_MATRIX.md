# G2 local-state sensor matrix

`robot_localization` state-vector order is X, Y, Z, roll, pitch, yaw, Vx, Vy,
Vz, Vroll, Vpitch, Vyaw, Ax, Ay, Az.

| State | G2 current | Reason | Future after hardware calibration |
|---|---|---|---|
| X, Y | ZED VIO relative pose | local visual-inertial displacement | retain after covariance review |
| Z | none; forced planar | Nav2 local state is planar | separate body/3D estimator decision |
| roll, pitch | none; forced planar | body attitude is not navigation pose | BNO08X/ZED only after frame proof |
| yaw | ZED VIO relative pose | avoids raw ZED IMU correlation | evaluate independent IMU yaw rate only after correlation study |
| Vx | actual VESC telemetry adapter | measured longitudinal speed, not command | enable after variance characterization |
| Vy | none | no premature no-slip hard constraint | evaluate pseudo-measurement from replay/slip evidence |
| Vz, Vroll, Vpitch, Vyaw, Ax, Ay, Az | none | no calibrated independent source in G2 | explicitly evaluate per sensor after hardware calibration |

Raw ZED IMU fusion in this EKF is `DISABLED_CORRELATED`; ZED VIO internally
uses its camera IMU. BNO08X fusion is `BLOCKED_BY_IMU_EXTRINSIC_MEASUREMENT`.
