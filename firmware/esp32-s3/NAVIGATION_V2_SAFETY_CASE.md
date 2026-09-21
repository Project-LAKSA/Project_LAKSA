# V2 safety case

Claim: V2 will not intentionally send an unapproved autonomous command to the
vehicle.

| Hazard | Preventive control | Independent evidence |
|---|---|---|
| Invalid path executed | Three-stage path gate; controller has accepted-path input only | deterministic collision corpus |
| Stale command | controller and supervisor timeouts -> zero | unit/loopback timeout tests |
| Lost localization / TF | health monitor disarms autonomy | TF and lifecycle fixtures |
| Sensor obstacle missed | LiDAR primary local obstacle source; collision monitor; stop on stale data | rosbag/loopback tests |
| UI gains authority | no command interface from Field Lab | graph/permission test |
| Manual control overridden | supervisor priority invariant | integration test |
| ESP32/actuator fault | watchdog and hardware safety dominate ROS | hardware-only test |

State machine: `BOOT -> SENSOR_WAIT -> LOCALIZATION_READY -> MAP_READY ->
PATH_READY -> AUTONOMY_ARMED -> EXECUTING`. Any health failure goes to
`DEGRADED` (zero autonomous request); E-stop or safety watchdog goes to
`FAULT/E_STOP`. `EXECUTING` requires fresh approved path, localization, TF,
costmaps, controller, and supervisor health. Hardware tests remain blocked
until devices are connected.
