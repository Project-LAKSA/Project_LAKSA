# G2.1 production-hardening report

## Scope

G2.1 changes only V2 local-state-estimation contracts. It starts no mapper,
localizer, Nav2 node, controller, sensor driver, actuator path, or production
service. The frozen legacy branches and mapping files are untouched.

## Installed evidence

The target Jetson is Ubuntu 22.04.5 / ROS 2 Humble. Its installed
`robot_localization` is `3.5.4-1jammy.20251017.223119`; the isolated ZED
workspace declares wrapper `5.4.1`, and `/usr/local/zed/zed-config-version.cmake`
declares SDK `5.4.1`.

Humble's installed `ekf.yaml` defines `relative=true` as making the first
measurement the zero point. ZED local VIO is already a continuous local pose,
so V2 sets `odom0_relative=false` and `odom0_differential=false`.

## Raw camera semantics and base conversion

ZED Wrapper 5.4.1 source publishes its odometry with header `odom` and child
`<camera_name>_camera_link`; V2 fixes the camera name to `zed`, yielding
`zed_camera_link`. The wrapper has no supported parameter that changes this
pose origin to LAKSA's vehicle base. An isolated real-EKF proof with the G1
non-zero camera mount exposed this: without conversion, stationary error was
about 0.109 m, the mount offset itself.

`vio_base_odometry_adapter_node` is therefore the smallest documented glue:
it composes raw `odom -> zed_camera_link` with canonical TF
`zed_camera_link -> base_footprint`, publishes only standard
`/laksa/vio/base_odom`, and publishes neither TF nor commands. It is not an
estimator. A real-Humble two-case proof after this composition passed 2/2;
stationary error was 0.027 m under injected synthetic noise.

The complete real-Humble synthetic corpus passed 15/15 in isolated ROS domain
86 with `ROS_LOCALHOST_ONLY=1`. All outputs were finite, planar, monotonic,
and observed the sole `odom -> base_footprint` TF. Mean fixture RMSE was
0.0547 m position, 0.0247 rad yaw, and 0.0215 m/s Vx. These values are not
physical performance claims.

`G2_1_RESULTS.json` records SHA-256 hashes for the configuration and harness
sources copied into `/tmp/laksa-g2-1`. The final G2.1 source tree retains those
same hashes; later changes in this closure are evidence and audit documentation
only. The temporary directory was removed after clean process teardown.

## Production contract

`g2_local_estimation.launch.py` launches only the adapter, EKF, read-only VESC
speed adapter, and read-only diagnostics monitor. Raw VIO remains
`/laksa/vio/odom`; EKF input is `/laksa/vio/base_odom`; output is
`/laksa/odometry/local` and the sole dynamic TF is
`odom -> base_footprint`. The production default excludes speed fusion until a
physical VESC speed variance is measured. When explicitly enabled, speed is
computed only from `measured_erpm * 0.000143738` (V004/T22 identified scale),
never from requested/active eRPM or commands.

`DIRECT_ESP32_CMD_VEL_BYPASS` is registered as critical debt for G6/G7:
legacy micro-ROS direct callbacks can call `apply_drive_command`. G2.1 neither
uses nor changes that code.

## Time, health, and limitations

The EKF explicitly sets `reset_on_time_jump=true`, no lag smoothing, no
corrected-publication replay, and no guessed production outlier thresholds.
The narrow diagnostic monitor reports `LOCAL_ODOMETRY_HEALTHY` fail-closed for
stale VIO/output, frame mismatch, timestamp regression, non-finite values, or
missing canonical TF. Body attitude glue, BNO08X fusion, physical covariance,
and physical restart behavior remain hardware-gated.

Synthetic statistics are qualification-fixture metrics only; they are not
claims about vehicle odometry accuracy.
