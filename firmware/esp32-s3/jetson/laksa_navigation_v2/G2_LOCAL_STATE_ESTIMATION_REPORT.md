# G2 local state estimation

G2 uses the installed ROS 2 Humble `robot_localization` EKF as the only local
navigation TF authority. The production-intent filter is planar and owns
`odom -> base_footprint`; it starts no global localization, mapper, Nav2 node,
controller, sensor driver, or actuator publisher.

The future ZED configuration is a VIO measurement overlay, not a modification
to legacy mapping: tracking `GEN_3`, internal IMU fusion, no area memory,
camera 3D, and both ZED TF publications disabled. ZED contributes relative
X/Y/yaw only. Vehicle Vx uses actual VESC telemetry through a standard-message
adapter; command eRPM is explicitly forbidden as a measurement. The adapter
fails closed until its physical variance is measured. BNO08X fusion remains
blocked by its unknown extrinsic.

The test-only harness is disabled by default, runs in an isolated ROS domain,
and uses only standard VIO/speed messages. It covers stationary, forward,
reverse, directional-radius turns, S-turn, stop/start, sensor dropouts,
outliers, delayed/out-of-order samples, and a body-pitch ramp while the
navigation state stays planar. Results are stored in
`G2_LOCAL_STATE_ESTIMATION_RESULTS.json` after the real Humble EKF run.

An early runner revision populated covariance matrices incorrectly by placing
`1.0` in every off-diagonal entry. The installed EKF trace proved that this
asserted false correlations and destabilized unselected states. G2 now emits
diagonal matrices with zero unmeasured correlation; a regression test prevents
the error. This correction is confined to V2 test/adapter code.

## Offline qualification result

The real installed Jetson package was `robot_localization` 3.5.4 on ROS 2
Humble. The qualification launched one fresh `ekf_node` per scenario in ROS
domain 81, with only `/laksa/vio/odom` and `/laksa/vehicle/speed` publishers.
All 15 required deterministic cases passed. Every output was finite, planar,
and timestamp-monotonic; each case observed the sole `odom -> base_footprint`
TF (45 outputs in total-input timeout and 81 otherwise). No map TF, Nav2,
hardware driver, or actuator publisher was present.

Final-state synthetic summary: position RMS 0.1177 m (maximum 0.2180 m), yaw
RMS 0.0358 rad (maximum 0.1150 rad), and Vx RMS 0.0232 m/s (maximum 0.0454
m/s). These are deterministic test-fixture measurements, not claims about
physical robot accuracy. The total-input dropout stops filtered/TF publication
after `sensor_timeout`; G7 must treat that loss of freshness as an autonomy
fault.

## Vy pseudo-measurement comparison

The required A/B experiment is preserved as machine-readable artifacts:
`G2_AB_NO_VY_CONSTRAINT.json` (A) and `G2_AB_VY_CONSTRAINT.json` (B). A passed
15/15 cases. B passed 14/15 and did not materially improve its synthetic pose,
yaw, or Vx RMSE. More importantly, its test-only `Vy=0` publisher continued
during the total VIO/speed dropout, so the EKF remained live and failed the
required all-input timeout observation. A is therefore the selected G2
configuration: no lateral-velocity pseudo-measurement. The B config is retained
only as reproducible negative evidence and cannot enter production intent.

The timestamp fault fixture now includes both an out-of-order VIO stamp and an
explicit duplicate stamp. A handled that fixture without loss of finite,
monotonic planar output.

`base_footprint -> base_link` dynamic body attitude remains a documented
official-component gap, deferred beyond G2. It neither changes nor reuses the
legacy mapping TF implementation.
