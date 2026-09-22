# G2 covariance policy

Zero covariance never means “unknown,” and a huge covariance never acts as an
on/off switch. Variable selection is made by `robot_localization` config
booleans; a measurement is omitted when it is not trustworthy.

| Input | Variable | Current source class | G2 policy |
|---|---|---|---|
| ZED VIO | X, Y, yaw | `SYNTHETIC_TEST_ONLY` offline; `SENSOR_REPORTED` on hardware | synthetic variance is exactly the injected Gaussian noise; preserve and audit wrapper covariance on hardware |
| VESC telemetry | body Vx | `SYNTHETIC_TEST_ONLY` offline; `UNKNOWN` physical variance | publish a physical speed measurement only after repeated telemetry characterization supplies a positive variance |
| BNO08X | all | `UNKNOWN` | excluded until extrinsic, orientation convention, bias, and covariance are measured |

Synthetic fixture standard deviations are 0.015 m (VIO X/Y), 0.012 rad (VIO
yaw), and 0.020 m/s (speed). The synthetic EKF-only rejection threshold is a
documented test guard for injected single-sample faults; it is absent from the
production-intent configuration. `PRODUCTION_OUTLIER_THRESHOLDS=PENDING_PHYSICAL_INNOVATION_DATA`.
Synthetic outcomes are classified as rejected, accepted-but-bounded, recovered,
or undetermined; they are never copied as physical Mahalanobis policy.

All emitted covariance arrays are row-major matrices: only documented
diagonal variances are non-zero; unmeasured cross-correlations are exactly
zero. G2 regression tests prevent accidentally filling every matrix element
with a scalar, which would falsely assert correlation between unrelated state
variables.
