# Local odometry health contract

G2 exports measurements, `robot_localization` diagnostics, filtered odometry,
and TF. It does not decide autonomy state. G7 must consume these measurable
signals fail-closed:

| Signal | Source | Healthy condition | Degraded meaning |
|---|---|---|---|
| filtered output freshness | `/laksa/odometry/local` | age below evidence-backed future bound | stale estimate, do not execute autonomy |
| input freshness | EKF diagnostics / input headers | VIO and/or speed within `sensor_timeout` | redundancy loss or prediction-only state |
| covariance | filtered odometry | finite, non-negative, bounded by later operational policy | uncertainty is untrusted |
| numerical state | filtered odometry | no NaN or Inf | immediate fault |
| planar TF | `/tf` | unique fresh `odom -> base_footprint`, z/roll/pitch near zero | frame contract fault |
| global TF | graph | absent in G2 local-only runtime | a publisher is an isolation fault |

`local_odometry_contract_monitor` provides a read-only
`/laksa/odometry/local/diagnostics` status with
`LOCAL_ODOMETRY_HEALTHY=true|false`. It fails closed for stale VIO/output,
wrong ZED or EKF frames, timestamp regression, non-finite data, missing
`odom -> base_footprint`, or missing `base_link -> zed_camera_link` mechanical
TF. Loss of speed alone is reported as reduced redundancy when VIO/output
remain fresh; it is not substituted with a command-derived velocity.

The configured deadlines are 0.5 s, matching the EKF `sensor_timeout`; future
hardware evidence may tighten them. G2.1 records output/input ages and reasons
but has no motion authority. G7 alone may consume its diagnostic state for
autonomy disarm.

Test-only `Vy=0` is specifically excluded from production intent: an A/B test
proved that a continuously published pseudo-measurement can mask the all-input
dropout that this contract must expose.
