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

Health assessment must distinguish VIO dropout, speed dropout, and all-input
dropout; it must never keep a stale last command or stale localization healthy.
The G2 all-input synthetic fixture proves the estimator stops filtered/TF
publication after `sensor_timeout`; G7 must map that absence to a fault or
autonomy disarm rather than extrapolating it as a healthy pose.
