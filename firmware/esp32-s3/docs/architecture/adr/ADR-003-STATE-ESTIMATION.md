# ADR-003: local state estimation

Use one `robot_localization` EKF as the candidate owner of `odom -> base_footprint`.
Fuse selected ZED VIO odometry and independent vehicle-speed/IMU observations
only after covariance and correlation review. Do not fuse the ZED IMU stream a
second time when its VIO covariance already incorporates it. Use per-variable
configuration, innovation rejection, and explicit sensor timeout/degraded
modes; do not use fake huge covariance as a switch.

The future body-attitude projection adapter alone owns
`base_footprint -> base_link`; robot_state_publisher owns only static mechanical
edges. The global mapper/localizer alone owns `map -> odom`. VIO loss: predict only,
then disarm autonomy when freshness/uncertainty bounds fail. IMU or speed loss
has separately documented reduced capability. Hardware calibration is pending.
