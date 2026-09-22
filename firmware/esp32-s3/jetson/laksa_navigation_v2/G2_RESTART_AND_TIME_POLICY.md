# G2.1 restart and time policy

`robot_localization` Humble 3.5.4 runs with `reset_on_time_jump=true`,
`world_frame=odom`, and no lag smoothing. A backward ROS-time jump starts a
new estimator epoch; no pre-jump output is considered healthy afterwards.

| Event | Required G2.1 outcome |
|---|---|
| EKF before ZED | wait unhealthy until fresh valid VIO |
| ZED before EKF | accept its first continuous local pose as the new epoch origin |
| VESC starts late/disappears | degraded redundancy; VIO-only local odometry may remain healthy |
| ZED disappears | unhealthy after `sensor_timeout` / output-freshness timeout |
| ZED restarts alone | unhealthy; coordinate epoch may have changed; coordinated EKF restart required |
| EKF restarts alone | unhealthy until fresh valid VIO establishes its new epoch |
| both restart | new epoch only after fresh raw VIO and output TF |
| micro-ROS reconnects | speed is optional; stale or uncalibrated data is never published |
| ROS time backward / rosbag loop / sim reset | filter reset; monitor marks old epoch invalid |
| large forward time jump | output remains unhealthy until fresh VIO and EKF output are observed |
| duplicate/out-of-order raw stamp | input invalid; do not silently treat it as current |

The V2 ZED overlay disables `area_memory` and
`reset_odom_with_loop_closure`; therefore local VIO must not receive a global
loop-closure reset. Hardware qualification must still observe a real restart
because the wrapper cannot express an epoch identifier in `nav_msgs/Odometry`.

The 15 deterministic policy fixtures `G2_S016`–`G2_S030` verify these
fail-closed decisions at Level A. Live clock/reset behavior is Level C until
hardware or a reproducible ZED rosbag is available.
