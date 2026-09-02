# LAKSA Current Runtime State

## Active workspace
/home/ubuntu/laksa_ws (ROS 2 Humble), with runtime source at /home/ubuntu/laksa_ws/src/laksa_bringup.

## Active repository
/home/ubuntu/src/Project_LAKSA. The active runtime package is a diverged copy and is preserved separately without synchronization.

## Git branch/HEAD before handoff
main at cba74e7fed8e262a878186b375b2cb7a75bab20c — Added initial micro ROS support to the esp32.

## systemd services
laksa-control-navigation.service, laksa-lidar-mapping.service, laksa-zed-camera.service, and laksa-micro-ros-agent.service are the verified startup roots. Unit state and referenced startup scripts are included under runtime/.

## Active launch tree
laksa-control-navigation.service → laksa_system.launch.py → joystick/manual control, drive supervisor, Nav2, exploration and state-estimation nodes.  
laksa-lidar-mapping.service → laksa_3d_mapping.launch.py → LiDAR filtering, RF2O, EKF, SLAM/RTAB-Map mapping hierarchy.  
laksa-zed-camera.service → ZED wrapper. laksa-micro-ros-agent.service → micro-ROS agent.

## LiDAR flow
sllidar → /scan → laser filters/navigation consumers; filtered scan → RF2O → /laksa/rf2o_odom → EKF.

## ZED flow
ZED publishes RGB, depth, point cloud and /zed/zed_node/odom. Current endpoint evidence is saved under runtime/ros_graph/.

## EKF flow
ESP32-derived state/IMU, RF2O odometry and ZED odometry → /ekf_filter_node → /laksa/odom and odom/base TF.

## Nav2 flow
Nav2 planner/controller/behavior/BT nodes → /laksa/nav_cmd_vel → drive supervisor.

## Exploration flow
lidar_cruise/exploration → /laksa/lidar_cruise_cmd_vel → drive supervisor.

## Final drive flow
manual, Nav2 and exploration requests → drive supervisor → /laksa/command and /laksa/brake → micro-ROS/ESP32 → VESC and steering.

## RTAB-Map status
A006 found RTAB-Map stopped after an invalid yaw information matrix error caused by unbounded EKF yaw uncertainty. Runtime graph files in this snapshot record its current post-change state without restarting it.

## Current yaw configuration
ZED odometry is EKF input odom1. ZED pose yaw is enabled and odom1_differential = false, so yaw is fused as an absolute measurement. BNO08X orientation was unavailable to the EKF.

## A011 results
Absolute ZED yaw bounded EKF yaw uncertainty while stationary. Previous yaw covariance growth was approximately 0.0600016/s; after the change it was approximately 0.0000150534/s. EKF and ZED remained alive and odometry had one publisher.

## A012 results
The ZED-only restart safety test passed its safety gate. ZED returned, EKF continued publishing, and no unexpected propulsion command was observed. No configuration was changed during that verification.
