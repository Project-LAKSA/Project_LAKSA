cmake_minimum_required(VERSION 3.8)
project(laksa_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/drive_supervisor_node.py
  scripts/dashboard_node.py
  scripts/laksa_control_math.py
  scripts/lidar_cruise_math.py
  scripts/lidar_cruise_node.py
  scripts/lidar_rollout_core.py
  scripts/lidar_watchdog_node.py
  scripts/steering_calibration_node.py
  scripts/state_measurements_node.py
  scripts/tf_static_keepalive_node.py
  scripts/zed_health_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch web
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
