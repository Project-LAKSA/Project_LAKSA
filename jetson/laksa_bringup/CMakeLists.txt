cmake_minimum_required(VERSION 3.8)
project(laksa_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/drive_supervisor_node.py
  scripts/dashboard_node.py
  scripts/frontier_exploration_adapter_node.py
  scripts/lidar_watchdog_node.py
  scripts/steering_calibration_node.py
  scripts/vesc_ackermann_odometry_node.py
  scripts/xbox_drive_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch web
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
