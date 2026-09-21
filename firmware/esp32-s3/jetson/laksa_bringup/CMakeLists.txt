cmake_minimum_required(VERSION 3.8)
project(laksa_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/drive_supervisor_node.py
  scripts/laksa_control_math.py
  scripts/drivetrain_conversion.py
  scripts/production_safety.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
