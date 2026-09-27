cmake_minimum_required(VERSION 3.8)
project(laksa_speed_race_nav2)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(rpp_lockstep_host src/rpp_lockstep_host.cpp)
target_compile_features(rpp_lockstep_host PUBLIC cxx_std_17)
ament_target_dependencies(
  rpp_lockstep_host
  geometry_msgs nav_msgs nav2_core nav2_costmap_2d nav2_map_server pluginlib
  rclcpp rclcpp_lifecycle std_msgs tf2 tf2_ros
)

install(TARGETS rpp_lockstep_host DESTINATION lib/${PROJECT_NAME})

ament_package()
