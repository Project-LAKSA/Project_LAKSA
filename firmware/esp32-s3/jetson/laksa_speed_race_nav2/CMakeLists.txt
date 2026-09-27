cmake_minimum_required(VERSION 3.8)
project(laksa_speed_race_nav2)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(ackermann_feasible_rpp_controller SHARED
  src/ackermann_feasible_rpp_controller.cpp
)
target_compile_features(ackermann_feasible_rpp_controller PUBLIC cxx_std_17)
target_include_directories(ackermann_feasible_rpp_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ackermann_feasible_rpp_controller
  geometry_msgs nav2_core nav2_costmap_2d nav2_regulated_pure_pursuit_controller
  pluginlib rclcpp rclcpp_lifecycle std_msgs tf2 tf2_ros
)
pluginlib_export_plugin_description_file(nav2_core ackermann_feasible_rpp_plugin.xml)
ament_export_include_directories(include)
ament_export_dependencies(nav2_core nav2_regulated_pure_pursuit_controller)

add_executable(rpp_lockstep_host src/rpp_lockstep_host.cpp)
target_compile_features(rpp_lockstep_host PUBLIC cxx_std_17)
ament_target_dependencies(
  rpp_lockstep_host
  geometry_msgs nav_msgs nav2_core nav2_costmap_2d nav2_map_server pluginlib
  rclcpp rclcpp_lifecycle std_msgs tf2 tf2_ros
)

install(
  TARGETS ackermann_feasible_rpp_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS rpp_lockstep_host DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ackermann_feasibility test/test_ackermann_feasibility.cpp)
  target_include_directories(test_ackermann_feasibility PRIVATE include)
endif()

ament_package()
