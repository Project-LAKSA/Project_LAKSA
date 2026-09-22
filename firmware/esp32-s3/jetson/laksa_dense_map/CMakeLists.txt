cmake_minimum_required(VERSION 3.8)
project(laksa_dense_map)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(rclcpp REQUIRED)
find_package(rtabmap_conversions REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(RTABMap 0.22.0 REQUIRED COMPONENTS core)
find_package(sensor_msgs REQUIRED)

add_executable(dense_map_assembler src/dense_map_assembler.cpp)
target_compile_features(dense_map_assembler PUBLIC cxx_std_17)
target_include_directories(dense_map_assembler PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(dense_map_assembler
  diagnostic_msgs
  pcl_conversions
  rclcpp
  rtabmap_conversions
  rtabmap_msgs
  sensor_msgs
)
target_link_libraries(dense_map_assembler ${PCL_LIBRARIES} rtabmap::core)

add_executable(dense_map_decompression_probe src/dense_map_decompression_probe.cpp)
target_compile_features(dense_map_decompression_probe PUBLIC cxx_std_17)
target_include_directories(dense_map_decompression_probe PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(dense_map_decompression_probe
  pcl_conversions
  rclcpp
  rtabmap_conversions
  rtabmap_msgs
)
target_link_libraries(dense_map_decompression_probe ${PCL_LIBRARIES} rtabmap::core)

install(TARGETS dense_map_assembler dense_map_decompression_probe DESTINATION lib/${PROJECT_NAME})

ament_package()
