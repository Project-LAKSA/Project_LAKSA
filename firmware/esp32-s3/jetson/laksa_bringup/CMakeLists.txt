cmake_minimum_required(VERSION 3.8)
project(laksa_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/drive_supervisor_node.py
  scripts/drivetrain_conversion.py
  scripts/followpath_dry_run_probe.py
  scripts/integration_realtime_probe.py
  scripts/supervisor_safety_probe.py
  scripts/dashboard_node.py
  scripts/laksa_control_math.py
  scripts/lidar_cruise_math.py
  scripts/lidar_cruise_node.py
  scripts/lidar_rollout_core.py
  scripts/lidar_watchdog_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch web
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_control_math test/test_control_math.py)
  ament_add_pytest_test(test_lidar_cruise_math test/test_lidar_cruise_math.py)
  ament_add_pytest_test(test_release_safety_contract test/test_release_safety_contract.py)
  ament_add_pytest_test(test_zed_configuration test/test_zed_configuration.py)
  ament_add_pytest_test(test_nav2_runtime_contract test/test_nav2_runtime_contract.py)
  ament_add_pytest_test(test_canonical_runtime_modes test/test_canonical_runtime_modes.py)
endif()

ament_package()
