cmake_minimum_required(VERSION 3.8)
project(laksa_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/drive_supervisor_node.py
  scripts/vehicle_identification_dry_run.py
  scripts/run_vehicle_identification_production_env.sh
  scripts/room_scale_longitudinal_recorder.py
  scripts/run_room_scale_longitudinal_production_env.sh
  scripts/analyze_room_scale_longitudinal.py
  scripts/automated_characterization_runner.py
  scripts/automated_steering_characterization_runner.py
  scripts/characterization_safety.py
  scripts/interactive_ros.py
  scripts/t21_imu_sanity_runner.py
  scripts/t21_offline_adjudicate.py
  scripts/t21a_offline_analyze.py
  scripts/batch_characterization_manifest.py
  scripts/batch_session_lease.py
  scripts/batch_executor_core.py
  scripts/local_campaign_core.py
  scripts/t25_t31_guided_capture.py
  scripts/local_campaign_executor.py
  scripts/model_fit.py
  scripts/held_out_validation.py
  scripts/campaign_report.py
  scripts/batch_characterization_runner.py
  scripts/characterization_stage_tools.py
  scripts/characterization_stage_runner.py
  scripts/supervised_stage_protocols.py
  scripts/supervised_stage_runner.py
  scripts/twin_characterization_roadmap.py
  scripts/drivetrain_conversion.py
  scripts/characterization_campaign_runner.py
  scripts/characterization_gate_probe.py
  scripts/characterization_isolated_integration.py
  scripts/followpath_dry_run_probe.py
  scripts/integration_realtime_probe.py
  scripts/supervisor_safety_probe.py
  scripts/dashboard_node.py
  scripts/laksa_control_math.py
  scripts/lidar_cruise_math.py
  scripts/lidar_cruise_node.py
  scripts/lidar_rollout_core.py
  scripts/lidar_watchdog_node.py
  scripts/state_measurements_node.py
  scripts/steering_calibration_node.py
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
  ament_add_pytest_test(test_local_campaign_core test/test_local_campaign_core.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
endif()

ament_package()
