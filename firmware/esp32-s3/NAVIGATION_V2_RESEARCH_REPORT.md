# Navigation V2 research report

Research date: 2026-09-21. Sources are upstream documentation, not blogs.

| Area | Evidence | V2 conclusion |
|---|---|---|
| Nav2 bringup | [First-Time Robot Setup](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/) | Build URDF, TF, odometry, footprint, lifecycle, and simulator in that order. |
| Frames | [REP-105](https://www.ros.org/reps/rep-0105.html) | Separate global `map`, continuous `odom`, and vehicle `base_link`; one authority per edge. |
| State estimation | [robot_localization configuration](https://docs.ros.org/en/kinetic/api/robot_localization/html/state_estimation_nodes.html) | EKF needs per-variable covariance discipline; avoid correlated duplicate ZED VIO/IMU fusion. |
| Planner | [Smac State Lattice](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/) | State Lattice supports precomputed vehicle-specific minimum control sets and is V2 primary candidate. |
| Controller | [Nav2 plugins](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/) | Evaluate MPPI Ackermann, RPP, and Vector Pursuit through the same corpus. |
| MPPI | [MPPI configuration](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/) | Ackermann motion model and final optimal trajectory validation are available, but do not replace global path safety gates. |
| Actuation | [Steering Controllers Library](https://control.ros.org/master/doc/ros2_controllers/steering_controllers_library/doc/userdoc.html) | Bicycle controller matches one traction + one steering virtual-axis topology; retain ESP32 safety authority. |
| Simulation | [Nav2 configuration guide](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/) | Loopback is suitable for fast testing; Gazebo is required for dynamics/sensor integration. |

## Platform decision

Humble remains the production baseline because JetPack/ZED/CUDA compatibility
is already proven there. Jazzy is the target for desktop simulation evaluation,
not an in-place Jetson migration. Kilted/Rolling documentation is used only to
identify future capabilities, not as a production dependency.

## Known upstream constraint

The recovered Humble Smac Hybrid implementation has a proven mismatch between
internal and PlannerServer collision semantics. V2 therefore compares State
Lattice before considering any pinned upstream overlay. A `SUCCESS` action is
not a safety result; only the V2 path gate is.
