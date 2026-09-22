# A029 Path Planning Preview

A029 adds a virtual, planner-only route preview to PROJECT LAKSA Field Lab. A
tap on known free space in the 2-D RTAB-Map occupancy map requests
`nav2_msgs/action/ComputePathToPose`; the resulting `nav_msgs/Path` is drawn in
the browser and published on `/laksa/planning/preview_path` for diagnostics.

The isolated runtime contains only:

- Nav2 `planner_server` with `nav2_smac_planner/SmacPlannerHybrid`
- `REEDS_SHEPP` motion primitives for forward/reverse Ackermann paths
- a global costmap sourced from `/zed_rtabmap/map`
- `laksa_planning_preview_lifecycle_manager`, managing only `planner_server`

It intentionally contains no controller, local costmap, behavior server, BT
navigator, waypoint follower, velocity smoother, or vehicle-command publisher.
A path displayed by Field Lab cannot move the vehicle.

## Required calibration

Planning is locked in `UNCALIBRATED` until
`laksa_dashboard/config/planning_geometry.yaml` contains measured values for:

- `wheelbase_m`
- either `minimum_turning_radius_m` or
  `effective_max_road_wheel_steering_deg`
- footprint extents `front_m`, `rear_m`, `left_m`, and `right_m`, measured from
  `base_footprint`
- `calibrated: true`, set deliberately after the measurements are reviewed

The steering angle is the effective road-wheel angle, not the PCA9685 servo
command. The former visualization-only chassis dimensions are not used.

## Runtime ownership

`laksa-planning-preview.service` is not enabled by its unit and must remain an
explicit operator-started experiment. The normal `laksa-control-navigation`
boot path remains Xbox/manual only. Starting Planner Server does not grant
motion authority.
