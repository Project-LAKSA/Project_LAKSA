# Competition speed-race architecture

This is a narrow simulation-first racing stack for **Jetson Orin Nano / Ubuntu
22.04 / ROS 2 Humble**. It is intentionally separate from the frozen recovery
and Navigation V2 branches.

```text
F1TENTH Gym ROS (sim only)
  /scan + /ego_racecar/odom + collision
        |              |
        v              v
Follow-the-Gap     slam_toolbox (mapping mode)
        |              |
        +-- /drive     +-- /map (never simulator truth)
                              |
                    save/freeze map
                              |
                 upstream centerline + TUM raceline tool
                              |
   frozen map + scan + odometry -> upstream particle filter
                              |
                 upstream Pure Pursuit -> /drive
                              |
                   three-lap mission orchestration -> stop
```

`/sim_ground_truth_map` is simulator-only scoring data. It is not remapped to
`/map` and cannot be consumed by mapping, localization, raceline generation,
or race control.

The only permitted LAKSA-authored runtime code in later milestones is a thin
mission state machine: `START`, `EXPLORE_MAP`, `MAP_COMPLETE`, `SAVE_MAP`,
`GENERATE_RACELINE`, `INITIALIZE_LOCALIZATION`, `RACE`, `FINISH`, and
`FAILURE`. It must not implement SLAM, localization, map-to-centerline,
raceline optimization, path tracking, or exploration.

No joystick, teleop, physical actuator, ESP32, VESC, or ZED process is part of
this competition simulation architecture.

## C1 recovered course asset

The canonical, user-approved LAKSA Speed Course is now stored in
[`course/canonical/speed_course`](course/canonical/speed_course). It has an
explicit `speed_course_map` frame, a frozen analytic geometry hash, 135 ft by
47 ft documented bounds, a 36-inch corridor, directed Start/Finish and ordered
gates. The simulator ground-truth occupancy asset is still never an input to
mapping, localization, raceline generation, or race control.

## Remaining C1 upstream boundary

The selected upstream `f1tenth_gym_ros` bridge accepts only
`vehicle_params: f1tenth|f1fifth|fullscale`. Although its underlying Gym API
can model arbitrary `VehicleParameters`, the ROS bridge has no configuration
surface for LAKSA's wheelbase, mass, asymmetric steering constraints, or
conservative steering bound. Creating another bridge or patching upstream
would violate the sprint constraints.

The course-geometry blocker is closed. No C1 vehicle qualification or
closed-loop autonomy claim is made until the simulator interface can represent
LAKSA_PROXY_V0 through an upstream-supported mechanism or a separately scoped
simulation-only adapter. C2--C5 remain blocked.
