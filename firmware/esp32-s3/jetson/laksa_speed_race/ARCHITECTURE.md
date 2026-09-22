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

## C0 finding that blocks C1--C5

The selected upstream `f1tenth_gym_ros` bridge accepts only
`vehicle_params: f1tenth|f1fifth|fullscale`. Although its underlying Gym API
can model arbitrary `VehicleParameters`, the ROS bridge has no configuration
surface for LAKSA's wheelbase, mass, asymmetric steering constraints, or
conservative steering bound. Creating another bridge or patching upstream
would violate the sprint constraints.

The repository also contains no approved Speed Course map, image, GeoJSON, or
reconstruction. The user-provided envelope is retained as a provisional
simulation requirement, not fabricated as a competition track.

Therefore no C1 vehicle/course qualification or later autonomy claim is made
until the two blockers are resolved using an upstream-supported mechanism or
an explicitly authorized exception.
