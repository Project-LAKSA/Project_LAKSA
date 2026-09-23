# C1 implementation report

## Completed offline C1 foundation

- Recovered the existing user-approved canonical Speed Course without
  redrawing it.
- Added a versioned course manifest, source hashes, deterministic rebuild
  wrapper, course validator and an Ament Python package for installation.
- Added a simulation-only C1 contract, deterministic three-lap gate and
  metrics schema. These modules contain orchestration/evidence logic only;
  they do not implement a controller, optimizer, SLAM, localization, or
  simulator.
- Added static tests for course integrity, deterministic regeneration,
  three-lap stop behaviour, no reverse metric masking, no invented CTE pass
  threshold and physical-topic isolation.

## Upstream evidence revalidated

The exact pinned commits in `speed_race_upstream.repos` were read in temporary
clones. `f1tenth_gym_ros@08395766c4d9dc5a763381f1dd6fa4a3d68df66e` confirms
that its `gym_bridge` accepts only `f1tenth`, `f1fifth`, or `fullscale` vehicle
presets. Its underlying pinned Gym core is MIT licensed and can model arbitrary
vehicle parameters, but the selected ROS bridge exposes no LAKSA_PROXY_V0
parameter surface.

`CL2-UWaterloo/Raceline-Optimization@9290c5d503462e46f7e3e9033002e7ddf165ba7b`
is LGPL-3.0 and remains an external-only tool. Its documented input is a
centreline/reference-track CSV with left/right widths, which the recovered
`centerline.csv` provides. Its output contract is recorded in
`course_manifest.json`; no optimizer code is copied into LAKSA.

`CL2-UWaterloo/f1tenth_ws@c20cf63d04b9841ffdb6b2f963bd737d78074136`
contains an MIT Pure Pursuit package that publishes `AckermannDriveStamped` to
`/drive`. It remains an external dependency; no controller code is copied.

## Current C1 boundary

`C1_COURSE_ASSET=PASS`.

No official closed-loop C1 run is claimed. It remains blocked before simulator
execution by the existing upstream bridge limitation:

`UPSTREAM_CAPABILITY_GAP=F1TENTH_GYM_ROS_CONFIGURABLE_LAKSA_VEHICLE_PARAMETERS`.

The permitted next implementation, if separately pursued, is a thin
simulation-only interface adapter around the exact pinned F1TENTH Gym core
that exposes LAKSA_PROXY_V0 parameters and only `/drive`, `/scan`, odometry,
collision and simulator truth topics. It must not alter upstream algorithms,
expose a physical topic, or replace the selected external Pure Pursuit and
raceline components.

Until that adapter and the external optimizer/controller execute in a ROS 2
Humble environment, `raceline.csv`, the official three-lap run, CTE
distribution, collision/off-track metrics and repeatability results are
`NOT_EXECUTED` rather than fabricated.
