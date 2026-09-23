# Competition speed-race architecture

This is a narrow simulation-first racing stack for **Jetson Orin Nano / Ubuntu
22.04 / ROS 2 Humble**. It is intentionally separate from the frozen recovery
and Navigation V2 branches.

```text
canonical course -> pinned Waterloo minimum-curvature helpers -> raceline

F1TENTH Gym core (KS/RK4, LAKSA_PROXY_V0, Frenet, max_laps=3)
  -> /c1/odom + map->c1/base_link
  -> pinned Waterloo Pure Pursuit
  -> /c1/drive_request
  -> local validation/stepping authority
  -> one env.step per accepted command
  -> /c1/drive_applied + metrics
  -> lap 3 -> terminal zero -> evidence flush -> clean shutdown
```

`/sim_ground_truth_map` is simulator-only scoring data. It is not remapped to
`/map` and cannot be consumed by mapping, localization, raceline generation,
or race control.

The C1 adapter contains only ROS/configuration translation, command limits,
three-lap state, metrics and shutdown. It does not implement dynamics,
raceline optimization or path tracking.

No joystick, teleop, physical actuator, ESP32, VESC, or ZED process is part of
this competition simulation architecture.

## C1 recovered course asset

The canonical, user-approved LAKSA Speed Course is now stored in
[`course/canonical/speed_course`](course/canonical/speed_course). It has an
explicit `speed_course_map` frame, a frozen analytic geometry hash, 135 ft by
47 ft documented bounds, a 36-inch corridor, directed Start/Finish and ordered
gates. The simulator ground-truth occupancy asset is still never an input to
mapping, localization, raceline generation, or race control.

## C1 isolation boundary

Only `/c1/odom`, `/c1/drive_request`, and `/c1/drive_applied` carry C1 motion
state. `/drive`, `/cmd_vel`, `/laksa/command`, `/laksa/set_drive_command`,
micro-ROS, VESC and GPIO are forbidden. Runtime containers have no network,
host devices, production mounts, elevated capabilities or Docker socket.

C1 uses simulator ground truth by design. SLAM, particle-filter localization,
unknown-course exploration, physical sensor fusion, physical actuation and the
master GPIO permit remain later milestones.
