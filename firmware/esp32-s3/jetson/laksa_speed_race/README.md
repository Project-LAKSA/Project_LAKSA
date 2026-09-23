# LAKSA Competition Speed Race

This directory is an isolated **competition-only** integration area. It does
not modify the recovered mapping stack, Navigation V2, ESP32 firmware, or the
physical command path.

The intended simulation-only pipeline is:

```text
F1TENTH Gym ROS -> /scan + /ego_racecar/odom
  -> Follow-the-Gap + slam_toolbox -> saved map
  -> upstream map-to-centerline + TUM raceline tool
  -> particle filter + upstream Pure Pursuit -> /drive
  -> F1TENTH Gym ROS
```

`/sim_ground_truth_map` is reserved for simulator truth and is never an input
to mapping, localization, raceline generation, or race control.

## Current checkpoint

C0 is complete: exact external refs and licenses are pinned in
[`speed_race_upstream.repos`](speed_race_upstream.repos), and
[`UPSTREAM_PROVENANCE.md`](UPSTREAM_PROVENANCE.md) records the selection
evidence. C1 now has a recovered, user-approved canonical course asset in
[`course/canonical/speed_course`](course/canonical/speed_course), plus offline
three-lap and metrics contracts. C1 cannot yet execute the official
closed-loop simulation because the selected upstream Gym ROS bridge does not
expose a configuration-only custom LAKSA vehicle-parameter interface. No
invented simulator or controller is substituted.

See [`SPEED_RACE_RESULTS.json`](SPEED_RACE_RESULTS.json) for machine-readable
status and exact blockers. Do not launch any legacy manual-control, joystick,
or physical actuation package from this directory.
