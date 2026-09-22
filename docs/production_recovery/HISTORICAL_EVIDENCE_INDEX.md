# Historical evidence index

## Read-only sources inspected

- Jetson checkout `/home/ubuntu/src/Project_LAKSA`, committed at
  `b178a91f3a87f90ae289f62588744bd7936a43f4` on
  `a028-manual-mapping-cockpit`, with tracked and untracked changes.
- Jetson workspace source `/home/ubuntu/laksa_ws/src`, which included
  `laksa_mapping`, `laksa_bringup`, `laksa_dashboard`, `laksa_lidar`,
  `laksa_dense_map`, `laksa_planning_lab`, `rf2o_laser_odometry`, and
  `frontier_exploration_ros2`.
- Jetson installed runtime `/home/ubuntu/laksa_ws/install` and service
  entrypoints under `/usr/local/lib/laksa`.
- Preserved freeze
  `/home/ubuntu/laksa_freezes/pre_final_stabilization_20260913T010911Z`:
  source patch, source/install snapshots, mapping hashes, runtime process and
  parameter captures, Field Lab state, and systemd captures. Its manifest
  records `b178a91f3a87f90ae289f62588744bd7936a43f4` and a matching mapping
  hash reference at capture time.
- `/private/tmp/laksa-forensic-20260921/` was absent at inspection time.

## Branch/reference evidence for the next audit

- `autonomy-handoff-2026-09-02` at `4787d05d3c5bdfc0be1c190a45ab32f8da27667d`.
- `a028-manual-mapping-cockpit` / source head
  `b178a91f3a87f90ae289f62588744bd7936a43f4`.
- `recovery/pre-characterization-mapping-planner-good` at
  `c9fe6b9487b88753cbae575ac53bef23ec28b2d8`.
- The freeze's runtime assets: `runtime/`, `ros/`, `systemd/`, and `installed/`.

This is an evidence index, not a ranking of configurations. In particular,
the active recovery configuration must not overwrite evidence of historically
integrated RPLIDAR + ZED RGB-D + RTAB-Map operation.
