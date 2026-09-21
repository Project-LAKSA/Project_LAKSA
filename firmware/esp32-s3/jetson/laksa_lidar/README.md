# LAKSA RPLIDAR sensor foundation

This package is an isolated Phase 1 boundary for the RPLIDAR A2M12. It does
not connect the LiDAR to mapping, planning, collision handling, or motion.

## Contract

- Raw input: `/scan_raw` (`sensor_msgs/msg/LaserScan`) from the existing
  `sllidar_ros2` driver owner.
- Validated output: `/laksa/lidar/scan_validated`. A message is republished
  only when its structure is valid, it is fresh on arrival, and its stamped
  scan frame can transform to `base_footprint`. The original message is
  published without filtering or field changes.
- Health: `/diagnostics` (`LAKSA/RPLIDAR_A2M12`) and the latched text state
  `/laksa/lidar/health_state` (`GOOD`, `DEGRADED`, or `BAD`).

The launch defaults to `use_existing_scan:=true`, so it cannot claim the
serial device accidentally. Set it false only after verifying `/scan_raw` has
no publisher. The optional driver uses the established `/dev/laksa_lidar`
udev link and A2M12 baud rate 256000. It deliberately publishes no TF: the
measured `base_footprint -> lidar_link` transform belongs to the existing
robot description.

The normal sensor launch stamps `/scan_raw` as `lidar_link`. Field Lab owns
the canonical robot description, whose measured transform is
`base_footprint -> lidar_link` independently of LiDAR availability. The old
`laksa_base_footprint -> laksa_lidar`
static publisher is available only through the explicitly disabled-by-default
`legacy_lidar_tf` launch argument for old demos.

Future RTAB fused-map, Nav2 ObstacleLayer, and collision-monitor consumers
must subscribe to the validated topic. They are intentionally not implemented
in Phase 1.

`record_lidar_validation [OUTPUT_DIR]` records only raw/validated scans,
diagnostics, TF, and fused odometry; it does not include ZED streams.
