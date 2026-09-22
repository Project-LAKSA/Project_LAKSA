# A030A LiDAR Mapping Fusion Shadow

Production remains `ZED_ONLY` and publishes the existing `/zed_rtabmap/*`
outputs. `HYBRID_SHADOW` is an explicit experiment that adds only the guarded
`/laksa/lidar/scan_validated` stream and publishes under
`/laksa/mapping_shadow/*` with the distinct `mapping_shadow_map` frame.

The hybrid RTAB configuration uses RGB-D plus LaserScan for registration and
occupancy (`Reg/Strategy=2`, `Grid/Sensor=2`). These settings are based on the
previously used Project LAKSA RTAB configuration; installed Jetson capability
and runtime behavior must still be confirmed before deployment.

Field Lab selects one output family at a time. Its magenta LiDAR overlay is the
latest physical scan transformed from its stamped source frame into the shadow
map frame. It preserves the measured planar geometry and height, and never
extrudes or stores the scan as 3-D structure.

Hybrid startup fails closed with `HYBRID_UNAVAILABLE` when the validated scan
or LiDAR health is stale/bad. Switching back to `ZED_ONLY` is deliberate; no
automatic source switching or fake scan is present.

After comparable sessions, generate a factual table with:

```bash
ros2 run laksa_mapping mapping_shadow_compare --output /home/ubuntu/laksa_mapping_sessions/a030a_comparison.md
```
