# Canonical LAKSA runtime

There are exactly two production operating modes. Historical A045/A046
experiments are diagnostics only and are not sources of runtime truth.

## LAKSA_MAPPING_MODE

Entrypoint: `laksa_mapping/launch/mapping_stack.launch.py`, owned by
`mapping_session_manager` and started from Field Lab.

The official ZED wrapper, `robot_localization`, `rtabmap_sync`, and RTAB-Map
produce the map. RTAB-Map is the sole `map -> odom` authority. Manual Xbox is
the only motion authority while mapping.

## LAKSA_NAVIGATION_MODE

Entrypoint: `laksa_dashboard/launch/navigation_saved_map.launch.py`, started
only after a successful transactional TAKE SNAPSHOT.

The official Nav2 map server loads the saved occupancy map and official AMCL
is the sole `map -> odom` authority. Official SmacPlannerHybrid and MPPI
Ackermann perform planning and path following. RTAB-Map is absent.

## Shared contracts

- TF: `map -> odom -> base_footprint`
- fused odometry topic: `/laksa/odometry/fused`
- wheelbase: `0.324 m`
- minimum turning radius: `1.09 m`
- host ROS middleware: `/etc/laksa/ros-runtime.env`
- autonomy starts DISARMED and Xbox always has highest authority

`laksa-planning-preview.service`, shadow/dual-map launches, ExactTime tests,
component-composition tests, and old frame aliases are not canonical runtime
entrypoints.
