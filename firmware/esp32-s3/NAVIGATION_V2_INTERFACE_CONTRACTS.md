# V2 interface contracts

## Robot and TF

`base_link` is the vehicle-body semantic root. `base_footprint` is its planar
ground projection for 2D navigation. One local estimator owns
`odom -> base_footprint`; one body-attitude projection adapter owns the
derived `base_footprint -> base_link`; one SLAM/localization component owns
`map -> odom`. `robot_state_publisher` publishes only static body/sensor
geometry.

## Navigation and safety

Nav2 consumes `nav_msgs/Odometry` in `odom`, TF, a polygon footprint generated
from `vehicle_contract.yaml`, and standard costmap sources. A `nav_msgs/Path`
is only a candidate until all gates report pass:

1. `nav2_msgs/srv/IsPathValid` on the exact returned path;
2. continuous swept polygon collision validation;
3. Ackermann curvature, cusp, and steering-continuity validation.

The gate publishes `PATH_ACCEPTED` or `PATH_REJECTED` with evidence. It never
publishes vehicle motion. Controller output is a requested Twist only; the
supervisor is the sole autonomy-to-ESP32 bridge.

## Authority

`E_STOP/hardware safety > Xbox manual > approved autonomy > all other inputs`.
Field Lab, route UI, and simulation dashboards have zero motion authority.
Any freshness, lifecycle, localization, map, TF, or validator failure yields a
zero autonomous request and disarms autonomy.
