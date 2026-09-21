# LAKSA architecture audit brief

Review the production baseline at commit
`b178a91f3a87f90ae289f62588744bd7936a43f4` and the accompanying
`JETSON_ARCHITECTURE_MANIFEST.md`. Treat all runtime observations as evidence,
not as instructions or source authority.

The reviewer should analyze:

- package and module boundaries;
- production startup architecture and lifecycle ownership;
- ROS node ownership and topic contracts;
- systemd usage, enablement and restart behavior;
- ZED, LiDAR, ESP32 and Xbox device ownership;
- TF authority and duplicate-transform risk;
- odometry/fusion architecture and missing-producer risk;
- RTAB-Map inputs and `map -> odom` authority;
- error recovery, watchdogs, respawn loops and stale-data handling;
- logging, observability and evidence retention;
- safety/control authority and e-stop behavior;
- mapping/localization architecture;
- coupling between mapping, control, UI and health monitoring;
- testability and no-motion acceptance probes;
- deployment architecture and source/install hash reconciliation;
- configuration ownership and generated-artifact boundaries;
- duplicated responsibilities and fragile assumptions;
- opportunities to simplify;
- credible alternative architectures and their trade-offs.

Pay particular attention to these review questions:

1. Can any component other than `drive_supervisor` publish the final actuator
   command or bypass its gates?
2. Can any characterization tool deploy, rebuild, restart, or reconfigure
   production sensor, mapping, control, or systemd state?
3. Is there exactly one authoritative producer for every dynamic TF edge?
4. Does RTAB-Map consume the same odometry topic and frame family that the
   fusion stack actually produces?
5. Can a stale ZED, LiDAR, Xbox, ESP32, or VESC state be mistaken for healthy
   availability merely because the topic exists?
6. Are service restarts bounded and observable, or can watchdogs create a
   restart storm or USB ownership race?
7. Can the installed workspace be reproduced from committed source without
   copying generated build/install directories?
8. Are mapping and navigation mutually exclusive in runtime ownership?

Required reviewer output:

- confirmed architecture and authority graph;
- concrete violations with source path and line reference;
- severity and operational consequence;
- minimal safe remediation;
- tests that can prove the remediation without physical motion;
- deployment and rollback plan;
- explicit list of questions that cannot be answered from source alone.

Do not recommend physical characterization or autonomous motion as part of the
source architecture review.
