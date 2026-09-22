# Open mapping questions for independent audit

This branch intentionally does not answer these questions. Status is limited to
the available repository and runtime evidence.

1. **Which exact pre-characterization runtime produced the best physical mapping?**
   **UNKNOWN.** The September freeze proves a captured runtime, not a measured
   ranking of mapping quality.
2. **Did final good RTAB-Map receive LaserScan plus RGB-D?**
   **PROBABLE.** The installed LiDAR service explicitly describes calibrated
   ZED/RPLIDAR RTAB-Map mode; exact final runtime graph must be reconciled.
3. **Was `Grid/Sensor=2` active in the final good state?**
   **UNKNOWN.** Parameter snapshots/configuration must be compared by the
   auditor; this task made no selection.
4. **Which LiDAR topic fed RTAB-Map (`/scan`, `/scan_raw`,
   `/laksa/lidar/scan_validated`, or other)?** **UNKNOWN.**
5. **Was RF2O active in the final good mapping state?** **PROBABLE.** The
   workspace contained `rf2o_laser_odometry`; process/launch evidence must
   establish whether it was active for the qualifying session.
6. **Which component owned `odom -> base`?** **UNKNOWN.**
7. **Was robot_localization active?** **UNKNOWN.**
8. **Which exact odometry topic fed RTAB-Map?** **UNKNOWN.**
9. **Which component owned `map -> odom`?** **UNKNOWN.**
10. **Which source/config changes occurred immediately before characterization
    deployment?** **UNKNOWN.** The source checkout was dirty and source,
    workspace, and install diverged.
11. **Did characterization deployment overwrite source, workspace source,
    install, systemd, or environment?** **CONFIRMED for source/workspace/install
    divergence; UNKNOWN for causal attribution.**
12. **What produced the historical low-latency Field Lab pose?** **UNKNOWN.**

The audit must cite exact files, hashes, service definitions, ros-graph/parameter
captures, and freeze artifacts before deciding any of these questions.
