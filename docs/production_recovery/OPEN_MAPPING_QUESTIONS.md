# Open mapping questions for independent audit

This branch intentionally does not answer these questions. Status is limited to
the available repository and runtime evidence.

1. **Which exact pre-characterization runtime produced the best physical mapping?**
   **UNKNOWN.** The September freeze proves a captured runtime, not a measured
   ranking of mapping quality.
2. **Did final good RTAB-Map receive LaserScan plus RGB-D?**
   **PROBABLE.** September-2 runtime evidence and the current source declare
   multimodal intent. The private workspace launch is instead ZED RGB-D/VIO
   only, so it cannot by itself establish the final good topology.
3. **Was `Grid/Sensor=2` active in the final good state?**
   **UNKNOWN.** No private-file delta changed that setting, while public
   historical evidence contains `Grid/Sensor` references. The final runtime
   value still requires a dated parameter/configuration reconciliation.
4. **Which LiDAR topic fed RTAB-Map (`/scan`, `/scan_raw`,
   `/laksa/lidar/scan_validated`, or other)?** **UNKNOWN.** The current
   fused launch names the validated topic; the divergent workspace launch
   names no LiDAR input. Neither proves the final good physical run.
5. **Was RF2O active in the final good mapping state?** **PROBABLE.** Sep-02
   graph evidence includes RF2O; no private divergent-file delta references it.
   Process/launch evidence must establish whether it was active for the
   qualifying session.
6. **Which component owned `odom -> base`?** **UNKNOWN.** The private delta
   proves only a topology/frame difference, not runtime TF publisher authority.
7. **Was robot_localization active?** **PROBABLE.** Sep-02 has an EKF
   parameter capture and current fused source launches an EKF; the divergent
   workspace mapping launch omits it. Runtime-date correlation remains needed.
8. **Which exact odometry topic fed RTAB-Map?** **CONFIRMED as a source
   difference, UNKNOWN as historical truth.** Current fused source names
   `/laksa/odometry/fused`; the divergent workspace names raw ZED odometry.
9. **Which component owned `map -> odom`?** **UNKNOWN.**
10. **Which source/config changes occurred immediately before characterization
    deployment?** **UNKNOWN.** The source checkout was dirty and source,
    workspace, and install diverged.
11. **Did characterization deployment overwrite source, workspace source,
    install, systemd, or environment?** **INCONCLUSIVE.** The private
    workspace supervisor contains disabled characterization interfaces, and
    source/workspace/install divergence is confirmed for selected artifacts,
    but no deployment record proves causation.
12. **What produced the historical low-latency Field Lab pose?** **UNKNOWN.**
    The private dashboard delta establishes different data/lifecycle behavior,
    not a timing cause.

See `SANITIZED_RUNTIME_DELTA.md` and
`PRE_CHARACTERIZATION_EVIDENCE_MATRIX.md`. The audit must cite exact files,
hashes, service definitions, ros-graph/parameter captures, and freeze artifacts
before deciding any of these questions.
