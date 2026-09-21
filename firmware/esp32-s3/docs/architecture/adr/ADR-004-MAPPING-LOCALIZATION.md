# ADR-004: navigation is decoupled from RGB-D reconstruction

RTAB-Map remains the recovered candidate for RGB-D reconstruction and Field Lab
3D visualization. V2 evaluates SLAM Toolbox + 2D localization against
RTAB-derived 2D navigation using identical rosbags/maps and measures
repeatability, CPU/GPU use, latency, reset behavior, and ramp performance.

Decision pending comparative replay: only one component may own `map -> odom`
in any mode. Navigation never depends on a point-cloud renderer. The recovered
RTAB mapping baseline remains unchanged until evidence supports a migration.
