# ADR-001: retain Humble on the current Jetson; make V2 portable to Jazzy

## Context

The recovered Jetson baseline is tied to its current JetPack/L4T, CUDA, ZED
SDK/wrapper 5.4.1, and ROS 2 Humble environment. Modern Gazebo guidance targets
Jazzy+, but an OS migration risks the recovered mapping runtime.

## Alternatives

1. Keep Humble production now.
2. Source-overlay newer Nav2 selectively.
3. Containerize newer userland on current L4T.
4. Migrate JetPack/Ubuntu/ROS later.

## Decision

Keep the recovered host on Humble for production and pin all overlays. Build V2
interfaces/configuration distro-portably; evaluate Jazzy in desktop/CI
simulation first. Do not source-overlay Nav2 as a production fix until a
pinned upstream change passes the corpus. Revisit OS migration only after a
ZED/CUDA/JetPack compatibility matrix and hardware qualification.

## Consequences / rollback

Humble lacks some newer features and remains subject to known planner defects;
the safety gate compensates. Rollback is selecting the frozen baseline branch.
