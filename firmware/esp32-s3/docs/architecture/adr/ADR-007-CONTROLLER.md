# ADR-007: controller selection

Benchmark Nav2 MPPI with `AckermannMotionModel` as primary candidate, Regulated
Pure Pursuit as simple reference/fallback, and Vector Pursuit when maintained
on the selected distro. Select only after identical V004/loopback metrics:
tracking, heading, collision, reverse/cusp behavior, CPU, p95/p99 latency, and
failure mode. No custom Adaptive Pure Pursuit is promoted by default.

Every controller receives only accepted paths and remains subject to collision
monitor, timeout, supervisor, and manual preemption.
