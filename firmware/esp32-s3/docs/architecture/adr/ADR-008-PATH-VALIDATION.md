# ADR-008: fail-closed path validation

Use official `IsPathValid` on the exact post-planning/post-smoothing path,
then retain two LAKSA-specific checks because official Humble semantics do not
prove swept-volume continuity or Ackermann/cusp feasibility. Every smoother is
followed by the full gate. Any failure emits evidence and `PATH_REJECTED`; it
cannot reach Controller Server. This custom code is justified only for those
two missing safety properties and has a deterministic corpus exit criterion.
