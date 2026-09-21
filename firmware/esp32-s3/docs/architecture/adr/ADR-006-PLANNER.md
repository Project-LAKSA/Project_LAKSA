# ADR-006: planner selection

Primary evaluation candidate: official Smac State Lattice using a LAKSA control
set generated from the canonical footprint, resolution, curvature, and reverse
policy. Fallback comparison: Smac Hybrid, never silently substituted.

The legacy Hybrid implementation has proven collision semantic defects in the
recovered Humble stack, so it cannot be production-qualified without a pinned
upstream/backport decision and corpus proof. `NO_SAFE_PATH` is a successful
safety result. A planner result alone is never executable.
