# LAKSA-original production audit handoff

## Scope

`production/laksa-mainline` is **not a declared golden mapping state**. It is
a clean LAKSA-original production candidate with active characterization
removed after a preservation commit. Do not infer that its mapping or
navigation is restored, qualified, or safe for autonomous operation.

Audit in read-only mode. Do not edit, commit, deploy, start hardware, or alter
the frozen references while reaching conclusions.

## Sanitized private-runtime evidence

The public branch now contains a complete semantic inventory of the divergent
Jetson source/configuration evidence without publishing raw files:

- `DIVERGENT_RUNTIME_INVENTORY.json` — nineteen hash-addressed private files;
- `SANITIZED_RUNTIME_DELTA.md` and `.json` — safe behavioral differences;
- `PRE_CHARACTERIZATION_EVIDENCE_MATRIX.md` — cross-period comparison.

Raw workspace/install files and raw diffs deliberately remain private. Do not
ask to reconstruct them from hashes or infer omitted source. Treat a semantic
finding as exactly scoped: a source difference proves divergence, not that it
was deployed during the historically best mapping session.

## Required comparisons

Compare this branch with:

- `autonomy-handoff-2026-09-02`;
- `autonomy_handoff/active_runtime`, `autonomy_handoff/runtime/ros_graph`, and
  `autonomy_handoff/runtime/systemd` when available in preserved evidence;
- `b178a91f3a87f90ae289f62588744bd7936a43f4`;
- Jetson source/runtime evidence from September 9–16;
- the locally retained September pre-final-stabilization freeze when accessible;
- preserved installed-runtime hashes in `PRODUCTION_SOURCE_DIVERGENCE.json`;
- the September-21 forensic snapshot if it is restored;
- `recovery/pre-characterization-mapping-planner-good`.

## Audit objective

Recover the actual pre-characterization multimodal mapping state, with
evidence for RPLIDAR, ZED RGB/depth, RTAB-Map, RF2O,
robot_localization/odometry, TF ownership, Field Lab pose path and latency.
Resolve the questions in `OPEN_MAPPING_QUESTIONS.md` using hashes and runtime
captures; do not select a configuration on aesthetics or branch naming.

The audit should first determine whether the historically qualified runtime was
the multimodal fused topology reflected in production-mainline, the compact
ZED-RTAB topology found in the divergent workspace, or a third dated runtime.
Do not implement any restoration while that conclusion remains unproven.

## Production separation facts

- Preservation commit: `990bc16d7711c4f6a5597a552af68d3cd36e5224`.
- Production-only removal commit:
  `053cb9a101df6d91c85091765a788d9fafe37bda`.
- Characterization runners/protocols and the supervisor's characterization
  input path were removed from the active tree, but remain recoverable in the
  preservation commit.
- No F1TENTH, RoboRacer, MuSHR, Waterloo, TUM-raceline, competition-stack, or
  Navigation V2 code was introduced by this branch.

## Auditor deliverable

Report each finding as `CONFIRMED`, `PROBABLE`, or `UNKNOWN`, cite exact
source/runtime evidence, describe source/workspace/install divergence, and
recommend the minimal evidence-backed mapping restoration only after the
historical topology has been proven.
