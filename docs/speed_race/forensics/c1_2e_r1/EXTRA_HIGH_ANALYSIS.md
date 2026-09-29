# C1.2e Extra High causal-analysis specification

You are analyzing an already completed, exactly reproduced C1.2e-R1 Nav2
MPPI Ackermann failure. Work only from this Git checkout. Do not access the
original Mac, run ROS/Nav2/Gym, start a simulator, tune parameters, or modify
controller code. The output is analysis and a proposed next experiment, not an
implementation.

## 1. Locate and verify the evidence

Discover the repository root from the current workspace; do not assume an OS
or absolute path. Locate this directory at
`docs/speed_race/forensics/c1_2e_r1/`. Before drawing conclusions:

1. Read `README.md`, `FORENSIC_MANIFEST.txt`, `replay_summary.json`,
   `provenance.json`, and `instrumentation_manifest.txt`.
2. Run or inspect `verify_forensics.py`; independently verify artifact hashes
   against `FORENSIC_MANIFEST.txt`.
3. Verify `REPLAY_REPRODUCTION=PASS_EXACT`, 1326 total steps, and first
   off-track at step 1326.
4. Verify Gym SHA `bdaec1420c3b0f103858d289866d0d4e2e597c30`, Nav2 SHA
   `a097086719c88f781aa59788eca29ac6ca5e56db`, and baseline repository SHA
   `7e280465090ee98e4470082cd8dff898e30d8377`.
5. Verify canonical configuration and implementation using the
   repository-relative paths listed in `README.md`.
6. Read `UPSTREAM_NAV2_MPPI_PROVENANCE.md` and the included pinned upstream
   source. Treat `instrumentation_nav2.diff` as the exact observational delta.

If provenance, hashes, row counts, or schema consistency fail, stop and report
the first blocker. The raw CSV/JSON evidence is authoritative. Do not silently
repair or normalize it.

## 2. Analysis discipline

- Establish explicit quantitative criteria before calculating each first-
  divergence or persistence result. State baseline window, noise floor,
  threshold, duration, and sign convention. Show sensitivity to reasonable
  neighboring thresholds so that the result is not an arbitrary crossing.
- Use signed errors for direction and absolute errors for magnitude.
- Distinguish controller evaluation `N` (pre-step state and command) from
  the resulting post-step state. Account for this one-step causal delay when
  assigning command effects.
- Treat wall-clock latency, ROS domain, process IDs, and absolute historical
  runtime paths as nondynamic metadata.
- Do not infer why a command was produced from steering alone. Use actual
  state, path-pipeline telemetry, configured critic semantics, per-critic
  contributions, optimizer semantics, and command response together.
- MPPI uses a weighted batch update. The diagnostic minimum-total-cost
  candidate is not a discrete selected trajectory. Phrase conclusions
  accordingly and do not claim that its per-critic values exactly decompose
  the continuous weighted control update.
- `INACTIVE_OR_ZERO` is ambiguous unless another captured reason resolves it.
- Attempt to falsify the hypothesis
  `MPPI_OBJECTIVE_PATH_REPRESENTATION_INCOMPATIBILITY`. PathAlignCritic
  inactivity alone does not prove causality.
- Separate the primary tracking failure, the safety-containment result, and
  physical real-time qualification. They may have different causes.

## 3. Divergence timeline

Using `replay_raw_telemetry.csv`, determine and justify:

- `FIRST_CTE_DIVERGENCE_STEP`
- `FIRST_HEADING_DIVERGENCE_STEP`
- `FIRST_CLEARANCE_DEGRADATION_STEP`
- `FIRST_PERSISTENT_DIVERGENCE_STEP`

Choose criteria before inspecting candidate crossing points. A valid approach
may combine initial-window trend/noise, sustained monotonic or regression
slope, and a persistence window, but report the exact method and threshold.
Cross-check each event against position, nearest global index, steering, speed,
and post-step response. Provide several rows before and after each event.

## 4. Controller-decision causality

Define the desired corrective steering sign geometrically from signed CTE,
path tangent, vehicle heading, and coordinate/sign conventions. Then find:

- `EARLIEST_WRONG_SIGN_COMMAND_STEP`
- `EARLIEST_WRONG_SIGN_COMMAND_EFFECT`
- `FIRST_CAUSAL_DIVERGENCE_STEP`
- `FIRST_CAUSAL_MECHANISM`
- `FIRST_CAUSAL_CONTROLLER_DECISION`
- `WHY_THAT_DECISION_WAS_SELECTED`

Do not equate a transient command sign with causal failure without showing the
state-relative corrective sign and its later dynamic effect. Explain the
weighted MPPI objective evidence, not a fictional one-trajectory selection.

## 5. Critic forensics

Use `critic_telemetry.csv`, the configuration, and included critic sources.
For every configured critic determine activation state, any identifiable
inactivity reason, raw diagnostic contribution, weighted diagnostic
contribution, and its relation to the actual output command.

Inspect at minimum:

- step 1;
- first CTE divergence;
- first heading divergence;
- first persistent divergence;
- approximately 25%, 50%, and 75% of the run;
- 100, 50, and 20 steps before failure;
- step 1325.

Rank absolute weighted diagnostic contributions at each checkpoint, while
preserving signs and exact zeros. Examine trends and discontinuities. Return:

- `DOMINANT_CRITICS_BEFORE_DIVERGENCE`
- `DOMINANT_CRITICS_DURING_DIVERGENCE`
- `DOMINANT_CRITICS_NEAR_FAILURE`
- `WHY_SELECTED_TRAJECTORY_WON` (interpret this required field as why the
  resulting weighted MPPI command was favored; explicitly correct the legacy
  field name in prose).

Attempt to falsify objective/path incompatibility by checking whether active
PathFollow, Goal, Constraint, PreferForward, Cost, PathAngle, and GoalAngle
signals could explain or oppose the observed command without PathAlign.

## 6. Path representation and PathAlign

Verify from evidence rather than assuming:

- global path count and length;
- local/transformed path count and length at the start, divergence points, and
  failure;
- local spacing distribution and global-index window progression;
- required PathAlign index 20 and maximum available index 7;
- PathAlign active/inactive counts and percentage;
- captured inactive reason.

Read the pinned `PathHandler`, PathAlign critic, utilities, costmap config, and
path publisher. Separate the effects of:

- nearest-pose search and destructive path pruning;
- `prune_distance`;
- costmap extent and first out-of-bounds return;
- global path spacing/density;
- TF frame and transformation behavior;
- `offset_from_furthest` and critic threshold logic;
- path orientations and `use_path_orientations`;
- path representation itself.

Determine:

- `PATH_REPRESENTATION_LIMITING_MECHANISM`
- `PATH_ALIGN_INACTIVITY_CAUSAL` as `YES`, `NO`, `CONTRIBUTORY`, or
  `INCONCLUSIVE`.

Do not label PathAlign inactivity causal merely because it is universal.
Seek counterevidence in phases where error is stable, other critic signals,
command sign, and path progression.

## 7. Recoverability

Define recoverability geometrically and dynamically using the vehicle
footprint, corridor, steering limit, observed speed, remaining clearance,
heading, and reachable curvature. Do not assume saturation: verify maximum
steering and saturation count. Estimate, with assumptions and bounds:

- `LAST_RECOVERABLE_STEP`
- `FIRST_UNRECOVERABLE_STEP`

If the available selected-command telemetry cannot prove counterfactual
reachability, report a bounded interval or `INCONCLUSIVE` rather than inventing
a precise step.

## 8. Safety-veto forensics

Read the independent veto implementation and compare its costmap collision
scope with Gym/ThreeLapGate full-body off-track scope. Quantify from available
state, footprint, corridor, and predicted-command evidence:

- `FIRST_OFFTRACK_PREDICTABLE_STEP`
- `FIRST_BOUNDARY_VETO_WOULD_TRIGGER_STEP`
- `BOUNDARY_PROTECTION_LEAD_STEPS`
- `BOUNDARY_PROTECTION_LEAD_TIME_S`
- `SAFETY_VETO_FALSE_NEGATIVE`
- `SAFETY_VETO_SCOPE_GAP`
- `OFF_TRACK_BOUNDARY_EQUALS_COSTMAP_LETHAL_BOUNDARY`

Do not invent unavailable projected clearance: the capture explicitly records
`NOT_AVAILABLE_BOOLEAN_COSTMAP_VETO`. If exact counterfactual prediction cannot
be derived without simulation, state the strongest defensible bound and mark
the exact field `INCONCLUSIVE` as appropriate. Distinguish collision safety
from competition track-boundary safety.

## 9. Horizon and speed

Verify from configuration:

- `time_steps=100`;
- `model_dt=0.01 s`;
- prediction horizon `1.0 s`;
- controller frequency `100 Hz`;
- actual and commanded speed trends.

Assess `HORIZON_GEOMETRICALLY_SUFFICIENT` using distance traveled over the
horizon, local path length, curvature/clearance geometry, and critic behavior.
Classify `SPEED_POLICY_CLASSIFICATION` as `CAUSAL`, `CONTRIBUTORY`,
`NON_CAUSAL`, or `INCONCLUSIVE`. Do not propose speed tuning before establishing
its role.

## 10. Real-time feasibility

Use the authoritative uninstrumented Trial-1 latency in
`docs/speed_race/C1_2D_NAV2_MPPI_ACKERMANN_REPORT.md` (approximately 25.160 ms
p50, 44.548 ms p95, 68.856 ms max) against a 10 ms controller period. Do not
use passive-instrumentation overhead as the principal benchmark. Determine:

- `PHYSICAL_REALTIME_FEASIBILITY`
- `PHYSICAL_CONTROLLER_FREQUENCY_QUALIFICATION`

Separate this physical deployment blocker from lockstep simulation causality.
Do not claim compute latency caused the off-track event unless dynamic evidence
supports that link.

## 11. Steering-rate demand

Using the per-step implied command rate, report maximum and percentages of
commands whose absolute rate exceeds strictly:

- 0.5 rad/s;
- 1.0 rad/s;
- 1.5 rad/s;
- 2.0 rad/s;
- 2.5 rad/s.

State numerator, denominator, treatment of the first sample, and whether
equality counts (use strict `>` for the requested fields). Keep
`PHYSICAL_STEERING_RATE_QUALIFICATION=PENDING`; digital command slew does not
qualify the physical servo.

## 12. Causal matrix

Classify each item as `CAUSAL`, `CONTRIBUTORY`, `NON_CAUSAL`, `REFUTED`, or
`INCONCLUSIVE`, with direct evidence and attempted falsification:

- `CAUSE_MPPI_OBJECTIVE`
- `CAUSE_PATH_REPRESENTATION`
- `CAUSE_PATH_ALIGN_ACTIVATION`
- `CAUSE_PATH_PRUNING`
- `CAUSE_PATH_TRANSFORM`
- `CAUSE_PATH_ORIENTATION`
- `CAUSE_TF_SEMANTICS`
- `CAUSE_ERROR_SIGN`
- `CAUSE_ACKERMANN_CONVERSION`
- `CAUSE_STEERING_LIMIT`
- `CAUSE_SPEED_POLICY`
- `CAUSE_PREDICTION_HORIZON`
- `CAUSE_SIMULATOR_DYNAMICS`
- `CAUSE_SAFETY_VETO_SCOPE`
- `CAUSE_COSTMAP_OFFTRACK_MISMATCH`
- `CAUSE_COMPUTE_LATENCY`
- `CAUSE_LOCKSTEP_SEMANTICS`
- `CAUSE_RNG`

For `CAUSE_ERROR_SIGN`, distinguish telemetry sign convention, controller
frame conventions, and an actual sign bug. For `CAUSE_PATH_TRANSFORM`,
distinguish correct transform behavior that yields insufficient context from
an erroneous transform.

## 13. Root cause

Return a structured root cause, separating:

1. primary tracking failure;
2. safety containment failure or scope gap;
3. physical real-time qualification blocker.

Return `ROOT_CAUSE` and `ROOT_CAUSE_CONFIDENCE`, where confidence is one of
`CONFIRMED`, `SUPPORTED`, `INCONCLUSIVE`, or `REFUTED`. Do not collapse multiple
mechanisms into one vague sentence. Explicitly state whether
`MPPI_OBJECTIVE_PATH_REPRESENTATION_INCOMPATIBILITY` survived attempted
falsification.

## 14. Corrective architecture (proposal only)

Only after completing the causal analysis, propose but do not implement:

1. a tracking correction;
2. a competition track-boundary safety correction;
3. a physical real-time architecture/qualification correction.

Prefer official upstream Nav2 MPPI mechanisms. Do not propose a custom
controller unless the evidence demonstrates an `UPSTREAM_CAPABILITY_GAP`.
For each proposal identify exact upstream mechanism/configuration class,
expected effect, regression risk, smallest controlled validation experiment,
and pass/fail gate. Do not combine independent changes or tune by guessing.

## 15. Next A/B experiment

Write, but do not execute, the smallest experiment that tests the proposed
tracking correction:

- A is the exact authoritative failed configuration.
- B differs by exactly one evidence-justified change.
- Preserve course, raceline, footprint, vehicle geometry, simulator, initial
  state, deterministic seeds, lockstep semantics, safety veto, and three-lap
  acceptance.
- Begin with the smallest replay/short-horizon gate that can falsify the
  mechanism; authorize a full trial only through prior gates.
- Specify telemetry, checkpoints, frozen hashes, stop rules, and objective
  pass/fail criteria before execution.

The work laptop must not run this experiment.

## 16. Required response

Provide a concise executive summary, an evidence table with repository-relative
citations to files/rows or steps, the divergence timeline, critic analysis,
causal matrix, root-cause separation, corrective proposals, and A/B experiment
specification. End with this machine-readable block (use `INCONCLUSIVE` or
`NOT_DERIVABLE_FROM_CAPTURE` rather than guessing):

```text
ARTIFACT_FILES_PRESENT=
ARTIFACT_PROVENANCE=
SOURCE_CONTEXT_VERIFIED=

FIRST_CTE_DIVERGENCE_STEP=
FIRST_HEADING_DIVERGENCE_STEP=
FIRST_CLEARANCE_DEGRADATION_STEP=
FIRST_PERSISTENT_DIVERGENCE_STEP=

PATH_ALIGN_REQUIRED_INDEX=
PATH_ALIGN_MAX_AVAILABLE_INDEX=
PATH_ALIGN_ACTIVE_COUNT=
PATH_ALIGN_INACTIVE_COUNT=
PATH_ALIGN_INACTIVE_PERCENT=
PATH_ALIGN_INACTIVITY_CAUSAL=
PATH_REPRESENTATION_LIMITING_MECHANISM=

DOMINANT_CRITICS_BEFORE_DIVERGENCE=
DOMINANT_CRITICS_DURING_DIVERGENCE=
DOMINANT_CRITICS_NEAR_FAILURE=

EARLIEST_WRONG_SIGN_COMMAND_STEP=
EARLIEST_WRONG_SIGN_COMMAND_EFFECT=
WHY_SELECTED_TRAJECTORY_WON=

LAST_RECOVERABLE_STEP=
FIRST_UNRECOVERABLE_STEP=

FIRST_OFFTRACK_PREDICTABLE_STEP=
FIRST_BOUNDARY_VETO_WOULD_TRIGGER_STEP=
BOUNDARY_PROTECTION_LEAD_STEPS=
BOUNDARY_PROTECTION_LEAD_TIME_S=

SAFETY_VETO_FALSE_NEGATIVE=
SAFETY_VETO_SCOPE_GAP=
OFF_TRACK_BOUNDARY_EQUALS_COSTMAP_LETHAL_BOUNDARY=

HORIZON_GEOMETRICALLY_SUFFICIENT=
SPEED_POLICY_CLASSIFICATION=

PHYSICAL_REALTIME_FEASIBILITY=
PHYSICAL_CONTROLLER_FREQUENCY_QUALIFICATION=

MAX_SIMULATED_STEERING_RATE_RADPS=
STEERING_RATE_GT_0_5_PERCENT=
STEERING_RATE_GT_1_0_PERCENT=
STEERING_RATE_GT_1_5_PERCENT=
STEERING_RATE_GT_2_0_PERCENT=
STEERING_RATE_GT_2_5_PERCENT=
PHYSICAL_STEERING_RATE_QUALIFICATION=PENDING

CAUSE_MPPI_OBJECTIVE=
CAUSE_PATH_REPRESENTATION=
CAUSE_PATH_ALIGN_ACTIVATION=
CAUSE_PATH_PRUNING=
CAUSE_PATH_TRANSFORM=
CAUSE_PATH_ORIENTATION=
CAUSE_TF_SEMANTICS=
CAUSE_ERROR_SIGN=
CAUSE_ACKERMANN_CONVERSION=
CAUSE_STEERING_LIMIT=
CAUSE_SPEED_POLICY=
CAUSE_PREDICTION_HORIZON=
CAUSE_SIMULATOR_DYNAMICS=
CAUSE_SAFETY_VETO_SCOPE=
CAUSE_COSTMAP_OFFTRACK_MISMATCH=
CAUSE_COMPUTE_LATENCY=
CAUSE_LOCKSTEP_SEMANTICS=
CAUSE_RNG=

FIRST_CAUSAL_DIVERGENCE_STEP=
FIRST_CAUSAL_MECHANISM=
FIRST_CAUSAL_CONTROLLER_DECISION=
WHY_THAT_DECISION_WAS_SELECTED=

ROOT_CAUSE=
ROOT_CAUSE_CONFIDENCE=

TRACKING_CORRECTIVE_ARCHITECTURE=
TRACKING_CORRECTIVE_CLASS=
TRACKING_CORRECTIVE_EXPECTED_EFFECT=
TRACKING_CORRECTIVE_REGRESSION_RISK=
TRACKING_CORRECTIVE_VALIDATION_GATE=

BOUNDARY_SAFETY_CORRECTIVE_ARCHITECTURE=
BOUNDARY_SAFETY_CORRECTIVE_CLASS=
BOUNDARY_SAFETY_EXPECTED_EFFECT=
BOUNDARY_SAFETY_REGRESSION_RISK=
BOUNDARY_SAFETY_VALIDATION_GATE=

REALTIME_CORRECTIVE_ARCHITECTURE=
REALTIME_CORRECTIVE_CLASS=
REALTIME_CORRECTIVE_VALIDATION_GATE=

NEXT_AB_EXPERIMENT=

NEW_SIMULATION_RUN=NO
TRIAL_2_RUN=NO
TRIAL_3_RUN=NO
CONTROLLER_PARAMETERS_MODIFIED=NO
PHYSICAL_HARDWARE_TOUCHED=NO
PUBLIC_REPO_WRITE_ATTEMPTED=NO

FIRST_BLOCKER=
NEXT_RECOMMENDED_ACTION=
FINAL_STATUS=
```
