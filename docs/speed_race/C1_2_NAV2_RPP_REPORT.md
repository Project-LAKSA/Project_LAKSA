# C1.2 Nav2 Regulated Pure Pursuit Report

## Starting state

C1.2 started from clean `competition/speed-race-track` commit
`5d8f63e59d67ebc4dbcf7253251d6e90ed031467`. The remote branch pointed to the
same commit. Work was performed in an isolated worktree and in
`/tmp/laksa-c1.2-runtime` on the Jetson, using `ROS_DOMAIN_ID=222` and
`ROS_LOCALHOST_ONLY=1`.

The experiment reached its strict Trial 1 stop rule. The implementation and
instrumentation are retained, but C1.2 is **not qualified**.

## Frozen invariants

The following inputs were verified before implementation and were not changed:

| Invariant | SHA-256 |
|---|---|
| Canonical course source | `222c897f6835a4877318cfd0ac7d76be98b7173faaeec44e028814ca3c6eb13e` |
| Canonical course geometry | `2c76075f838a7a1c3e0891385b27f2e6f26e641ad13068280093fda273a84858` |
| C1.1 full raceline (`speed_course_raceline.csv`) | `1086933179481201163c4ce30d566aca6a4b6377bd723632583f3eb935e043c4` |
| C1.1 controller raceline (`pure_pursuit_raceline.csv`) | `22ad91de3edbcbdf765f2cf223db53d43be820d829409da356a9def5a4c41783` |
| Waterloo configuration | `083cfca2c03efe3b619bc3a86a88dc6dd1c51f054bd2ab981bc47244c5717083` |
| `ThreeLapGate` implementation | `5255497ba4e81a10988debf9e49ae832cfaea14ff071d113c1c72fb560364ba1` |

C1.1 full-body clearance remains 0.024078635053796427 m against the frozen
0.023813685623730953 m requirement. Course geometry, both raceline files,
vehicle dimensions, steering limit, Gym dynamics and `ThreeLapGate` remain
byte-identical.

## Nav2 RPP provenance

The external upstream was cloned at exact commit
`a097086719c88f781aa59788eca29ac6ca5e56db`. Its
`nav2_regulated_pure_pursuit_controller/package.xml` declares version 1.1.20.
Only that upstream package was built over the ROS 2 Humble underlay; its source
was not modified or vendored.

Source review confirmed that interpolated lookahead, velocity-scaled lookahead,
Pure Pursuit curvature, curvature-dependent velocity regulation, collision/TTC
projection and plan transformation remain upstream Nav2 behavior. The LAKSA
code contains no replacement tracking or simulator mathematics.

## Implementation architecture

The C1.2 additions are:

- a finite Nav2 path publisher over the unchanged C1.1 raceline;
- an external-plugin lockstep host for exact Nav2 RPP 1.1.20;
- a mechanical `TwistStamped` to `AckermannDriveStamped` adapter;
- a C1.2 launch using only `/c1/*` command/state interfaces;
- stamped command admission in the existing Gym authority;
- deterministic contract and isolation tests.

The Waterloo launch and configuration remain as the frozen baseline.

## Lockstep causality design

The host has no controller timer. An accepted, positive, monotonically
increasing odometry stamp is the only call site for
`computeVelocityCommands()`. The resulting `TwistStamped` preserves that stamp;
the Ackermann request preserves it again; and `StateStampGate` allows at most
one matching request to reach Gym. Duplicate stamps are ignored and mismatched
stamps fault without stepping.

Unit and source-contract tests pass. Trial 1 faulted before an upstream command
was returned, so end-to-end 1:1 runtime causality remains unproven rather than
being inferred from a zero-step run.

## Ring-unrolling design

The controller CSV is headerless and contains 547 rows, where row 547 exactly
duplicates row 1. Runtime construction removes only that duplicate for each
copy and produces `rows[0:546] * 4 + rows[0]`, exactly 2185 poses. The source
CSV bytes remain unchanged. A bounded monotonic index is telemetry-only; Nav2's
upstream `max_robot_pose_search_dist=2.0` controls plan pruning.

## Ackermann adapter proof

The adapter performs only:

`delta_raw = atan(0.324 * omega / v)`

and clamps the result to `[-0.288, +0.288]` rad. Reverse, non-finite input and
rotate-in-place commands fail closed. Analytical sign, zero-speed and exact
saturation cases pass. No gain, filter, PID or tracking intelligence was added.

## Configuration

The initial audited configuration was used unchanged: desired speed 1.0 m/s,
lookahead 0.5 m, adaptive range 0.5-1.0 m, lookahead time 1.0 s, interpolation
enabled, curvature regulation enabled with minimum radius
1.093722637313 m/minimum speed 0.25 m/s, TTC collision detection enabled at
1.0 s, reversing and rotate-to-heading disabled, and maximum robot pose search
distance 2.0 m. Its SHA-256 is
`1da091b2bbbec5cb93c8c0e71e7f47ba16db4e51c48a0d76e96cfe97ef357d93`.

No parameter tuning was performed.

## Tests

The isolated Jetson build completed for the external RPP package,
`laksa_speed_race_nav2`, `laksa_speed_race`, and the frozen Waterloo package.
`colcon test-result --verbose` reported 99 tests, 0 errors, 0 failures and 6
skips. The LAKSA Python suite reported 42 passing tests. The external RPP tests,
LAKSA C0/C1 tests, frozen-raceline/hash checks, ring-unrolling, Ackermann,
stamp-admission, `ThreeLapGate` identity and physical-topic isolation all
passed.

## Waterloo baseline reproduction

The frozen baseline reproduced exactly in the resulting environment:

- 82 simulator steps;
- terminal pose `(21.21421241760254, 2.146467924118042, -0.42329955101013184)`;
- collision 1, off-track 1, laps 0, reverse 0, invalid commands 0;
- CTE RMS 0.22857311357571794 m, p95 0.26671433297337627 m, max
  0.2667165038340429 m;
- heading RMS 0.2585738568252443 rad;
- maximum absolute steering 0.2879999876022339 rad;
- terminal zero observed and zero post-terminal steps.

This establishes that C1.2 did not alter the baseline experiment.

## Nav2 Trial 1

Exactly one Nav2 Trial 1 was launched. The 2185-pose path and frozen hashes
validated, and the exact RPP plugin loaded. On the initial state, upstream
`isCollisionImminent()` threw:

`RegulatedPurePursuitController detected collision ahead!`

The lockstep host converted that exception into a controller fault. Gym
executed zero steps. No real Gym collision, off-track, reverse or invalid
command event occurred. The mission ended in `FAULT`, with final applied
command `(0.0 m/s, 0.0 rad)` and zero post-terminal steps. No CTE or heading
statistics exist because the simulator did not advance.

A read-only raster replay of Nav2's footprint-edge collision semantics found:

- the initial footprint is collision-free;
- projected samples through approximately 0.700 s are collision-free;
- the projected footprint first intersects lethal 0.05 m raster cells at
  approximately 0.817 s.

The replay used the audited initial regulated command approximation
`v=0.428551 m/s`, `omega=-0.91431 rad/s` only to localize the upstream fault;
those values were not hardcoded into C1.2. This evidence classifies the first
blocker as an initial Nav2 RPP TTC projection collision on the canonical 5 cm
costmap, not as a Gym collision or a C1.1 static-clearance failure.

The host also emitted TF2 diagnostics because the manually populated buffer
was not marked as using a dedicated thread. The plugin nevertheless reached
its collision check. This integration defect is recorded but was not changed
after the Trial 1 stop rule.

## Determinism

Not run. Trial 1 failed, so the required three identical successful Trial 1
executions were prohibited.

## Three-lap qualification

Not run. Trial 1 did not pass and therefore Trials 2 and 3 and the full
three-lap gate were prohibited. `EXACT_THREE_LAPS=FAIL` for the attempted run;
this is not a statement about the unchanged `ThreeLapGate` unit contract.

## A/B comparison

| Controller | Status | Failure time | Laps | Collision | Off-track | Reverse | Invalid | CTE RMS / p95 / max (m) | Heading RMS (rad) | Max steering (rad) | Saturation | Min/avg speed | Lap time |
|---|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---|---|---|
| Waterloo Pure Pursuit | Known baseline failure reproduced | 0.82 s | 0 | 1 | 1 | 0 | 0 | 0.228573 / 0.266714 / 0.266717 | 0.258574 | 0.288000 | Existing summary reports 0 | 1.0 / 1.0 m/s | none |
| Nav2 RPP 1.1.20 | Trial 1 failed before step 1 | 0.00 s | 0 | 0 actual; predictive TTC fault | 0 | 0 | 0 | unavailable | unavailable | 0.0 applied | 0 | unavailable | none |

These results do not establish that either controller is better. They identify
different first failures under the same frozen experiment inputs.

## Safety / isolation

Execution used ROS domain 222 with localhost-only discovery and an isolated
`/tmp` workspace. The C1.2 graph contains no `/drive`, `/cmd_vel`,
`/laksa/command`, `/laksa/set_drive_command`, micro-ROS, VESC or GPIO authority.
No physical hardware, production workspace or production service was touched.
After the run, no C1 ROS nodes or processes remained.

The C++ host and Gym authority exited cleanly, but the path and Ackermann Python
nodes were interrupted by launch shutdown and reported exit code -2. Therefore
the strict clean-shutdown gate is reported as failed, despite zero orphan
processes.

## Git state

Only C1.2 simulation integration, tests, provenance and this report are part of
the change. Raw Jetson logs and run artifacts remain private under
`/tmp/laksa-c1.2-results` and are not committed.

Files added:

- `laksa_speed_race_nav2/CMakeLists.txt`, `package.xml`, and
  `src/rpp_lockstep_host.cpp`: package and deterministic host for the external
  Nav2 plugin;
- `config/c1_nav2_rpp.yaml`: frozen initial RPP parameters;
- `laksa_speed_race/nav2_raceline_node.py`: hash-checked mechanical ring
  unrolling;
- `laksa_speed_race/nav2_ackermann_adapter_node.py`: stamped mechanical
  Ackermann conversion and compact controller telemetry;
- `launch/c1_nav2_three_lap.launch.py`: isolated C1.2 graph;
- `test/test_c1_nav2_rpp.py`: frozen-input, ring, Ackermann, causality and gate
  regression contracts;
- this report.

Files modified:

- `gym_adapter_node.py`: optional C1.2 stamp admission, controller provenance,
  controller-fault evidence and terminal zero without a Gym step;
- `setup.py` and `package.xml`: install and declare the C1.2 executables;
- `test_c1_isolation.py`: include the C1.2 graph in physical-topic isolation;
- `test_three_lap_runtime.py`: preserve the existing metadata assertion after
  controller provenance became selectable;
- `test_c0_provenance.py`: require the new exact Navigation2 pin;
- `UPSTREAM_PROVENANCE.md` and `speed_race_upstream.repos`: record Nav2 1.1.20
  at its audited commit.

The isolated build/test command selected the four relevant packages:

```text
colcon build --symlink-install --packages-select \
  nav2_regulated_pure_pursuit_controller laksa_speed_race_nav2 \
  laksa_speed_race pure_pursuit
colcon test --packages-select \
  nav2_regulated_pure_pursuit_controller laksa_speed_race_nav2 \
  laksa_speed_race pure_pursuit
colcon test-result --verbose
```

The frozen Waterloo baseline used `c1_three_lap.launch.py`; the single C1.2
attempt used `c1_nav2_three_lap.launch.py` with `max_laps:=3` and output path
`/tmp/laksa-c1.2-results/nav2_trial_1`. No second C1.2 launch occurred.

The final review found no credentials, private keys, proprietary material,
generated build/install/log files, production changes or physical command
interfaces in the commit. Nav2 remains an external Apache-2.0 dependency;
existing MIT/LGPL/GPL provenance boundaries are unchanged.

## Remaining blocker

`FIRST_BLOCKER=NAV2_RPP_INITIAL_TTC_COLLISION_ON_CANONICAL_5CM_COSTMAP`

The next task must research the observed TTC projection and the lockstep TF
buffer integration before authorizing any parameter change or rerun. It must
not modify the C1.1 raceline, course, steering limit, controller mathematics or
Gym collision semantics. C1.2 remains unqualified.
