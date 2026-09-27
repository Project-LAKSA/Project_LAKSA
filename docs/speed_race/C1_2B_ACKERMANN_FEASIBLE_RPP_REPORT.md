# C1.2b Ackermann-Feasible Nav2 RPP Report

## 1. Baseline

C1.2b started from clean `competition/speed-race-track` commit
`bced7043d06889d4a6739bca6fba602f6bc7cd67`. The remote branch pointed to
the same commit. Work was performed in an isolated local worktree and in
`/tmp/laksa-c1.2-runtime` on the Jetson. The production workspace, production
services and physical hardware were not touched.

The frozen inputs remained byte-identical:

| Invariant | SHA-256 |
|---|---|
| Canonical course source | `222c897f6835a4877318cfd0ac7d76be98b7173faaeec44e028814ca3c6eb13e` |
| Canonical course geometry | `2c76075f838a7a1c3e0891385b27f2e6f26e641ad13068280093fda273a84858` |
| C1.1 controller raceline | `22ad91de3edbcbdf765f2cf223db53d43be820d829409da356a9def5a4c41783` |
| Waterloo baseline configuration | `083cfca2c03efe3b619bc3a86a88dc6dd1c51f054bd2ab981bc47244c5717083` |
| `ThreeLapGate` | `5255497ba4e81a10988debf9e49ae832cfaea14ff071d113c1c72fb560364ba1` |

## 2. C1.2 failure and C1.2a root cause

C1.2 stopped before its first Gym step because upstream RPP projected its
unconstrained requested curvature through TTC. At the initial state, RPP
requested approximately `-2.1334891826 1/m`. Velocity regulation reduced
linear velocity but preserved that curvature, yielding an equivalent steering
request of approximately `-0.604829645 rad`, outside LAKSA's `+/-0.288 rad`
envelope. The downstream Ackermann clamp happened after TTC and therefore
could not make TTC evaluate the motion the vehicle would actually execute.

C1.2a proved this was a true collision for the impossible Twist, not a TF,
timestamp, footprint or costmap false positive. The same initial state was
collision-free across the TTC horizon when projected at LAKSA's maximum
physical curvature.

## 3. Upstream provenance

The controller remains Nav2 Regulated Pure Pursuit Controller 1.1.20 from
`ros-navigation/navigation2@a097086719c88f781aa59788eca29ac6ca5e56db`.
The external upstream checkout and the ROS system installation were not
modified.

The LAKSA-owned plugin derives from the upstream controller and copies only
the exact upstream `computeVelocityCommands()` orchestration from that pinned
source. It continues to call upstream plan transformation, interpolated and
adaptive lookahead, curvature generation, velocity regulation and TTC. The
single behavioral delta is an Ackermann curvature constraint after upstream
velocity regulation and before upstream `isCollisionImminent()`. The exact
same constrained `(linear velocity, angular velocity)` pair is passed to TTC
and returned as the command.

## 4. Architecture of the correction

For wheelbase `L=0.324 m` and steering limit `delta_max=0.288 rad`:

```text
kappa_max = tan(delta_max) / L
          = 0.9143085878302811 1/m

kappa_cmd = clamp(kappa_req, -kappa_max, +kappa_max)
omega_cmd = v_cmd * kappa_cmd
```

The downstream adapter still computes
`delta=atan(L*omega_cmd/v_cmd)` and retains its `+/-0.288 rad` clamp as
defense in depth. Across every successfully returned runtime command, that
downstream clamp made no material change.

The plugin is explicitly exported as
`laksa_speed_race_nav2::AckermannFeasibleRppController`. The C1 launch selects
that plugin while retaining every existing RPP parameter, the frozen course,
the C1.1 raceline, Gym configuration and `ThreeLapGate`.

## 5. Exact code delta

Added:

- `laksa_speed_race_nav2/ackermann_feasible_rpp_plugin.xml`: explicit plugin
  registration and provenance;
- `laksa_speed_race_nav2/include/laksa_speed_race_nav2/ackermann_feasibility.hpp`:
  pure finite Ackermann feasibility calculation;
- `laksa_speed_race_nav2/include/laksa_speed_race_nav2/ackermann_feasible_rpp_controller.hpp`
  and `src/ackermann_feasible_rpp_controller.cpp`: pinned-upstream derived
  plugin with the one pre-TTC feasibility constraint;
- `laksa_speed_race_nav2/test/test_ackermann_feasibility.cpp`: deterministic
  boundary, zero-velocity, steering-envelope and exact C1.2a regression tests.

Minimally modified:

- the Nav2 package build/manifest and C1 configuration to build and select the
  LAKSA-owned plugin;
- the lockstep host to use the plugin by default and mark its synchronously
  populated TF buffer as externally controlled;
- the Ackermann adapter to capture feasibility telemetry and every upstream
  `lookahead_collision_arc` sample;
- the Gym authority and launch to support qualification-only limits of zero,
  one or a fixed number of steps without changing normal `-1` behavior;
- shutdown handling in the two Python helper nodes;
- focused source-contract and progressive-gate tests.

No tracking law, simulator mathematics, race geometry, controller parameter,
collision threshold or mission criterion was changed.

## 6. Tests and Gate 0

The isolated ROS 2 Humble build covered:

```text
nav2_regulated_pure_pursuit_controller
laksa_speed_race_nav2
laksa_speed_race
pure_pursuit
```

Result: `BUILD=PASS`; `107 tests, 0 errors, 0 failures, 6 skipped`.
The four new C++ feasibility tests passed. The 45 LAKSA Python tests passed.
The suite includes the exact initial C1.2a regression, TTC/returned-command
source ordering, duplicate-stamp rejection and physical-topic isolation.

## 7. Gate 1: initial command, zero Gym steps

Gate 1 passed. The runtime produced:

```text
kappa_req = -2.1334891825932987 1/m
kappa_max =  0.9143085878302811 1/m
kappa_cmd = -0.9143085878302811 1/m
v_cmd     =  0.42855084304633406 m/s
omega_cmd = -0.39182771611917017 rad/s
delta_eq  = -0.288 rad
TTC       = PASS
```

TTC recorded eight projected samples and no collision. The initial drive was
validated, Gym executed zero steps, terminal zero was emitted, all four C1
processes exited cleanly and no orphan remained.

## 8. Gate 2: exactly one Gym step

Gate 2 passed:

```text
controller evaluations = 1
accepted drive requests = 1
Gym steps              = 1
duplicate stamps       = 0
stamp mismatches       = 0
collision/off-track    = 0/0
steps after terminal   = 0
final command          = (steering=0, speed=0)
```

All processes exited cleanly with no orphan. This is the reached-gate runtime
proof of lockstep 1:1 causality.

## 9. Gate 3: short horizon

Gate 3 failed and triggered the mandated stop rule. No Trial 1 or later gate
was run.

The controller returned and Gym accepted exactly 39 stamped commands. Gym
advanced exactly 39 steps to simulation time `0.39 s`. Across those commands:

- `abs(kappa_cmd) <= 0.9143085878302811 1/m`;
- `abs(delta_applied) <= 0.288 rad`;
- downstream steering saturation count was zero;
- TTC evaluated the same constrained command that was returned;
- duplicate-stamp and stamp-mismatch counts were zero;
- Gym collision, off-track, reverse and invalid-command counts were zero.

The 40th controller evaluation did not return a command. Upstream TTC rejected
the physically constrained trajectory with:

```text
nav2_rpp_exception:AckermannFeasibleRppController detected collision ahead!
```

The failed state was the post-step-39 pose:

```text
x=20.720827102661133 m
y=2.2949748039245605 m
yaw=-0.08238866925239563 rad
```

The projected command remained at the physical negative-curvature limit. A
read-only replay of the recorded TTC arc against the frozen 5 cm Nav2 map and
the unchanged footprint identified the first lethal outline contact at TTC
sample index 8 (zero based):

```text
projected pose ~= (21.15207020587118, 2.177827009878199, -0.493827533776)
costmap cell   = (429, 36)
cost           = LETHAL_OBSTACLE (254)
```

This is a new first blocker. The C1.2a integration mismatch is fixed: TTC no
longer evaluates impossible curvature. However, after 0.39 s of closed-loop
evolution, the physically feasible maximum-curvature TTC arc itself reaches a
lethal map cell within the unchanged one-second horizon. Per the task's stop
rule, no parameter tuning or further runtime attempt was made.

Gate 3 still terminated safely: terminal zero was persisted, steps after
terminal were zero, all processes exited cleanly and no orphan remained.

## 10. Gates not authorized after the stop

- Gate 4 / full Nav2 Trial 1: `NOT_RUN`
- Gate 5 / x3 determinism: `NOT_RUN`
- Gate 6 / final three-lap qualification: `NOT_RUN`

No lap was attempted or claimed.

## 11. Telemetry artifacts

Private machine/runtime evidence remains under:

```text
/tmp/laksa-c1.2b-results/gate1_zero_step
/tmp/laksa-c1.2b-results/gate2_one_step
/tmp/laksa-c1.2b-results/gate3_short_horizon
```

Each reached gate contains its `summary.json`, commands, trajectory, events,
controller telemetry, TTC samples and runtime log. Raw machine-specific data
is not committed to the public repository.

## 12. Safety and final verdict

Physical-topic isolation remained green. Only `/c1/*` state and command
interfaces participated. Production was not modified, physical hardware was
not touched, and all reached gates ended with a zero command and no orphan
processes.

`C1_2B_STATUS=FAIL` because Gate 3 failed. The implementation correctly fixes
the demonstrated impossible-curvature TTC mismatch, but it is not a qualified
three-lap controller baseline.

The next action is a separate, explicitly authorized research task on the
step-40 physically feasible TTC collision. It should analyze the frozen-path
progression and upstream RPP command evolution at that state before deciding
whether any controller parameter change is justified. It must not modify the
course, C1.1 raceline, vehicle geometry or safety gates.
