# C1.2d Nav2 MPPI Ackermann Qualification Report

## Executive result

C1.2d passed Gate 0 completely after two narrowly scoped test-infrastructure
dispositions. The original ARM64 numerical failure was reproduced exactly and
disposition B was validated. The subsequently misnamed pytest helper was
renamed and collection/build/all tests passed. The first historical S0 replay
returned a physically feasible command but correctly stopped on a harness
shutdown failure that left four isolated C1 processes alive. A dedicated,
bounded process-session owner now terminates the complete replay tree without
matching process names. S0, S32, and S39 subsequently returned physically
feasible commands, passed the independent veto, shut down gracefully, and
left zero C1 processes. Gym was not run. No course, raceline, costmap,
footprint, ThreeLapGate, production workspace, service, or physical hardware
was changed.

The failing upstream assertion is
`CriticTests.PathAlignCritic` in
`nav2_mppi_controller/test/critics_tests.cpp:596`. On the Jetson it computes
`6599.984375`; the test expects `6600.0 +/- 0.01`. The absolute difference is
`0.015625`. The same failure is represented in both CTest and gtest result
XML, so the aggregate report lists it twice; it is one test case. No upstream
source or test was modified and the tolerance was not weakened.

## Starting state and frozen invariants

- Branch: `competition/speed-race-track`
- Starting HEAD: `21588774eef8023811e751019b13abc9a43b692e`
- Starting remote HEAD: `21588774eef8023811e751019b13abc9a43b692e`
- Worktree: clean before implementation
- C1.1 raceline SHA256:
  `22ad91de3edbcbdf765f2cf223db53d43be820d829409da356a9def5a4c41783`
- Canonical source SHA256:
  `222c897f6835a4877318cfd0ac7d76be98b7173faaeec44e028814ca3c6eb13e`
- Canonical geometry SHA256:
  `2c76075f838a7a1c3e0891385b27f2e6f26e641ad13068280093fda273a84858`
- ThreeLapGate SHA256:
  `5255497ba4e81a10988debf9e49ae832cfaea14ff071d113c1c72fb560364ba1`

The frozen raceline and ThreeLapGate hashes remained byte-identical. The
course, costmap assets, footprint, Gym configuration, and Waterloo baseline
were not edited.

## Upstream provenance

The exact Humble-compatible source is
`ros-navigation/navigation2@a097086719c88f781aa59788eca29ac6ca5e56db`, tag
and package version `1.1.20`, Apache-2.0. It was built unchanged from an
isolated source copy at
`/tmp/laksa-c1.2-runtime/src/navigation2/nav2_mppi_controller` on the Jetson.

Relevant upstream implementation points are:

- `nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp`,
  `AckermannMotionModel::applyConstraints()`: constrains sampled `wz` using
  `AckermannConstraints.min_turning_r`.
- `nav2_mppi_controller/src/optimizer.cpp`: optimizes and returns the command
  from the constrained MPPI control sequence.
- `nav2_mppi_controller/src/critics/cost_critic.cpp`, `CostCritic`: scores
  trajectory samples and uses `FootprintCollisionChecker` when
  `consider_footprint=true` and the upstream cost-screening conditions apply.
- `nav2_mppi_controller/src/noise_generator.cpp`, `NoiseGenerator`: with
  `regenerate_noises=false`, generates a fixed noise matrix at initialization
  and reuses it.

No capability backport, controller fork, or optimizer modification was made.

## Architecture implemented

The existing timer-free lockstep host now loads either controller plugin by
configuration. C1.2d selects upstream
`nav2_mppi_controller::MPPIController`. Each accepted stamped odometry state
causes at most one `computeVelocityCommands()` call. Before publication, the
returned MPPI Twist is checked against LAKSA's Ackermann envelope and passed
through an independent full-footprint predictive veto. Only a validated
command reaches the existing mechanical Twist-to-Ackermann adapter and Gym
step authority.

The implemented command chain is:

```
stamped state -> upstream MPPI Ackermann model -> returned Twist
              -> LAKSA feasibility validator
              -> independent full-footprint veto
              -> mechanical Ackermann conversion
              -> existing stamped Gym authority
```

The LAKSA guard does not alter a command. It rejects non-finite, reverse,
rotation-in-place, excessive-curvature, or excessive-steering commands. Thus a
downstream clamp cannot hide optimizer infeasibility.

## Ackermann constraints and command conversion

- Wheelbase: `0.324 m`
- Steering limit: `+/-0.288 rad`
- Minimum turning radius: `1.0937226373133722 m`
- Maximum curvature: `tan(0.288) / 0.324 = 0.9143085878302811 1/m`
- MPPI motion model: `Ackermann`
- `AckermannConstraints.min_turning_r`: `1.0937226373133722`
- `vx_min`: `0.0 m/s`
- `vx_max`: `1.0 m/s`
- `wz_max`: `0.9143085878302811 rad/s`

For nonzero `v`, the adapter reconstructs
`delta = atan(0.324 * wz / v)`. A defensive `+/-0.288 rad` clamp remains, but
normal MPPI commands must pass the upstream-envelope validator before the
adapter. Any material defensive-clamp activation would be a qualification
failure.

## Independent safety layer

The independent veto evaluates the exact returned, physically feasible
`(v, wz)` command with the canonical asymmetric footprint
`[[-0.149,-0.148],[0.419,-0.148],[0.419,0.148],[-0.149,0.148]]` against the
frozen costmap through Nav2's upstream `FootprintCollisionChecker`. It projects
for `1.0 s` at spatial increments equal to the frozen map resolution. It does
not modify MPPI output or replace upstream critics.

The independent veto passed at historical S0, S32, and S39. Closed-loop Gym
effectiveness remains **unqualified** because this task intentionally stopped
after the replay gates.

## Parameters

The committed C1.2d configuration uses upstream MPPI example/default values
where no LAKSA-specific value exists, with frozen project constraints applied:

- `controller_frequency=100 Hz`, `model_dt=0.01 s`: matches lockstep Gym dt.
- `time_steps=100`: one-second predictive horizon at the frozen dt.
- `batch_size=2000`, `iteration_count=1`, `temperature=0.3`, `gamma=0.015`:
  upstream Humble MPPI baseline values, not tuned from C1 outcomes.
- `motion_model=Ackermann`, `min_turning_r=1.0937226373133722`: frozen LAKSA
  geometry.
- `regenerate_noises=false`: fixed sampling matrix for repeatability.
- `CostCritic.consider_footprint=true`: requests upstream footprint-aware
  collision scoring.
- Path, goal, constraint, cost, and prefer-forward critics use upstream
  baseline weights. No parameter sweep was performed.

## Steering-rate characterization

An exhaustive repository search found no authoritative LAKSA steering slew or
rate value. No value was invented. C1.2d therefore implements and tests only
the authoritative steering-angle and minimum-radius limits, and records
`STEERING_RATE_CHARACTERIZATION_REQUIRED=YES`.

## Build and tests

Jetson isolated workspace: `/tmp/laksa-c1.2-runtime`.

Build command (abridged only for line wrapping):

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/tmp/laksa-c1-native/ackermann_root/opt/ros/humble:$CMAKE_PREFIX_PATH
colcon build --symlink-install \
  --packages-select nav2_regulated_pure_pursuit_controller \
  nav2_mppi_controller laksa_speed_race_nav2 laksa_speed_race pure_pursuit
```

Build result: **PASS**, five packages built.

Project-owned tests:

- `laksa_speed_race_nav2`: **5/5 PASS**
- `laksa_speed_race`: **47/47 PASS**
- Total project-owned: **52/52 PASS**

The regression coverage includes frozen-raceline and ThreeLapGate hashes,
Ackermann model/configuration, exact physical-envelope validation, no reverse,
zero-speed handling, command validation before publication, independent-veto
ordering, duplicate-stamp rejection, and physical-topic isolation.

Original pinned upstream MPPI suite before disposition B:

- One distinct failure:
  `CriticTests.PathAlignCritic` (`critics_tests.cpp:596`)
- ARM64 actual: `6599.984375`
- Expected: `6600.0`
- Absolute difference: `0.015625`
- Upstream tolerance: `0.01`

Gate 0 originally stopped correctly on this result. The historical test was
neither skipped nor weakened; its semantic replacement and reruns are recorded
below.

## Gate 0 ARM64 Numerical Disposition

The original unchanged upstream assertion was reproduced once on the actual
Jetson environment before applying the disposition:

- Architecture: `aarch64`
- Kernel: `Linux 5.15.148-tegra`
- Compiler: GCC/G++ `11.4.0`
- Test: `CriticTests.PathAlignCritic`
- Source: `nav2_mppi_controller/test/critics_tests.cpp:596`
- Assertion: `EXPECT_NEAR(xt::sum(costs, immediate)(), 6600.0, 1e-2)`
- Actual: `6599.984375`
- Expected: `6600.0`
- Absolute error: `0.015625`

This exactly matched the independently researched float32 AArch64 reduction
case, so disposition B was applied. The source-controlled patch helper replaces
only the aggregate assertion with:

```cpp
for (const auto cost : costs) {
  EXPECT_FLOAT_EQ(cost, 6.6f);
}
```

`costs` is an `xt::xtensor<float, 1>` containing 1000 trajectory costs. The
replacement therefore validates every trajectory's intended semantic result
without depending on aggregate reduction order and without widening a
tolerance.

The affected test was rebuilt and run four times:

- Targeted test: **PASS**
- Repeat 1: **PASS**
- Repeat 2: **PASS**
- Repeat 3: **PASS**

Runtime files were hashed before and after the disposition and remained
identical:

- `src/critics/path_align_critic.cpp`:
  `1bb0cd21617e04a065383ce2eedc3bfdf91cfc0690df9b6c74042bfc01f293a6`
- `include/nav2_mppi_controller/critics/path_align_critic.hpp`:
  `ce407dcaad9eb37b7179b8eb0c71953ddefe001460266d44a1de939f8898ad93`
- `src/optimizer.cpp`:
  `50489455e643cdc5b79a219c8a1886e1a5320a838cea145df9b7fc389a6ad8db`
- `src/controller.cpp`:
  `9b5524ad0437c914592484f5b7cd306233d8389ced22cd97d2f6ad92197c9aaf`

The complete Gate-0 restart produced:

- Build: **PASS**
- Nav2 MPPI upstream tests: **PASS**, 17/17 CTest targets, including 23/23
  cases in `critics_tests`
- LAKSA regression suite: **FAIL during collection**
- New first blocker: pytest collected
  `docker/patch_nav2_mppi_path_align_test.py` because the helper filename ends
  in `_test.py`; the module's top-level `sys.argv[1]` access raised
  `IndexError`

At that prior stop point, the helper was intentionally left unrenamed and Gate
0 was not rerun. The follow-up collection disposition and full restart are
recorded below; this preserves rather than erases the original stop evidence.

Diff classification:

- `TEST_ONLY_DIFF`: reproducible helper that changes the one upstream
  PathAlignCritic assertion to per-trajectory `EXPECT_FLOAT_EQ` checks, plus a
  LAKSA regression asserting that semantic form.
- `C1_2D_IMPLEMENTATION_DIFF`: the pre-existing dirty MPPI integration listed
  below.
- `RUNTIME_UPSTREAM_DIFF`: none from the Gate-0 disposition.
- `PATH_ALIGN_RUNTIME_CODE_MODIFIED=NO`
- `MPPI_RUNTIME_CODE_MODIFIED_BY_G0_FIX=NO`

## Gate 0 pytest collection disposition

The accidentally collected helper was:

```text
firmware/esp32-s3/jetson/laksa_speed_race/docker/
  patch_nav2_mppi_path_align_test.py
```

The filename matched pytest's `*_test.py` discovery pattern. Its only
legitimate invocation was the Gate-0 infrastructure command that patches the
pinned external Nav2 test source; there were no Dockerfile, CMake, or runtime
references. The LAKSA regression test and this report were the only repository
references.

It was renamed to:

```text
firmware/esp32-s3/jetson/laksa_speed_race/docker/
  patch_nav2_mppi_path_align.py
```

The regression reference was updated. The helper's patch operation is
unchanged and is now wrapped in an `argparse` CLI with a `main` guard, so direct
misuse returns a clear usage error rather than an import-time `IndexError`.
Pytest discovery configuration was not changed.

Collection-only validation on the ARM64 Gate-0 host found **48 legitimate
Python tests**, no helper module, no missing tests, and no collection error.
The renamed helper was then exercised idempotently against the pinned external
test source and `CriticTests.PathAlignCritic` passed again. Runtime MPPI hashes
remained unchanged.

The complete Gate-0 restart produced:

- Build: **PASS**, five packages
- Nav2 MPPI upstream: **17/17 PASS**
- LAKSA C++ Ackermann tests: **5/5 PASS**
- LAKSA Python regression tests: **48/48 PASS**
- Aggregate colcon results: **401 tests, 0 errors, 0 failures, 71 upstream
  skips**
- Physical-topic isolation: **PASS**
- Duplicate-stamp rejection: **PASS**
- Static command-feasibility ordering: **PASS**
- Static independent-safety-veto ordering: **PASS**

`C1_2D_GATE_0=PASS`.

## Progressive qualification results

The original mandatory stop at S0 is preserved below. After the surgical
lifecycle correction and a complete green Gate-0 restart, all three authorized
historical replays passed. The task did not authorize Gym.

- Gate 0 static/unit: **PASS**
- Historical S0 replay: **PASS**
- Historical S32 replay: **PASS**
- Historical S39 replay: **PASS**
- Start-state recovery: **NOT RUN**
- Lockstep Gym runtime: **NOT RUN**
- Short horizon: **NOT RUN**
- X3 determinism: **NOT RUN**
- Trial 1: **NOT RUN**
- Trials 2/3: **NOT RUN**

The isolated S0 controller replay returned:

```text
vx                 = 0.016281509771943092 m/s
wz                 = 0.0044019222259521484 rad/s
equivalent kappa   = 0.27036327021328854 1/m
equivalent steering= 0.08737466934699004 rad
physical envelope  = PASS
independent veto   = PASS (no controller fault; command published)
```

However, the harness failed to terminate the path publisher and lockstep host.
The captured orphan process IDs were `3208022`, `3208023`, `3208025`, and
`3208026`. After evidence capture, only those isolated C1 processes were
terminated; a follow-up process scan found zero remaining C1 processes.

### Historical S0 replay shutdown disposition

The failure was reproduced once before changing the harness. The exact process
tree was:

```text
PID 3239536 bash /tmp/c1d_replay_state.sh ...
  PID 3239539 /usr/bin/python3 /opt/ros/humble/bin/ros2 run
    laksa_speed_race c1_nav2_raceline
    PID 3239542 .../c1_nav2_raceline
  PID 3239540 /usr/bin/python3 /opt/ros/humble/bin/ros2 run
    laksa_speed_race_nav2 rpp_lockstep_host ...
    PID 3239543 .../rpp_lockstep_host ...
```

The harness had `PPID=3239506`, while the harness, wrappers, and executables
all inherited `PGID=3239506` and `SID=3239506` from the SSH shell. The cleanup
sent `SIGINT` only to wrapper PIDs `3239539` and `3239540`. It did not signal
the executable grandchildren `3239542` and `3239543`, then blocked in an
unbounded shell `wait`. This is why the observed survivors were exactly the
two `ros2 run` wrappers plus their two child executables. There was no detached
or double-forked process; ownership and signal scope were wrong.

The source-controlled replacement is
`laksa_speed_race.owned_process_session.OwnedProcessSession`. It launches one
replay worker with `start_new_session=True`, making the worker PID the unique
PGID/SID for every replay descendant. Shutdown is bounded and scoped only to
that PGID:

1. send `SIGINT` and wait 3 seconds;
2. if required, send `SIGTERM` and wait 2 seconds;
3. if required, send `SIGKILL` and wait 2 seconds;
4. reap the session leader and Linux subreaper-owned descendants;
5. fail unless the owned PGID has no remaining process.

No `pkill`, `killall`, process-name match, or global ROS shutdown is used.
Humble topic observers use the verb-level `--no-daemon` option so the replay
does not share lifecycle with a persistent ROS CLI daemon. Child exit status,
missing observations, and controller faults now propagate as replay failures.

Eight deterministic lifecycle tests cover normal cleanup, controller
exception, timeout, TERM fallback, KILL fallback, descendant reaping,
unrelated-process preservation, and immediate repeated invocation. All eight
passed on the ARM64 Jetson. The complete LAKSA Python suite increased from 48
to **56/56 PASS**. Gate 0 remained green: upstream MPPI **17/17**, LAKSA C++
**5/5**, aggregate colcon **409 tests, 0 errors, 0 failures, 71 upstream
skips**.

An unrelated `sleep` sentinel (`PID=3277839`, `PGID=3277829`) was present
before the final S0 replay and remained alive afterward. It was then
terminated explicitly by PID. This proves the replay cleanup did not affect
the unrelated process.

### Replay requalification evidence

Each replay used a distinct isolated ROS domain and one unique owned process
group. Each ended in the graceful `SIGINT` phase with zero remaining PIDs and
recorded safe terminal semantics `(steering=0.0, speed=0.0)`. No Gym command
authority was present.

| State | `v` (m/s) | `wz` (rad/s) | `kappa` (1/m) | steering (rad) | feasibility | safety veto | remaining PIDs |
|---|---:|---:|---:|---:|---|---|---:|
| S0 | 0.016281509771943092 | 0.0044019222259521484 | 0.27036327021328854 | 0.08737466934699004 | PASS | PASS | 0 |
| S32 | 0.014251111075282097 | 0.0040302770212292671 | 0.28280440731527234 | 0.09137348000065269 | PASS | PASS | 0 |
| S39 | 0.014268609695136547 | 0.0040302220731973648 | 0.2824537330060311 | 0.09126080633807891 | PASS | PASS | 0 |

At S39, MPPI returned a physically feasible safe command without curvature
saturation or an external feasibility clamp. The fault topic remained empty
for all three replays.

No Gym steps were authorized. Therefore there are no CTE, clearance,
collision, lap, lockstep-Gym, or ThreeLapGate qualification results to report.

## Files changed

- `firmware/esp32-s3/jetson/laksa_speed_race/config/c1_nav2_mppi.yaml`:
  frozen MPPI Ackermann configuration.
- `firmware/esp32-s3/jetson/laksa_speed_race/launch/c1_nav2_mppi_three_lap.launch.py`:
  simulation-only C1.2d launch.
- `firmware/esp32-s3/jetson/laksa_speed_race/laksa_speed_race/nav2_ackermann_adapter_node.py`:
  receives generic controller feasibility telemetry.
- `firmware/esp32-s3/jetson/laksa_speed_race/laksa_speed_race/owned_process_session.py`:
  owns and bounds one complete replay process session.
- `firmware/esp32-s3/jetson/laksa_speed_race/laksa_speed_race/historical_replay.py`:
  source-controlled S0/S32/S39 replay worker and controller.
- `firmware/esp32-s3/jetson/laksa_speed_race/test/test_owned_process_session.py`:
  eight harmless lifecycle and isolation regressions.
- `firmware/esp32-s3/jetson/laksa_speed_race/setup.py`:
  installs the historical replay entry point.
- `firmware/esp32-s3/jetson/laksa_speed_race/test/test_c1_nav2_rpp.py`:
  MPPI configuration, feasibility, and ordering regressions.
- `firmware/esp32-s3/jetson/laksa_speed_race/test/test_c1_isolation.py`:
  includes the MPPI launch/config in physical-isolation scanning.
- `firmware/esp32-s3/jetson/laksa_speed_race/UPSTREAM_PROVENANCE.md`:
  records the unchanged MPPI use of the existing Nav2 pin.
- `firmware/esp32-s3/jetson/laksa_speed_race/docker/patch_nav2_mppi_path_align.py`:
  reproducible test-only disposition for the architecture-sensitive aggregate
  assertion.
- `firmware/esp32-s3/jetson/laksa_speed_race_nav2/CMakeLists.txt` and
  `package.xml`: expose the local feasibility header and declare the runtime
  MPPI dependency.
- `firmware/esp32-s3/jetson/laksa_speed_race_nav2/include/laksa_speed_race_nav2/ackermann_feasibility.hpp`:
  non-mutating Twist feasibility validator.
- `firmware/esp32-s3/jetson/laksa_speed_race_nav2/src/rpp_lockstep_host.cpp`:
  configurable upstream plugin loading, feasibility validation, telemetry, and
  independent footprint veto.
- `firmware/esp32-s3/jetson/laksa_speed_race_nav2/test/test_ackermann_feasibility.cpp`:
  direct validator regression.
- `docs/speed_race/C1_2D_NAV2_MPPI_ACKERMANN_REPORT.md`: this evidence report.

## Safety and isolation

Static physical-topic isolation tests pass. The C1.2d launch and replay contain
only `/c1/*` interfaces, use the isolated workspace, and were not allowed to
step Gym. Production branches, workspaces, services, devices, GPIO, VESC,
ESP32, and physical hardware were not touched.

## First blocker and next action

`FIRST_BLOCKER=NONE_WITHIN_HISTORICAL_REPLAY_SCOPE`.

The next action is a separately authorized C1.2d lockstep Gym short-horizon
qualification. It must retain the now-proven replay process ownership and stop
at the first runtime gate failure. Physical steering-rate characterization
remains required before making actuator-dynamics claims.

## Closed-loop Gym qualification

### Gate A — pre-flight

The closed-loop qualification began from clean local and remote
`competition/speed-race-track` commit
`e9ff67d7ccfb37969dc21604ff58fca13c0d41a2`. The isolated Jetson workspace
contained no stale C1 processes before launch. Frozen artifacts remained
unchanged:

- C1.1 raceline SHA256:
  `22ad91de3edbcbdf765f2cf223db53d43be820d829409da356a9def5a4c41783`
- canonical manifest SHA256:
  `b0e2e709fb2cb664600353ffb6145eff08f069e688e847c9ffc390eae8e26ead`
- MPPI configuration SHA256:
  `87d9b54df6bda3a93b30bbd6c28e5b16bbf1e43c612ba08d0854595f38bf4692`
- MPPI launch SHA256:
  `ce0e9b89b91a957a21d0691026fca8951f32dcaab2f57de2eaa79c6ee80089a2`
- ThreeLapGate SHA256:
  `5255497ba4e81a10988debf9e49ae832cfaea14ff071d113c1c72fb560364ba1`

The minimum relevant committed smoke checks passed in the isolated ARM64
environment: Ackermann feasibility C++ tests **5/5 PASS** and Python
regressions **56/56 PASS**. The previously qualified complete Gate 0 remains
MPPI upstream **17/17 PASS**, with physical-topic isolation and the independent
safety-veto static contract passing. `ORPHAN_PROCESSES_BEFORE_START=0`.

`GATE_A_PREFLIGHT=PASS`.

### Gate B — zero-step lockstep

The zero-step launch used ROS domain 226, `qualification_step_limit:=0`, the
isolated overlay `/tmp/laksa-c1.2-runtime/install`, and output directory
`/tmp/laksa-c1.2d-results/closed_loop/zero_step`. The launch was owned by the
qualified bounded process-session mechanism.

Gate B stopped before publishing a usable initial command and before any
`Gym.step()` call. Both `c1_gym_adapter` and
`c1_nav2_ackermann_adapter` failed during Python module import with:

```text
ModuleNotFoundError: No module named 'ackermann_msgs'
```

The pinned MPPI host initialized successfully, but launch shutdown followed
the two required child failures. Consequently no `summary.json`, controller
telemetry, state advance, command, or Gym evidence was produced. Gate B cannot
claim timestamp, TF, feasibility, or safety-veto success.

The dependency exists in the isolated extracted ROS root at
`/tmp/laksa-c1-native/ackermann_root/opt/ros/humble`, including its Python
module and package-local setup. Gate-0 compilation had resolved it through
`CMAKE_PREFIX_PATH`, but the closed-loop runtime environment did not source or
otherwise add that isolated root to Python/package resolution. Direct
`import ackermann_msgs` therefore failed after sourcing the base Humble and C1
overlays. The package manifest already declares `ackermann_msgs`; this is an
isolated runtime-overlay integration defect, not controller behavior.

Process cleanup remained correct despite the launch failure:

- launch process group: `3324374`
- launch leader return code: `0`
- bounded lifecycle phase: `already_exited`
- remaining owned PIDs: none
- C1 orphan processes: `0`

The top-level launch command returning zero despite required child-process
failure is a separate harness-hardening issue. It did not convert Gate B into
a pass because the required result artifacts were absent and the child errors
were explicit.

`ZERO_STEP_LOCKSTEP=FAIL_ENVIRONMENT_DEPENDENCY`.

Per the qualification stop rule, Gate C (one-step), short horizon, x3
determinism, and Trial 1 were not run. No controller parameters, course,
raceline, costmap, footprint, Gym dynamics, start pose, or ThreeLapGate
semantics were changed.

### Closed-loop first blocker and next action

`FIRST_BLOCKER=ISOLATED_ACKERMANN_MSGS_RUNTIME_OVERLAY_NOT_SOURCED`.

The next action is to integrate the existing `ackermann_msgs` dependency into
the isolated C1 runtime overlay, add a preflight that proves both ROS package
prefix resolution and Python importability, and propagate a nonzero result
when a required launch child exits in error. Then restart Gate B from the
beginning. Gate C must not run until that new Gate B passes. The correction
must remain isolated from `/opt/ros`, production workspaces, services, and
physical interfaces.
