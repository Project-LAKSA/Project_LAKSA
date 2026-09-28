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

## Gate B isolated `ackermann_msgs` runtime integration

### Dependency forensics and source order

The ARM64 `ackermann_msgs` dependency is not installed in `/opt/ros/humble`,
the production LAKSA workspace, or the third-party production workspace. It is
the ROS Humble binary package `ros-humble-ackermann-msgs` version 2.0.2,
license BSD, extracted exclusively for C1 from:

```text
/tmp/laksa-c1-native/debs/ros-humble-ackermann-msgs_2.0.2-3jammy.20260907.212421_arm64.deb
```

Its package prefix and install space are both:

```text
/tmp/laksa-c1-native/ackermann_root/opt/ros/humble
```

The package-local canonical setup file is:

```text
/tmp/laksa-c1-native/ackermann_root/opt/ros/humble/share/ackermann_msgs/local_setup.bash
```

Gate-0 compilation had exposed that prefix only through `CMAKE_PREFIX_PATH`.
The failed Gate-B runtime sourced `/opt/ros/humble/setup.bash` followed by the
isolated C1 install, so neither `AMENT_PREFIX_PATH` nor `PYTHONPATH` contained
the extracted package. The corrected deterministic order is:

1. `/opt/ros/humble/setup.bash`;
2. the package-local `ackermann_msgs/local_setup.bash` above;
3. `/tmp/laksa-c1.2-runtime/install/setup.bash`;
4. prepend `/tmp/laksa-c1.2-runtime/venv/lib/python3.10/site-packages` to
   `PYTHONPATH`.

No package was copied, installed globally, or added to `/opt/ros`.

### Runtime preflight and fail-fast harness

`runtime_preflight.py` now requires all of the following before a launch can
start:

- `ackermann_msgs.msg` imports;
- `AckermannDriveStamped` imports;
- `ros2 pkg prefix ackermann_msgs` succeeds;
- the resolved prefix exactly equals the configured isolated prefix;
- the package-local setup file exists.

`c1_native_runtime.sh` establishes the source order above and invokes the
source-controlled `c1_closed_loop_qualification` harness. The harness owns the
complete launch session, writes structured result evidence, rejects reused
output directories, and returns nonzero when preflight, readiness, a critical
child, required evidence, or bounded cleanup fails.

The MPPI launch now treats early exits from the raceline node, MPPI lockstep
host, and Ackermann adapter as critical launch shutdown events. A separate
readiness-only mode excludes Gym and requires two consecutive graph snapshots
containing:

```text
nodes:  /c1/nav2_raceline
        /c1/mppi_lockstep_host
        /c1/nav2_ackermann_adapter

topics: /c1/nav2_path
        /c1/nav2_cmd_vel
        /c1/drive_request
```

All graph probes use the exact launch environment, including the same
`ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY=1`, and use bounded 3-second discovery
spins. Final readiness qualification on domain 232 passed; graceful shutdown
observed four owned processes and left zero PIDs.

The exact runtime dependency preflight passed:

```text
ACKERMANN_MSGS_IMPORT=PASS
ACKERMANN_MSGS_ROS_PREFIX=PASS
ACKERMANN_MSG_CLASS_IMPORT=PASS
RUNTIME_OVERLAY=PASS
```

Targeted environment, child-propagation, lifecycle, and isolation tests passed
**23/23**. After the final implementation, the complete LAKSA Python suite
passed **70/70** on the ARM64 Jetson. The isolated `laksa_speed_race` rebuild
passed.

### Gate B rerun and stop decision

Gate B was restarted from zero using a new output directory. Preflight passed,
but the invocation selected `ROS_DOMAIN_ID=233`. Fast DDS rejected that domain
before controller evaluation because its calculated RTPS port exceeded the
valid port range:

```text
Calculated port number is too high. Probably the domainId is over 232 or
portBase is too high.
```

The MPPI lockstep host exited with status 1. The new critical-child launch
handler immediately shut down the other three C1 children, and the parent
harness returned status 1 because no valid zero-step evidence existed. This is
direct runtime proof that critical-child failure propagation works.

No `summary.json` or controller telemetry was produced, so no controller
evaluation occurred and no `Gym.step()` was called. The owned process group
terminated in the `already_exited` phase with zero remaining PIDs.

Per the explicit Gate-B stop rule, Gate B was not retried with a valid domain
and Gate C was not run.

Frozen artifacts remained unchanged:

- C1.1 raceline SHA256:
  `22ad91de3edbcbdf765f2cf223db53d43be820d829409da356a9def5a4c41783`
- canonical manifest SHA256:
  `b0e2e709fb2cb664600353ffb6145eff08f069e688e847c9ffc390eae8e26ead`
- ThreeLapGate SHA256:
  `5255497ba4e81a10988debf9e49ae832cfaea14ff071d113c1c72fb560364ba1`

Production source remained clean at
`1f0db4f027e2d3aa97d83d2c0a15216627efb5b1`; no production service or physical
hardware was touched.

`ZERO_STEP_LOCKSTEP=FAIL_INVALID_ROS_DOMAIN_ID`.

`FIRST_BLOCKER=INVALID_ROS_DOMAIN_ID_233_FASTDDS_PORT_OVERFLOW`.

The next action is to add an explicit valid-domain preflight and restart Gate B
from scratch with a fresh isolated ROS domain in the supported range 0–232.
Gate C remains unauthorized until that Gate B produces one valid controller
evaluation and exactly zero Gym steps.

## Gate B ROS domain allocation disposition

### Root cause and verified DDS bounds

No source-controlled allocator selected domain 233. The preceding qualification
commands supplied domains manually and sequentially: 227 for dependency
preflight, 228–232 for readiness development, then 233 for Gate B. The harness
accepted that caller-provided integer without an independent range check.

The Jetson Humble environment reports `rmw_fastrtps_cpp`. Harmless local ROS
CLI probes proved that domains 0 and 232 initialize successfully while domain
233 exits nonzero. The verified interval is therefore 0–232 for this runtime.

### Bounded allocator and collision avoidance

`ros_domain.py` now implements two independent protections:

1. every explicit override is parsed as an integer and validated within
   0–232 before dependency preflight or any C1 child starts;
2. dynamic C1 allocation maps every integer seed into the high-domain pool
   200–232 using `200 + (seed % 33)`, then rotates only within that pool.

The allocator acquires a nonblocking `flock` lease under
`/tmp/laksa-c1-domain-locks` and accepts a candidate only when a bounded,
daemon-free ROS discovery probe finds no active graph. It never scans, kills,
or alters unrelated ROS processes. Domain 0 and the lower domain range remain
outside the C1 allocation pool to avoid default/production graphs.

The exact historical allocator input 233 now maps to domain 202. An explicit
`--domain-id 233` was independently rejected with status 1 before
`ackermann_msgs` preflight or process creation:

```text
invalid ROS_DOMAIN_ID 233 from --domain-id;
allowed integer range is 0..232
```

Boundary, malformed-value, wide-input-range, collision rotation, historical
233, and no-child-on-invalid-override tests passed. The targeted runtime suite
passed **28/28** and the complete LAKSA Python suite passed **86/86**. The
isolated package rebuild passed.

### Single Gate B requalification

The one authorized Gate-B rerun used allocator seed 233 and selected fresh
domain 202. The lease was held through the run. The dependency preflight and
two-snapshot readiness handshake passed on that same domain before Gym was
started. Readiness observed the three required nodes and all required topics.

Gym startup then failed before initial-state publication:

```text
ModuleNotFoundError: No module named 'f1tenth_gym'
```

The isolated source checkout exists at
`/tmp/laksa-c1-native/f1tenth_gym`, but it is neither installed nor included in
the current isolated runtime Python path. Neither
`/tmp/laksa-c1-native/venv/bin/python` nor
`/tmp/laksa-c1.2-runtime/venv/bin/python` exists as an executable environment;
the configured site-packages path therefore does not provide the simulator
module.

The critical Gym-child failure shut down the launch. The parent harness
returned nonzero because no `summary.json` existed. Controller telemetry
contains only its header: controller evaluations were zero and Gym steps were
zero. No command, state stamp, or safety-veto evaluation can be claimed.

A harmless unrelated `sleep` sentinel remained alive through the complete
failure and shutdown sequence, proving unrelated-process preservation. The
owned C1 process group left zero remaining PIDs. Gate C was not run and Gate B
was not retried.

The pre-existing unrelated submodule deletion at
`firmware/esp32-s3/components/esp32_BNO08x` was not modified, restored, staged,
or incorporated into C1.2d.

`ZERO_STEP_LOCKSTEP=FAIL_F1TENTH_GYM_RUNTIME_IMPORT`.

`FIRST_BLOCKER=ISOLATED_F1TENTH_GYM_RUNTIME_NOT_IMPORTABLE`.

The next action is a separately authorized runtime dependency correction that
installs or exposes the already-pinned `/tmp/laksa-c1-native/f1tenth_gym`
checkout inside the isolated C1 Python environment, adds a fail-fast import
and provenance check, and then restarts Gate B once. Gate C remains
unauthorized.

## Gate B F1TENTH Gym runtime provenance disposition

### Pinned checkout and offline import layout

The authorized checkout is:

```text
/tmp/laksa-c1-native/f1tenth_gym
```

It is a clean detached checkout of `f1tenth/f1tenth_gym` at the frozen SHA:

```text
bdaec1420c3b0f103858d289866d0d4e2e597c30
```

The pinned revision uses a root package layout: its import package is
`/tmp/laksa-c1-native/f1tenth_gym/f1tenth_gym`, and project metadata is in the
checkout-root `pyproject.toml`. The runtime now prepends that checkout root to
`PYTHONPATH`; it does not install a PyPI package, clone another copy, or modify
the checkout. Existing frozen Python dependencies are exposed from the
already-populated offline directory `/tmp/laksa-c1-native/pydeps`. Importing
all Gym modules used by `gym_adapter_node.py`, including `F110Env`, passed in
the isolated runtime without network access.

The resulting Python path order is:

1. `/tmp/laksa-c1-native/f1tenth_gym`;
2. `/tmp/laksa-c1-native/pydeps`;
3. `/tmp/laksa-c1.2-runtime/venv/lib/python3.10/site-packages`;
4. the ROS and isolated C1 overlay paths already established by their setup
   files.

### SHA and import-provenance preflight

Before any critical ROS child starts, the harness now:

- verifies `f1tenth_gym/f1tenth_gym/__init__.py` exists in the configured
  checkout;
- reads the checkout's actual `git rev-parse HEAD`;
- requires exact equality with the pinned SHA;
- requires `git status --short` to be empty;
- imports `f1tenth_gym`;
- resolves `f1tenth_gym.__file__` and requires it to lie inside that checkout's
  package directory.

An import from system or user site-packages, another checkout, a modified
checkout, or a mismatched SHA fails preflight before runtime process creation.
The wrong-provenance regression test passed. The targeted runtime suite passed
**34/34**. The complete LAKSA Python suite passed **90/90**, and the isolated
colcon package test passed with zero failures. No test was disabled.

The real preflight selected domain 202 and recorded:

```text
F1TENTH_GYM_IMPORTED_FROM=
  /tmp/laksa-c1-native/f1tenth_gym/f1tenth_gym/__init__.py
F1TENTH_GYM_ACTUAL_SHA=
  bdaec1420c3b0f103858d289866d0d4e2e597c30
F1TENTH_GYM_GIT_STATUS=CLEAN
F1TENTH_GYM_PROVENANCE=PASS
```

### Single Gate B requalification

The single authorized Gate-B retry started from zero C1 processes and used the
corrected allocator, which selected and leased fresh domain 207 from seed
`1946542349998266247`. The complete domain, Ackermann-message, overlay, Gym
import, Gym path, and Gym SHA preflight passed before launch. The two-snapshot
critical-child readiness handshake passed.

Exactly one canonical initial state produced exactly one MPPI controller
evaluation. The lockstep host copied the odometry header to the returned
`TwistStamped`, and the Ackermann adapter copied that same header to the drive
request. The Gym state gate accepted the same stamp:

```text
STATE_STAMP=1790601053385046456
CONTROLLER_INPUT_STAMP=1790601053385046456
CONTROLLER_OUTPUT_STAMP=1790601053385046456
CONTROLLER_EVALUATIONS=1
GYM_STEPS=0
```

The command was finite, physically feasible, and accepted unchanged by the
independent safety veto:

```text
vx=0.016281509771943092 m/s
wz=0.0044019222259521484 rad/s
kappa=0.27036327021328854 1/m
steering=0.08737466934699005 rad
safety_veto=PASS
downstream_feasibility_clamp_activations=0
```

The zero-step qualification limit intentionally produced terminal zero without
calling `Gym.step()`. Evidence recorded one accepted request, no duplicate or
mismatched state stamps, zero simulator steps, zero post-terminal steps, and
final applied command `(0.0 rad, 0.0 m/s)`. The launch exited with status 0;
the owned process group had no remaining PIDs. A harmless pre-existing test
sentinel remained alive throughout, proving unrelated-process preservation.

`ZERO_STEP_LOCKSTEP=PASS` and `ZERO_STEP_CAUSALITY=PASS`.

Gate C was not run. It is now authorized by the completed Gate-B contract, but
requires a separate explicit task.

Frozen artifacts remained unchanged. The C1.1 raceline still hashes to
`22ad91de3edbcbdf765f2cf223db53d43be820d829409da356a9def5a4c41783`.
Course, costmap, footprint, ThreeLapGate, controller parameters, vehicle model,
and F1TENTH Gym source were not modified. Production and physical hardware were
not touched. The unrelated `firmware/esp32-s3/components/esp32_BNO08x`
submodule deletion was not modified or staged by this work.

## Gate C — One-Step Lockstep

Gate C was executed once on the ARM64 Jetson with no stale C1 process present.
The complete preflight passed before launch: the dynamically allocated domain
was valid and empty, `ackermann_msgs` resolved from the isolated overlay, and
the clean Gym checkout resolved from
`/tmp/laksa-c1-native/f1tenth_gym/f1tenth_gym/__init__.py` at
`bdaec1420c3b0f103858d289866d0d4e2e597c30`. The C1.1 raceline remained
byte-identical at
`22ad91de3edbcbdf765f2cf223db53d43be820d829409da356a9def5a4c41783`.

The harness gained a dedicated `one-step` mode. It sets the existing
`qualification_step_limit` to one and validates the resulting artifacts. The
Gym adapter already stops before publishing post-step odometry when that limit
is reached; consequently state N+1 cannot trigger a second controller
evaluation. For evidence only, the adapter records state N, state N+1, their
logical simulator stamps separated by the frozen 0.01-second `dt`, command
index, collision/off-track state, and full-body clearance using the existing
C1.1 four-corner/polyline clearance function. No controller, simulator, course,
or safety behavior changed.

The isolated rebuild passed. Targeted one-step/runtime tests passed **36/36**,
and the complete LAKSA Python suite passed **92/92**.

The single Gate-C run used dynamically allocated domain 204 and produced this
state N:

```text
x=20.628631591796875 m
y=2.298687696456909 m
yaw=0.0 rad
speed=0.0 m/s
stamp=1790601697079106871 ns
full_body_clearance=0.30919994145690904 m
```

MPPI evaluated that state once and returned:

```text
vx=0.016281509771943092 m/s
wz=0.0044019222259521484 rad/s
kappa=0.27036327021328854 1/m
equivalent_steering=0.08737466934699005 rad
effective_turning_radius=3.6987272687266426 m
```

The command was finite and within the frozen ±0.288-rad steering envelope.
The downstream feasibility clamp did not activate, and the independent safety
veto passed the exact command applied to Gym. Command index 1 caused exactly
one `Gym.step()`.

State N+1 was:

```text
x=20.628631591796875 m
y=2.298687696456909 m
yaw=5.0986074029424344e-08 rad
speed=0.0001548371510580182 m/s
stamp=1790601697089106871 ns
full_body_clearance=0.3091999338599842 m
collision=false
off_track=false
```

The measured deltas were:

```text
dx=0.0 m
dy=0.0 m
dyaw=+5.0986074029424344e-08 rad
dspeed=+0.0001548371510580182 m/s
```

At this very small first command and 10-ms integration interval, position did
not change at the simulator's reported float32 resolution. Speed and yaw did
advance, all values remained finite, and there was no teleportation. Positive
steering predicts increasing yaw; observed yaw increased, so steering-sign
semantics passed.

The odometry input stamp, MPPI output stamp, safety evidence stamp, and drive
request stamp were all `1790601697079106871`. State N+1's logical simulator
stamp advanced by exactly 10,000,000 ns. Evidence contained one controller row,
one command row, one trajectory row, one accepted request, one simulator step,
zero duplicate/mismatched stamps, and zero post-terminal steps. N+1 was not
fed back to MPPI.

The intentional one-step qualification limit produced terminal zero and clean
process shutdown. The launch returned zero, no C1 process remained, and an
unrelated sentinel survived unchanged.

```text
ONE_STEP_LOCKSTEP=PASS
ONE_STEP_CAUSALITY=PASS
CONTROLLER_EVALUATIONS=1
GYM_STEPS=1
CLEAN_SHUTDOWN=PASS
ORPHAN_PROCESSES=0
SHORT_HORIZON_AUTHORIZED=YES
```

No short horizon, repeat, determinism run, or Trial 1 was executed. Those
remain outside this Gate-C task.

## C1.2d Short-Horizon Closed Loop

Exactly one short-horizon run was executed on the ARM64 Jetson. No prior C1.2d
artifact defined a different exact horizon, so the run used the task's minimum
authorized horizon of 50 causal controller evaluations and state transitions.
This reaches beyond the historical RPP S32 and S39 regions without tuning the
horizon after observing results.

Before the run, the isolated packages rebuilt successfully. The Ackermann
feasibility C++ suite passed **5/5**, and the complete LAKSA Python suite passed
**94/94**. Pinned Gym provenance, the runtime overlay, the frozen raceline hash,
physical-topic isolation, and the absence of stale C1 processes were verified.
The preflight allocator selected domain 226; the single dynamic run independently
selected and leased fresh domain 221.

The run completed all 50 requested steps:

```text
CONTROLLER_EVALUATIONS=50
GYM_STEPS=50
LOCKSTEP_CAUSALITY=PASS
DUPLICATE_STAMP_COUNT=0
STALE_COMMAND_COUNT=0
HIDDEN_GYM_STEP_COUNT=0
COLLISIONS=0
OFF_TRACK_EVENTS=0
REVERSE_COMMANDS=0
INVALID_COMMANDS=0
SAFETY_VETO_COUNT=0
DOWNSTREAM_FEASIBILITY_CLAMP_ACTIVATIONS=0
```

Each telemetry row contains the same state stamp at the MPPI input and output,
one feasibility decision, one drive request, and one Gym transition. The exact
executable command passed the independent safety veto before each Gym call.
All commands were finite and inside the frozen Ackermann envelope. The maximum
absolute steering was `0.08737466934699005 rad`; no steering saturation occurred.

The 0.5-second tracking result was safe but does not yet demonstrate convergence.
Using the first and last measured closed-loop samples, signed CTE changed from
`0.26671643419952507 m` to `0.26683371350746476 m`, an increase of about
0.117 mm. Heading error changed from `-0.000023786786456092557 rad` to
`0.003839877844177053 rad`. These errors remained controlled over this horizon,
but the evidence must not be read as a longer-horizon tracking or lap result.

```text
CTE_RMS=0.2667444078469783 m
CTE_P95=0.26681693681593427 m
CTE_MAX=0.26683371350746476 m
HEADING_ERROR_RMS=0.00210082211541244 rad
HEADING_ERROR_P95=0.0036173509335810203 rad
HEADING_ERROR_MAX=0.003839877844177053 rad
```

The raw reset observation does not provide Gym's raceline CTE and therefore
contains zero in `summary.json`. The initial CTE and heading values above come
from the first machine-readable closed-loop sample; the independent metrics
artifact records this scope explicitly.

At MPPI step 32:

```text
x=20.647403717041016 m
y=2.298712730407715 m
yaw=0.0024150684475898743 rad
actual_speed=0.13028880953788757 m/s
CTE=0.26674258799167666 m
heading_error=0.0023658613790815686 rad
curvature=0.09163104620565092 1/m
steering=0.029679741065838715 rad
full_body_clearance=0.30816354350532355 m
safety_veto=PASS
```

At MPPI step 39:

```text
x=20.657482147216797 m
y=2.2987396717071533 m
yaw=0.002913136500865221 rad
actual_speed=0.1569029837846756 m/s
CTE=0.2667696209893062 m
heading_error=0.0029062067836678196 rad
curvature=0.07438169349455119 1/m
steering=0.02409500466951955 rad
full_body_clearance=0.30792810881591065 m
safety_veto=PASS
```

Thus the new MPPI trajectory had not recreated RPP's sustained maximum-steering
policy by its own S32/S39 indices. This is a trajectory-index comparison only;
those states are not claimed to equal the historical RPP states.

Actual speed rose from `0.0 m/s` to `0.18972113728523254 m/s`, with mean
`0.09792036229511723 m/s`, p95 `0.18333093971014022 m/s`, and maximum
`0.18972113728523254 m/s`. Commanded speed averaged
`0.17418503526598214 m/s` and peaked at `0.24063703417778015 m/s`.
Full-body clearance decreased from `0.30919994145690904 m` to its minimum
`0.3074667817231728 m` at step 50, remaining positive throughout.

Maximum command-to-command steering change was
`0.024858176040717925 rad`; the corresponding simulated 10-ms command slew was
`2.4858176040717925 rad/s`. This is descriptive simulation telemetry only.
No authoritative physical steering-rate limit exists, so
`PHYSICAL_STEERING_RATE_QUALIFICATION=PENDING` remains mandatory.

Controller compute latency was 28.431155 ms p50, 33.01263185 ms p95, and
63.965212 ms maximum. Gym step latency was 1.4910375466570258 ms p50,
2.513581817038357 ms p95, and 92.37696195486933 ms maximum. Initialization is
reported separately by the harness; these values are the 50 recorded control
cycles, including first-cycle warm-up.

The step-limit stop is intentional. The generic three-lap `summary.json` marks
the terminal state as `FAULT:qualification_step_limit_reached` and its
three-lap overall acceptance as failed, while the mode-specific harness verifies
the exact 50-step stop and records `status=PASS`. Terminal zero was observed,
no post-terminal step occurred, the launch exited cleanly, no C1 process
remained, and an unrelated sentinel survived unchanged.

Machine-readable evidence is stored on the Jetson at
`/tmp/laksa-c1.2-short-horizon.PnsLEF` and mirrored privately at
`/private/tmp/laksa-c1.2-short-horizon.PnsLEF`. It includes raw summary,
trajectory, command, event, feasibility, and controller telemetry CSV files;
the merged `short_horizon_telemetry.csv`; `independent_metrics.json`; and these
static plots:

- `trajectory_vs_raceline.png`
- `cte_vs_step.png`
- `heading_error_vs_step.png`
- `steering_vs_step.png`
- `speed_vs_step.png`
- `clearance_vs_step.png`

No x3 determinism run or Trial 1 was executed. The completed short-horizon gate
authorizes x3 determinism, subject to separate authorization. Course, raceline,
costmap, footprint, ThreeLapGate, MPPI parameters, Gym source, production, and
physical hardware remained unchanged. The unrelated BNO08x submodule deletion
was neither modified nor staged.

## X3 determinism methodology (frozen before Run 1)

The existing C1.2d plan did not define a numerical tolerance. The pinned Humble
MPPI `NoiseGenerator` calls `xt::random::randn<float>` through xtensor's global
default `std::mt19937`. No C1.2d code calls `xt::random::seed`; each fresh host
process therefore starts from the deterministic default-constructed engine.
The frozen `regenerate_noises=false` parameter generates one fixed noise matrix
at initialization and reuses it. Gym independently resets every invocation with
the existing `seed=12345`; simulated LiDAR noise remains zero.

The X3 seed policy is therefore the already-qualified behavior, unchanged:

- MPPI: default-constructed xtensor `std::mt19937`, no added user-facing seed,
  and fixed initialization noise because `regenerate_noises=false`;
- Gym: explicit seed 12345 on every canonical reset;
- ROS domain: deterministic bounded allocation from each output path and an
  empty-graph lease. Domain IDs may differ and are not controller RNG inputs;
- wall-clock ROS stamps, process IDs, filesystem paths, and measured latencies
  are intentionally nondeterministic metadata and are excluded from trajectory
  equality.

Because each repeat uses the same executable, ARM64 host, deterministic
lockstep order, default MPPI random sequence, Gym seed, and frozen artifacts,
the numerical criterion is predeclared as **exact normalized reproducibility**.
For each of 50 steps, the parsed IEEE-754 values for pose, yaw, actual speed,
commands, curvature, steering, CTE, heading error, and full-body clearance must
compare exactly between every run pair. The canonical command hash is SHA256
over big-endian binary serialization of `(step, vx, wz, curvature, steering)`.
All pairwise maximum dynamic deltas must be zero and all three command hashes
must match. No tolerance will be introduced after observing results.

Safety repeatability is a separate criterion: every run must independently
complete 50 lockstep steps with zero collision, off-track, reverse, invalid
command, safety veto, feasibility clamp, duplicate stamp, stale command, hidden
Gym step, orphan process, or shutdown failure. Overall X3 passes only if both
exact numerical determinism and three-run safety repeatability pass.

## X3 determinism results

Exactly three fresh short-horizon invocations were executed after the preceding
criterion and seed policy were frozen. Each invocation reconstructed the Gym
environment, reset with seed 12345, created a new MPPI host and optimizer,
leased a fresh empty ROS domain, and started from zero C1 processes. Runs 1, 2,
and 3 used domains 202, 226, and 208 respectively. All three per-run preflights
passed.

Every run completed exactly 50 controller evaluations and 50 Gym steps with
lockstep causality passing. Each recorded zero duplicate stamps, stale commands,
hidden Gym steps, collision, off-track, reverse, invalid command, safety veto,
or downstream feasibility clamp. Terminal zero was observed, post-terminal
steps were zero, shutdown succeeded, no owned process remained, and an unrelated
sentinel survived all three invocations.

The dynamically relevant metrics were identical in all three runs:

| Metric | Run 1 | Run 2 | Run 3 |
|---|---:|---:|---:|
| CTE initial (m) | 0.26671643419952507 | 0.26671643419952507 | 0.26671643419952507 |
| CTE final (m) | 0.26683371350746476 | 0.26683371350746476 | 0.26683371350746476 |
| CTE RMS (m) | 0.2667444078469783 | 0.2667444078469783 | 0.2667444078469783 |
| CTE p95 (m) | 0.26681693681593427 | 0.26681693681593427 | 0.26681693681593427 |
| CTE max (m) | 0.26683371350746476 | 0.26683371350746476 | 0.26683371350746476 |
| Heading initial (rad) | -0.000023786786456092557 | -0.000023786786456092557 | -0.000023786786456092557 |
| Heading final (rad) | 0.003839877844177053 | 0.003839877844177053 | 0.003839877844177053 |
| Heading RMS (rad) | 0.00210082211541244 | 0.00210082211541244 | 0.00210082211541244 |
| Heading p95 (rad) | 0.0036173509335810203 | 0.0036173509335810203 | 0.0036173509335810203 |
| Heading max (rad) | 0.003839877844177053 | 0.003839877844177053 | 0.003839877844177053 |
| Max absolute steering (rad) | 0.08737466934699005 | 0.08737466934699005 | 0.08737466934699005 |
| Steering saturation count | 0 | 0 | 0 |
| Max command steering delta (rad) | 0.024858176040717925 | 0.024858176040717925 | 0.024858176040717925 |
| Max simulated steering demand (rad/s) | 2.4858176040717925 | 2.4858176040717925 | 2.4858176040717925 |
| Final speed (m/s) | 0.18972113728523254 | 0.18972113728523254 | 0.18972113728523254 |
| Mean speed (m/s) | 0.09792036229511723 | 0.09792036229511723 | 0.09792036229511723 |
| Max speed (m/s) | 0.18972113728523254 | 0.18972113728523254 | 0.18972113728523254 |
| Minimum full-body clearance (m) | 0.3074667817231728 | 0.3074667817231728 | 0.3074667817231728 |

Controller computation latency varied, as expected for excluded scheduling
metadata:

| Run | p50 (ms) | p95 (ms) | max (ms) |
|---|---:|---:|---:|
| 1 | 21.604275 | 28.54947835 | 66.013262 |
| 2 | 23.0635145 | 34.04288995 | 65.229881 |
| 3 | 30.784472 | 34.67946245 | 61.653491 |

The final state was exactly equal in all three parsed telemetry streams:

```text
x=20.676645278930664 m
y=2.298804998397827 m
yaw=0.0038593821227550507 rad
speed=0.18972113728523254 m/s
```

The pairwise maximum deltas were zero for x, y, Euclidean position, yaw, actual
speed, CTE, heading error, commanded vx, commanded wz, and steering. Canonical
big-endian command serialization produced the same SHA256 for all runs:

```text
c9052d7ed66323e0869ae3d7b113f50aba99dd5579b86ff9a9afd6facab4b210
```

Therefore `X3_SAFETY_REPEATABILITY=PASS`,
`X3_NUMERICAL_DETERMINISM=PASS`, and `X3_DETERMINISM=PASS` under the criterion
frozen before Run 1. This authorizes Trial 1 under the C1.2d gate sequence, but
Trial 1 was not executed in this task. Physical steering-rate qualification
remains pending.

Raw machine-specific evidence is intentionally private and remains at:

```text
Jetson: /tmp/laksa-c1.2-x3-run1.kFE9Q7
Jetson: /tmp/laksa-c1.2-x3-run2.2aY55K
Jetson: /tmp/laksa-c1.2-x3-run3.DqhFT3
Local mirror: /private/tmp/laksa-c1.2-x3/run1
Local mirror: /private/tmp/laksa-c1.2-x3/run2
Local mirror: /private/tmp/laksa-c1.2-x3/run3
Comparison: /private/tmp/laksa-c1.2-x3/x3_comparison.json
```

Frozen course, raceline, costmap, footprint, ThreeLapGate, controller
parameters, and Gym source remained unchanged. Production and physical hardware
were not touched. The unrelated BNO08x submodule deletion was neither modified
nor staged.

## Trial 1 — Full Three-Lap Qualification

Trial 1 was authorized and executed exactly once from commit
`80e11379d97dbf2d9a0ee6cd108ac914b2e10a21`. The local and remote branch
heads matched before execution. Preflight passed with the pinned
`f1tenth_gym` checkout at
`bdaec1420c3b0f103858d289866d0d4e2e597c30`, the canonical C1.1 raceline
SHA256 `22ad91de3edbcbdf765f2cf223db53d43be820d829409da356a9def5a4c41783`,
ThreeLapGate SHA256
`5255497ba4e81a10988debf9e49ae832cfaea14ff071d113c1c72fb560364ba1`,
and controller-configuration SHA256
`87d9b54df6bda3a93b30bbd6c28e5b16bbf1e43c612ba08d0854595f38bf4692`.
The isolated ROS domain allocator selected domain 227. No stale C1 process was
present, Gym provenance and ROS dependency preflights passed, and physical
topic isolation remained active.

The qualification harness was extended only to expose the existing unlimited
qualification mode as `trial1` and to validate its evidence fail-closed. The
launch still uses the existing frozen ThreeLapGate with
`qualification_step_limit=-1`; no controller, critic, controller parameter,
course, raceline, costmap, footprint, vehicle model, Gym source, or gate
semantics changed. The validator requires exactly three completed laps, a
`COMPLETE` terminal state, terminal zero, no post-terminal steps, zero safety
incidents, one-to-one state/command/step row counts, matching stamps, physical
feasibility, and the lap-event sequence 1, 2, 3. Synthetic regression coverage
was added for these requirements. On the Jetson, the Ackermann C++ tests passed
5/5 and the Python suite passed 96/96 before the attempt.

### Result

`TRIAL_1_STATUS=FAIL_OFF_TRACK`. The controller completed 1,326 lockstep
evaluations and 1,326 Gym steps (13.259999999999762 simulated seconds), then
the full-body off-track check changed from false to true during partial lap 1.
No lap completed. The run stopped immediately; it was not retried, and Trials
2 and 3 were not run.

The first failure was at step 1326:

```text
completed laps: 0
pose: (23.793434143066406, 2.549774169921875, 0.14295849204063416)
actual speed: 0.24173088371753693 m/s
command: vx=0.24153591692447662 m/s, wz=0.0005316961323842406 rad/s
curvature: 0.0022013129109510086 1/m
equivalent steering: 0.0007132252622111841 rad
CTE: 0.512046343006913 m
heading error: 0.13401234342107493 rad
full-body clearance: -0.00007242838671644991 m
collision/off-track: 0/1
safety veto: not triggered
downstream feasibility clamp: not activated
state/controller input/controller output stamp: 1790620101790961833 ns
```

At the preceding step (1325), full-body clearance was still
`0.0003188512914837349 m`, CTE was `0.5117224803594151 m`, heading error was
`0.1339155742904934 rad`, and off-track was false. This establishes the first
causal boundary crossing between steps 1325 and 1326. The applied command was
finite and Ackermann-feasible, but the controller's long-horizon lateral error
had diverged from `0.26671643419952507 m` initially to
`0.512046343006913 m`. The independent safety veto passed the exact executable
command at the failure step; this attempt therefore exposes tracking divergence
to the course boundary and warrants a separate evidence-driven controller and
safety-layer audit. No tuning is performed here.

### Partial-run metrics

| Metric | Value |
|---|---:|
| CTE initial / final (m) | 0.26671643419952507 / 0.512046343006913 |
| CTE RMS / p95 / max (m) | 0.35756152947690606 / 0.4906782438292889 / 0.512046343006913 |
| Heading initial / final (rad) | -0.000023786786456092557 / 0.13401234342107493 |
| Heading RMS / p95 / max (rad) | 0.08727063796842102 / 0.13516109990781155 / 0.13892107929499797 |
| Max absolute steering (rad) | 0.08737466934699005 |
| Steering saturation count / fraction | 0 / 0.0% |
| Max command steering delta (rad) | 0.024858176040717925 |
| Max simulated steering-rate demand (rad/s) | 2.4858176040717925 |
| Command speed mean / p95 / max (m/s) | 0.24400664711031125 / 0.25136659294366837 / 0.25309669971466064 |
| Actual speed mean / p95 / max (m/s) | 0.23974773821343356 / 0.2501738891005516 / 0.2509780824184418 |
| Clearance initial / final / minimum (m) | 0.30919994145690904 / -0.00007242838671644991 / -0.00007242838671644991 |
| Minimum-clearance step / partial lap | 1326 / 1 |
| Controller latency p50 / p95 / max (ms) | 25.1600935 / 44.547588 / 68.855999 |
| Gym-step latency p50 / p95 / max (ms) | 1.5256130136549473 / 2.0773470168933272 / 92.21370203886181 |

Lap times and per-lap tracking metrics are `NOT_COMPLETED`: the failure occurred
before the first lap event. Collision, reverse-command, invalid-command,
safety-veto, downstream-clamp, duplicate-stamp, stale-command, and hidden-step
counts were all zero. State stamps matched controller input and output stamps
for all 1,326 rows, so `LOCKSTEP_CAUSALITY=PASS` for the attempted prefix.

The terminal state is `FAULT`, as required for the detected off-track event.
The final applied command was exactly `(0.0, 0.0)`, zero simulator steps
occurred after terminal, bounded process-group shutdown passed, no owned process
remained, and the unrelated-process sentinel survived. Production and physical
hardware were not touched. Physical steering-rate qualification remains
pending; the simulated demand is descriptive only.

Machine-specific evidence is intentionally private and remains at:

```text
Jetson raw evidence: /tmp/laksa-c1.2-trial1.oxCbxI
Local raw evidence: /private/tmp/laksa-c1.2-trial1.oxCbxI
Merged telemetry: /private/tmp/laksa-c1.2-trial1.oxCbxI/trial1_failure_telemetry.csv
Analysis: /private/tmp/laksa-c1.2-trial1.oxCbxI/trial1_analysis.json
Trajectory plot: /private/tmp/laksa-c1.2-trial1.oxCbxI/trial1_trajectory_vs_raceline.png
CTE plot: /private/tmp/laksa-c1.2-trial1.oxCbxI/trial1_cte.png
Heading plot: /private/tmp/laksa-c1.2-trial1.oxCbxI/trial1_heading_error.png
Steering plot: /private/tmp/laksa-c1.2-trial1.oxCbxI/trial1_steering.png
Speed plot: /private/tmp/laksa-c1.2-trial1.oxCbxI/trial1_speed.png
Clearance plot: /private/tmp/laksa-c1.2-trial1.oxCbxI/trial1_clearance.png
```

The Trial-1 verdict is `FAIL_OFF_TRACK`; `EXACT_THREE_LAPS=FAIL`. Trial 2 is
not authorized by this result. The next action is a separate research task to
explain the sustained MPPI lateral divergence and why the independent veto did
not stop before the full-body boundary crossing, without changing frozen race
artifacts or weakening safety criteria.
