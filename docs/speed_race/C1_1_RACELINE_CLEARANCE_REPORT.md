# C1.1 Full-Body Raceline Clearance

## Executive result

The demonstrated static raceline-clearance defect is corrected without changing
the canonical course, LAKSA_PROXY_V0 geometry, Gym collision code, controller
mathematics, controller parameters, dynamics, or mission logic.

The corrected raceline passes the static full-body preflight, but the first
unchanged closed-loop trial fails during controller tracking near Start/Finish.
Per the C1.1 stop rule, no controller tuning and no second or third official
trial were performed.

- `C1_1_GEOMETRY=PASS`
- `C1_1_RUNTIME=FAIL_CONTROLLER_TRACKING`
- `C1_1_STATUS=PARTIAL`

## Pinned upstream research

All conclusions below were checked against the pinned sources already used by
C1.

| Repository / SHA | File and symbol | Finding |
|---|---|---|
| `CL2-UWaterloo/Raceline-Optimization@9290c5d503462e46f7e3e9033002e7ddf165ba7b` | `params/f110.ini`, `optim_opts_mincurv.width_opt` | `width_opt` is documented as vehicle width for optimization *including safety distance*. |
| `trajectory_planning_helpers@fde6cee2b7bf6dd7d0f8f3d32f6a1be3cfe35b56` | `trajectory_planning_helpers/opt_min_curv.py::opt_min_curv` | The reference track stores right and left widths. The admissible reference-point deviations are each reduced by `w_veh / 2`. |
| `CL2-UWaterloo/Raceline-Optimization@9290c5d503462e46f7e3e9033002e7ddf165ba7b` | `helper_funcs_glob/src/calc_min_bound_dists.py::calc_min_bound_dists` | Waterloo's post-check evaluates all four corners using physical vehicle length and width. |
| `f1tenth/f1tenth_gym@bdaec1420c3b0f103858d289866d0d4e2e597c30` | `f1tenth_gym/envs/simulator.py::SimulatorParameters.ttc_threshold` | The collision threshold is `0.005 m`. |
| same | `f1tenth_gym/envs/lidar/laser_models.py::check_ttc_jit` | Collision is true when `scan - side_distances <= ttc_threshold`; this is geometric clearance, not time-to-collision in the current implementation. |
| same | `f1tenth_gym/envs/lidar/laser_models.py::distance_transform` and `ScanSimulator2D.__init__` | Map ray marching uses an EDT scaled by raster resolution and `eps=0.0001 m`. |

No research contradiction was found. The prior `5 mm` interpretation was
correct, and the optimizer's `w_veh` is the supported mechanism for reserving a
center-reference corridor for vehicle width plus safety distance.

## Root cause and correction

The former `effective_vehicle_width_m=0.336` produced an ideal minimum
full-body clearance of `0.004017015281351566 m`, less than Gym's `0.005 m`
collision threshold even before raster and output-discretization uncertainty.

The required preflight clearance is source-derived:

```text
required = Gym TTC threshold
         + raster half-cell diagonal
         + Gym ray-march epsilon
         + bounded raceline chord error

         = 0.005
         + 0.02 * sqrt(2) / 2
         + 0.0001
         + 0.91431 * 0.2^2 / 8

         = 0.023813685623730953 m
```

The chord-error term is the circular-arc sagitta upper bound for the frozen
maximum curvature and the generated `0.2 m` output spacing. The raster term is
the maximum center-to-corner distance of one `0.02 m` occupancy cell.

Using the pinned Waterloo optimizer unchanged, deterministic 1 mm input sweeps
showed `0.3805 m` failed and `0.381 m` was the smallest tested 1 mm width that
passed. The wrapper now supplies `w_veh=0.381 m`. This is a virtual optimization
width including safety distance; physical LAKSA width remains `0.296 m`.

The corrected generated artifact has:

- raceline SHA256: `1086933179481201163c4ce30d566aca6a4b6377bd723632583f3eb935e043c4`
- minimum full-body vector clearance: `0.024078635053796427 m`
- required clearance: `0.023813685623730953 m`
- clearance margin: `0.00026494943006547325 m`
- generated maximum curvature: `0.6988846677794767 1/m`
- fine-path maximum curvature: `0.707976208 1/m`
- exact Gym raster fine-path minimum `scan - body`: `0.013085814 m`
- exact Gym threshold: `0.005 m`
- exact Gym collision samples: `0`

Two independent ARM64 generations produced identical raceline artifacts and
hashes.

## Static preflight

The validator now:

1. checks the four corners of the asymmetric Gym collision body (including its
   `+0.135 m` longitudinal center offset),
2. measures against exact canonical centerline segments,
3. reconstructs and verifies the clearance formula from manifest values,
4. rejects the historical `4.017 mm` case,
5. runs before Gym environment creation.

Result: `PASS`.

## Validation

- isolated ROS 2 Humble build: `PASS`
- `colcon test`: `32 tests, 0 errors, 0 failures, 0 skipped`
- C0 tests: `4/4 PASS`
- C1 tests: `28/28 PASS` (including runtime/static tests)
- physical-topic isolation: `PASS`
- live adapter-only graph: `/c1/drive_request`, `/c1/drive_applied`, `/c1/odom`, `/tf`; no prohibited physical motion topic
- canonical source SHA256 unchanged: `222c897f6835a4877318cfd0ac7d76be98b7173faaeec44e028814ca3c6eb13e`
- canonical geometry SHA256 unchanged: `2c76075f838a7a1c3e0891385b27f2e6f26e641ad13068280093fda273a84858`

## Official Trial 1

The first trial used the unchanged Waterloo Pure Pursuit configuration, KS,
`dt=0.01`, seed `12345`, speed `1.0 m/s`, and the existing ThreeLapGate.

Result: `FAIL_CONTROLLER_TRACKING`; the sequence stopped after Trial 1 as
required.

- simulated time: `0.8200000000000005 s`
- simulator steps: `82`
- completed laps: `0`
- collision edges: `1`
- off-track events: `1`
- reverse commands: `0`
- invalid commands: `0`
- maximum absolute steering: `0.2879999876022339 rad`
- final applied command: steering `0`, speed `0`
- simulator steps after terminal: `0`
- terminal state: `FAULT:collision`
- clean shutdown: `PASS`
- orphan C1 processes: `0`

First failure evidence:

- location: Start/Finish straight, approximately `0.60 m` along the new raceline
- vehicle pose: `(21.21421241760254, 2.146467924118042, -0.42329955101013184)`
- nearest generated raceline sample: `(21.2530405, 2.0320130)`
- raceline curvature there: `-0.0002008 1/m`
- signed CTE: `0.1144571612066219 m`
- heading error: `-0.42334264933125354 rad`
- requested/applied steering: `+0.2879999876022339 rad` (hard limit)
- requested/applied speed: `1.0 m/s`
- collision/off-track: `true/true`
- controller latency: not instrumented by the existing C1 evidence contract

The static raceline itself is admissible. The unchanged controller begins from
the frozen start pose about `0.2667 m` from the raceline, initially commands the
opposite steering limit, then reverses to the positive steering limit before
the collision. This is a measured controller-tracking/startup-alignment issue,
not a residual static raceline-clearance failure.

Raw runtime CSV/log evidence remains in private temporary storage and is not
published in Git.

## Files changed

- `firmware/esp32-s3/jetson/laksa_speed_race/config/c1_raceline.yaml`
- `firmware/esp32-s3/jetson/laksa_speed_race/course/canonical/speed_course/course_manifest.json`
- `firmware/esp32-s3/jetson/laksa_speed_race/course/canonical/speed_course/pure_pursuit_raceline.csv`
- `firmware/esp32-s3/jetson/laksa_speed_race/course/canonical/speed_course/speed_course_raceline.csv`
- `firmware/esp32-s3/jetson/laksa_speed_race/course/scripts/generate_c1_raceline.py`
- `firmware/esp32-s3/jetson/laksa_speed_race/laksa_speed_race/course_validation.py`
- `firmware/esp32-s3/jetson/laksa_speed_race/laksa_speed_race/gym_adapter_node.py`
- `firmware/esp32-s3/jetson/laksa_speed_race/test/test_c1_course.py`
- `firmware/esp32-s3/jetson/laksa_speed_race/test/test_three_lap_runtime.py`
- `docs/speed_race/C1_1_RACELINE_CLEARANCE_REPORT.md`

## Next action

Run a separate research-before-implementation audit of the Pure Pursuit startup
alignment and first-second steering behavior. Preserve the corrected raceline,
course, vehicle geometry, Gym collision semantics, and all C1.1 acceptance
gates unchanged.
