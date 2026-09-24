# C1 implementation report

## Verified status

`C1_STATUS=FAIL`

The C1 software builds and tests in an isolated ROS 2 Humble environment on
the Jetson, and real closed-loop Gym/Waterloo runs were executed. C1 does not
pass: the frozen raceline has insufficient body clearance for Gym's unchanged
collision contract. No three-lap success is claimed.

## Frozen inputs

- Course source SHA256: `222c897f6835a4877318cfd0ac7d76be98b7173faaeec44e028814ca3c6eb13e`
- Course geometry SHA256: `2c76075f838a7a1c3e0891385b27f2e6f26e641ad13068280093fda273a84858`
- Geometry modified: NO
- Raceline SHA256: `5082057770360d4955195d66b2587102c03b7bea7f3c57da50ad957766f3a7f5`
- Pure Pursuit raceline SHA256: `50bfbbb27bf8ed25b74728bdab52a767689e7e51d4569b8707119d23f7ad3ec1`
- Raceline maximum curvature: `0.7149366116445661 1/m` on ARM64
- LAKSA proxy SHA256: `b9d072d92db353b0763fada60c40a85cdb82ce2b92b7c3c982652d3ef6719b76`
- Initial controller config SHA256: `083cfca2c03efe3b619bc3a86a88dc6dd1c51f054bd2ab981bc47244c5717083`

The raceline byte hash changed from the original x86-only artifact only to
make upstream-derived floating-point serialization deterministic across
x86_64 and ARM64. Source course geometry was not quantized or changed.

## Pinned upstreams

- F1TENTH Gym: `bdaec1420c3b0f103858d289866d0d4e2e597c30`
- Waterloo Pure Pursuit: `c20cf63d04b9841ffdb6b2f963bd737d78074136`
- Waterloo Raceline-Optimization: `9290c5d503462e46f7e3e9033002e7ddf165ba7b`
- trajectory_planning_helpers: `fde6cee2b7bf6dd7d0f8f3d32f6a1be3cfe35b56`

Controller and simulator mathematics were not modified.

## Jetson qualification

- Host: `ubuntu@192.168.55.1`
- OS: Ubuntu 22.04.5 LTS, `aarch64`
- ROS: Humble
- Execution: native isolated workspace under `/tmp/laksa-c1-native`
- ROS isolation: `ROS_DOMAIN_ID=221`, `ROS_LOCALHOST_ONLY=1`
- Production overlay: not sourced
- Fresh build: PASS, two packages
- `colcon test`: PASS, 30 tests, zero failures
- Physical topics in C1 graph: none
- Production source checkout: clean and unchanged at
  `1f0db4f027e2d3aa97d83d2c0a15216627efb5b1`

Docker was installed, but its daemon could not create a bridge because the
Jetson kernel lacked the required iptables raw table. Native Humble execution
was used without changing the host OS or production services.

## Evidence-driven integration fixes

1. C1 package data sources were made relative so `colcon --symlink-install`
   creates a valid share tree.
2. Optimizer-derived output serialization was fixed at seven decimal places
   after measured ARM/x86 jitter crossed the eighth decimal. Two ARM
   regenerations then matched the checked-in hash byte-for-byte.
3. Terminal evidence now persists `summary.json`, `trajectory.csv`,
   `commands.csv`, and `events.csv` without the former package-share scope
   error.
4. ROS shutdown is requested inside the timer callback and performed after
   the executor returns, avoiding the observed callback deadlock.
5. Gym now selects the already-approved `speed_course_hires.png` at 0.02 m
   instead of the coarser 0.05 m Nav2 raster. At the first measured failure
   pose this increased map clearance from 2.82 mm to 17.07 mm without changing
   course geometry.

## Closed-loop result

Nineteen real closed-loop executions were attempted; eighteen produced valid
terminal evidence after the evidence-persistence fix. The initial frozen
controller failed after 78 steps with `off_track`. Controlled upstream
parameter trials covered:

- lookahead: `0.8`, `0.9`, `0.95`, `0.965`, `0.9725`, `0.97625`,
  `0.978125`, `0.98`, and `1.0 m`
- `K_p`: `0.4`, `0.45`, `0.48`, `0.49`, `0.495`, `0.5`, and `0.6`
- speed: `0.8` and `1.0 m/s`

Every evidenced run failed before lap one, either with an unchanged Gym
collision or the full-body authoritative-corridor `off_track` gate. All
post-fix terminal runs had:

- terminal zero: PASS
- steps after terminal: 0
- reverse requests: 0
- invalid commands: 0
- maximum absolute steering: `0.2879999876 rad`
- clean adapter/controller shutdown: PASS
- orphan C1 processes: 0

The 18 complete machine-readable result sets are retained privately at
`/private/tmp/laksa-c1-runtime-evidence-a8e65c9`; raw run evidence was not
added to the public repository.

The best controller-tracking trials reached approximately 20 simulated
seconds with CTE RMS about 0.057--0.060 m, but did not complete a lap.
`CTE_PASS_THRESHOLD` remains `UNSET`.

## Proven geometric blocker

The frozen raceline's minimum ideal full-body corridor clearance is only
`0.0040170153 m`. Gym's unmodified wall-collision threshold is `0.005 m`,
before occupancy rasterization.

At the minimum-clearance frozen raceline sample
`(40.1623623, 7.7237627, -1.9171024)`, a direct Gym reset on the canonical
0.02 m raster measured:

- `scan - body_extent = -0.0072640421 m`
- collision after one zero-speed `env.step()`: true
- authoritative source geometry changed: NO

Thus even an ideal vehicle pose on the frozen raceline collides under the
approved simulator contract. Changing Pure Pursuit parameters cannot repair
that contradiction. Collision detection, vehicle dimensions, steering limits,
and off-track checks were not weakened.

## Required next decision

C1 needs a separately reviewed regeneration of the *derived raceline* with an
explicit positive full-body clearance that includes Gym's 5 mm collision
threshold and occupancy-raster tolerance. The canonical course geometry must
remain frozen. Alternatively, a different collision/raster contract would
require explicit approval; it was not assumed here. C2 is not authorized.

## Claim boundary

This work does not validate physical LAKSA dynamics, localization, mapping,
sensor behavior, tire/slip behavior, or physical success probability. No
physical hardware, GPIO, production workspace, or production service was
modified.
