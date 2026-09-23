# C1 implementation report

## Verified status

`C1_STATUS=PARTIAL`

The approved implementation is present and every executable non-ROS gate on
the current host passes. Official three-lap evidence is not claimed: this
macOS host has no Docker, ROS 2 Humble, or colcon. The remaining blocker is
environmental, not a Gym-core capability gap.

## Frozen course

- Source: `PREEXISTING_LAKSA_USER_APPROVED_RECONSTRUCTION`
- Source SHA256: `222c897f6835a4877318cfd0ac7d76be98b7173faaeec44e028814ca3c6eb13e`
- Geometry SHA256: `2c76075f838a7a1c3e0891385b27f2e6f26e641ad13068280093fda273a84858`
- Dimensions: `41.148 m x 14.3256 m`
- Width: `0.9144 m`
- Start: `(20.62863090812533, 2.2986877548722693, 0)`, crossing `+X`
- Geometry modified: NO

The task text supplied geometry digest `...ab4858`; no checked-in artifact has
that digest. Frozen bytes, approval metadata, and the course manifest agree on
`...a84858`, so the implementation preserved the approved asset.

## Deterministic raceline evidence

- Waterloo Raceline-Optimization: `9290c5d503462e46f7e3e9033002e7ddf165ba7b`
- trajectory_planning_helpers: `fde6cee2b7bf6dd7d0f8f3d32f6a1be3cfe35b56`
- Solver: `quadprog==0.1.7`
- Two independent generations: byte-identical
- Gym raceline SHA256: `84ce6c22991ef23b6c05048513c6e5f3661d00e06bc38036eb3e4b18bf3f3142`
- Pure Pursuit CSV SHA256: `74c9fa7a1a97541566af30c75ae468758180eacb588a17da9039d97c1a4dd618`
- Samples: 547, including exactly one terminal closure sample
- Maximum absolute curvature: `0.7149366116420127 1/m`
- Curvature limit: `0.91431 1/m`
- Full LAKSA body envelope: PASS
- Speed profile: constant `1.0 m/s`, acceleration `0`, reverse absent

The wrapper calls pinned `prep_track`, `opt_min_curv`, `create_raceline`, and
`calc_head_curv_an`. It contains no optimizer. A shape-only compatibility shim
normalizes an old `splev` return shape rejected by modern SciPy; numeric
distance and optimization mathematics are unchanged.

## Runtime implementation

- Gym core: `bdaec1420c3b0f103858d289866d0d4e2e597c30`
- Gym ROS reference: `08395766c4d9dc5a763381f1dd6fa4a3d68df66e`
- Waterloo Pure Pursuit: `c20cf63d04b9841ffdb6b2f963bd737d78074136`
- LAKSA proxy config SHA256: `b9d072d92db353b0763fada60c40a85cdb82ce2b92b7c3c982652d3ef6719b76`
- Controller config SHA256: `083cfca2c03efe3b619bc3a86a88dc6dd1c51f054bd2ab981bc47244c5717083`

The adapter owns Gym configuration, command validation, exactly-one-step
authority, lap/fault state, metrics, terminal zero, and shutdown. It imports
Gym's dynamics unchanged. Waterloo Pure Pursuit mathematics is unchanged; the
runtime image adds only its missing `nav_msgs` package-manifest declaration.

```text
/c1/odom -> Waterloo Pure Pursuit -> /c1/drive_request
          -> C1 Gym adapter -> Gym core -> /c1/odom
                              -> /c1/drive_applied
```

Forbidden physical command topics are absent from launch and config. Both
container services use no network, no privileges, dropped capabilities, no
devices, no production mounts, and no Docker socket.

## Tests executed

```text
python -m unittest discover -v -s test -p 'test_*.py'
24 tests passed
```

Additional verified checks:

- canonical course validator: PASS
- deterministic course rebuild twice: PASS
- deterministic rebuild fixture uses a disposable course copy and leaves the
  frozen checked-in raster bytes unchanged: PASS
- raceline generation twice: byte-identical PASS
- complete-body envelope and curvature validation: PASS
- pinned Gym core loads LAKSA_PROXY_V0: PASS
- Gym KS/RK4 reset and one headless `dt=0.01` step: PASS
- physical topic/container isolation: PASS
- CTE pass threshold remains `UNSET`

## External runtime blocker

`docker`, `ros2`, and `colcon` are absent from this host. Therefore Docker
image builds, the Waterloo ROS controller build, and official three-lap runs
are `NOT_EXECUTED_ENVIRONMENT_BLOCKED`. No lap/CTE/collision result is
fabricated.

## Exact continuation

On a Docker-capable machine, from the package directory:

```bash
LAKSA_GIT_SHA=$(git rev-parse HEAD) docker compose -f docker-compose.c1.yaml build raceline runtime
LAKSA_GIT_SHA=$(git rev-parse HEAD) docker compose -f docker-compose.c1.yaml run --rm raceline
LAKSA_GIT_SHA=$(git rev-parse HEAD) docker compose -f docker-compose.c1.yaml up --abort-on-container-exit runtime
```

After the first passing run, repeat the runtime command twice with distinct
output directories and validate all three `summary.json` files. Do not proceed
to C2 until all exact-three-lap and safety gates pass.

## Claim boundary

C1 does not validate tire/slip/friction/inertia fidelity, motor or servo
dynamics, physical localization, sensor noise, or physical maximum speed.
Physical hardware and production were not touched.
