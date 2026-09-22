# G2.1 read-only audit handoff

## Scope and immutable review target

- Repository: `Project-LAKSA/Project_LAKSA`
- Branch: `architecture/navigation-v2`
- G2.1 implementation-and-evidence commit:
  `26151a167d25bbb9595a282a91611226c7545f92`
- Review mode: **read only**

Do not edit, commit, push, deploy, open a pull request, start hardware, or
publish motion commands. Do not alter the frozen legacy branches.

G2.1 establishes the local-motion contract only:

```text
/laksa/vio/odom (raw ZED camera odometry)
  -> camera-to-base measurement adapter
  -> /laksa/vio/base_odom
  -> robot_localization EKF
  -> /laksa/odometry/local and odom -> base_footprint
```

It contains no map authority, planner, controller, actuator, or sensor-driver
launch. Physical qualification remains blocked by disconnected hardware.

## Evidence to inspect

Start with:

- `G2_1_PRODUCTION_HARDENING_REPORT.md`
- `G2_1_RESULTS.json`
- `G2_RESTART_AND_TIME_POLICY.md`
- `LOCAL_ODOMETRY_HEALTH_CONTRACT.md`
- `STATE_ESTIMATION_COVARIANCE_POLICY.md`
- `G2_FIRMWARE_VEHICLE_CONTRACT_AUDIT.json`
- `config/ekf_local_odom.yaml`
- `config/zed_local_vio_overlay.yaml`
- `launch/g2_local_estimation.launch.py`

The isolated Jetson Level-B run used ROS 2 Humble
`robot_localization 3.5.4-1jammy.20251017.223119`, test root
`/tmp/laksa-g2-1`, `ROS_DOMAIN_ID=86`, and `ROS_LOCALHOST_ONLY=1`. Its
hash manifest is embedded in `G2_1_RESULTS.json`. It passed 15/15 synthetic
scenarios, the non-zero camera/base transform proof, output-frame proof,
single-TF-authority proof, restart/time, dropout, and NaN/Inf checks. It did
not access hardware, production files, or production services.

## Required audit questions

Classify G2.1 as `PASS`, `PASS_WITH_REQUIRED_CHANGES`, or `FAIL`, after
reviewing:

1. Humble `robot_localization` semantics, especially
   `odom0_relative=false` and `odom0_differential=false`.
2. Raw ZED odometry parent/child frames and the deliberately non-zero
   camera/base transform.
3. The camera-to-base measurement adapter: it must transform measurements,
   publish no TF, and publish no motion command.
4. Restart and estimation-epoch behavior, including time resets.
5. Separation of synthetic covariance/outlier fixtures from production policy.
6. VESC measured-eRPM speed conversion and its source of truth.
7. Firmware geometry divergences still registered as command-path safety debt.
8. Local-odometry health semantics and absence of motion authority.
9. Deferred body-attitude architecture.
10. The critical `DIRECT_ESP32_CMD_VEL_BYPASS` debt: no autonomous command
    path may bypass `drive_supervisor` before G6/G7.
11. Whether G3 may safely depend only on `/laksa/odometry/local` and
    `odom -> base_footprint`.

Historical G1 commits have legacy JetAuto attribution and must not be
rewritten. G2.1 commits use `Leobardo Gomez <luisleo181196@hotmail.com>`.

## Read-only commands

```bash
git checkout architecture/navigation-v2
git show --stat 26151a167d25bbb9595a282a91611226c7545f92
PYTHONPATH=firmware/esp32-s3/jetson/laksa_navigation_v2 \
  python3 -m unittest discover -s firmware/esp32-s3/jetson/laksa_navigation_v2/test -v
```

Do not proceed to G3 implementation until this independent audit is accepted.
