# PROJECT LAKSA recovered-baseline review handoff

Review branch: `recovery/pre-characterization-mapping-planner-good`.

Frozen forensic reference: `baseline/legacy-recovered-mapping-2026-09-21` at
`a4c321fc5c1cff3f99e443b24f97f9ac3c2d8bf5`, tagged
`laksa-legacy-recovered-baseline-2026-09-21`.

Central review remote: public GitHub repository
`Project-LAKSA/Project_LAKSA` (`git@github.com:Project-LAKSA/Project_LAKSA.git`)
as remote `github`. `jetson` remains the direct deployment checkout and is not
the shared source of truth. Inspect V2 first, then the frozen baseline and this
recovery branch. Do not modify the frozen baseline.

## Entrypoints

- Production control: `jetson/systemd/laksa-control-navigation.service` ->
  `jetson/scripts/laksa-control-navigation` -> `laksa_bringup/manual_control.launch.py`.
- Recovered mapping: `jetson/laksa_mapping/launch/mapping_stack.launch.py`,
  `config/indoor_live_zed.yaml`, `config/rtabmap_common.yaml`.
- Nav2/planner configuration: `jetson/laksa_bringup/config/nav2_ackermann.yaml`.
- Robot model: `jetson/laksa_description/urdf/`.
- Motion authority: `jetson/laksa_bringup` supervisor and ESP32 micro-ROS
  boundary; Xbox manual control remains highest normal authority.
- Field Lab: `jetson/laksa_dashboard`; observer only.
- Digital twin/planner evidence: `jetson/laksa_planning_lab` and V004 artifacts.

## Trust boundaries

Mapping was recovered and stationary-qualified; five clean resets passed.
Characterization is isolated. Legacy Smac Hybrid is **not** production
qualified: see `PLANNER_INTERNAL_COLLISION_FORENSICS.json` and the recovery
report. Hardware-dependent ZED/TF proof is pending because hardware is absent.
Do not treat planner success, dashboard state, or experimental lab code as
authority to move the vehicle. Review, recommend, and compare against the V2
ADRs; do not modify the frozen baseline.
