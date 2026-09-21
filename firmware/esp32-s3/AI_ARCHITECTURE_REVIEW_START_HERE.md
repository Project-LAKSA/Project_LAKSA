# AI architecture review: start here

Review the frozen `baseline/legacy-recovered-mapping-2026-09-21` first. It is
a forensic reference, not an autonomous release. Read the recovery reports,
`JETSON_ARCHITECTURE_MANIFEST.md`, mapping launch/config, systemd units,
supervisor, ESP32 interface, Field Lab, planning lab, and V004 artifacts.

Audit package boundaries, launch/lifecycle topology, TF authority, topics and
actions, deployment, safety authority, mapping/localization, planner/controller
assumptions, tests, duplicated configuration, race conditions, and dead code.
Do not modify the frozen baseline. Compare recommendations to V2 ADRs and cite
concrete files and upstream evidence. Treat legacy Smac paths as unqualified;
do not infer execution safety from planner success.

Central review publication is currently blocked pending a clearly personal,
private project remote. When that remote is configured, inspect
`architecture/navigation-v2` first, then the frozen baseline and recovery
branch; never modify the frozen baseline.
