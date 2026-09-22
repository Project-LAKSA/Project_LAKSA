# V2 migration plan

| Phase | Deliverable | Required evidence | Status |
|---|---|---|---|
| 0 | frozen forensic baseline | tag + hashes + clean worktree | complete |
| G1 | canonical vehicle / generated geometry / REP-105 TF | schema, deterministic artifacts, URDF, negative TF and ramp tests | complete offline |
| G2.1 | production-hardening local state estimation | raw-frame, restart, time-jump, measurement-scale, and health contracts | complete offline pending hardware Level C |
| G3 | 2D mapping + localization comparison; RTAB isolated as 3D | replay metrics, one map->odom authority | planned |
| G4 | layered perception + costmaps | fixture maps and boundary tests | planned |
| G5 | State Lattice planner qualification | 105+ deterministic cases, zero collision escapes | planned |
| G6 | controller qualification: RPP baseline, MPPI Ackermann candidate | loopback/V004 metrics | planned |
| G7 | lifecycle / BT / Collision Monitor / safety | transition/failure tests | planned |
| G8 | Loopback + Gazebo V004 closed loop | same-stack launch test | planned |
| G9 | hardware stationary | BLOCKED_BY_HARDWARE until connected | blocked |
| G10 | manual mapping validation | explicit operator gate | planned |
| G11 | low-speed autonomous validation | explicit operator gate | planned |
| G12 | performance qualification | explicit operator gate | planned |

No phase authorizes vehicle motion. Production migration begins only after
Levels 0-5 evidence and an explicit hardware validation gate.
