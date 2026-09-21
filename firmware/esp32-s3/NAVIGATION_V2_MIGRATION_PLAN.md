# V2 migration plan

| Phase | Deliverable | Required evidence | Status |
|---|---|---|---|
| 0 | frozen forensic baseline | tag + hashes + clean worktree | complete |
| 1 | canonical vehicle contract | schema/unit test, generated footprint | started |
| 2 | REP-105 TF contract | URDF/TF static test | planned |
| 3 | estimator interfaces | bag/fixture covariance test | planned |
| 4 | mapping/localization comparison | replay metrics, one map->odom authority | planned |
| 5 | layered costmaps | fixture maps and boundary tests | planned |
| 6 | planner comparison | 105+ deterministic cases, zero collision escapes | planned |
| 7 | validator gate | rejected paths cannot reach controller | planned |
| 8 | controller comparison | loopback/V004 metrics | planned |
| 9 | actuation adapter decision | interface + supervisor-preemption test | planned |
| 10 | BT/lifecycle | transition/failure tests | planned |
| 11 | Gazebo/ros_gz twin | same-stack launch test | planned |
| 12 | hardware stationary | BLOCKED_BY_HARDWARE until connected | blocked |

No phase authorizes vehicle motion. Production migration begins only after
Levels 0-5 evidence and an explicit hardware validation gate.
