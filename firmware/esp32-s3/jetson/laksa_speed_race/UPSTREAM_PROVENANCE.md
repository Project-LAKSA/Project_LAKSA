# Competition upstream provenance

All third-party components remain external dependencies. LAKSA does not copy
or modify their algorithms. The refs below were resolved on 2026-09-22.

| Component | Pinned ref | License | Role | Integration status |
|---|---|---|---|---|
| [f1tenth_system](https://github.com/f1tenth/f1tenth_system) `humble-devel` | `94cb8d7fb5439315316bf80aadbc7256b80cb4e2` | MIT | Hardware-topic and Ackermann interface reference only | Reference; no joystick/teleop imported |
| [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) `dev-humble` | `08395766c4d9dc5a763381f1dd6fa4a3d68df66e` | MIT | Primary ROS 2 Humble simulator | Selected, blocked on custom-vehicle configuration interface |
| [f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) `dev-humble` | `bdaec1420c3b0f103858d289866d0d4e2e597c30` | MIT | Simulator dynamics and LiDAR engine | Selected through Gym ROS |
| [f1tenth/particle_filter](https://github.com/f1tenth/particle_filter) `humble-devel` | `ec599a1c4f3d4edc5f7d356e2f2990839e56bb7d` | No SPDX license declared | Frozen-map localization candidate | External-only; legal approval/attribution required before distribution |
| [f1tenth/range_libc](https://github.com/f1tenth/range_libc) `humble-devel` | `f55480a7044367c219745480b436b8b7117f8282` | NOASSERTION | PF ray-casting dependency | External-only; license must be resolved before distribution |
| [CL2-UWaterloo/f1tenth_ws](https://github.com/CL2-UWaterloo/f1tenth_ws) `main` | `c20cf63d04b9841ffdb6b2f963bd737d78074136` | MIT | Pure Pursuit reference and launch/interface reference | Selected Pure Pursuit candidate; external source only |
| [CL2-UWaterloo/Raceline-Optimization](https://github.com/CL2-UWaterloo/Raceline-Optimization) `master` | `9290c5d503462e46f7e3e9033002e7ddf165ba7b` | LGPL-3.0 | TUM-derived minimum-curvature optimization | External tool only; never copied into LAKSA |
| [RhythmChandak/F1Tenth-follow-the-gap](https://github.com/RhythmChandak/F1Tenth-follow-the-gap) `main` | `60800ac363480b7b07f13a2ff9aacd504f76d074` | MIT | Mapping traversal candidate; Python node uses `LaserScan` -> `AckermannDriveStamped` | Candidate selected pending a Humble build/run |

## Rejected candidates

`Hamza-cpp/f1tenth_lab4_follow_the_gap` at
`226089379960ce7138d39acc626ff4feab9ee6c1` is an MIT lab skeleton with
unimplemented callbacks. `CL2-UWaterloo/f1tenth_ws` documents its own
`gap_follow` as incomplete. Neither is a valid source for a first-pass mapping
traversal under the no-new-algorithm rule.

`Ryan19941212/F1tenth` at `f2ffe5578effdc80bd5e27ff4bc4dc5be8ea77ca` is a
useful architecture reference, but the GitHub metadata did not provide a
license. It is not copied or included as a dependency.

## Verified Gym ROS interface

At the selected ref, Gym ROS publishes `/scan`, `/ego_racecar/odom`, and
`/ego_racecar/collision`, and accepts `ackermann_msgs/AckermannDriveStamped`
on `/drive`. Its bundled keyboard teleop is disabled by setting
`kb_teleop: false`; it is not a LAKSA competition dependency.
