# C1 runtime qualification report

## 1. Executive result

`C1_STATUS=FAIL`

The actual closed loop executed on ROS 2 Humble/aarch64, with Waterloo Pure
Pursuit commanding the pinned F1TENTH Gym KS model through C1-scoped topics.
The frozen raceline is geometrically incompatible with the unchanged Gym
collision contract: its ideal minimum body clearance is 4.02 mm while Gym's
collision threshold is 5 mm before rasterization. No run completed lap one.

## 2. Environment and isolation

- Jetson: Ubuntu 22.04.5 LTS, aarch64, ROS 2 Humble
- Workspace: `/tmp/laksa-c1-native`
- Source checkout: `/tmp/laksa-c1-runtime-a8e65c9`
- ROS domain: 221, localhost only
- Production overlays: none
- C1 nodes observed: `/c1_gym_adapter`, `/pure_pursuit`, TF listener
- C1 topics observed: `/c1/odom`, `/c1/drive_request`, waypoint markers,
  `/tf`, `/tf_static`, parameter events, and rosout
- Forbidden physical command topics observed: none
- micro-ROS, VESC, GPIO, production drive supervisor in C1 graph: none

The production source checkout remained clean. Production workspaces and
services were neither built nor changed.

## 3. Build and tests

Fresh explicit-root build:

```text
colcon build --base-paths src/laksa_speed_race src/pure_pursuit \
  --symlink-install --build-base build-final2 --install-base install-final2 \
  --packages-select pure_pursuit laksa_speed_race
```

Result: PASS. `colcon test`: 30 tests, zero errors/failures/skips. Upstream
Pure Pursuit produced compiler warnings but controller mathematics were not
changed.

## 4. Course, raceline, and proxy

- course source: `222c897f6835a4877318cfd0ac7d76be98b7173faaeec44e028814ca3c6eb13e`
- geometry: `2c76075f838a7a1c3e0891385b27f2e6f26e641ad13068280093fda273a84858`
- raceline: `5082057770360d4955195d66b2587102c03b7bea7f3c57da50ad957766f3a7f5`
- proxy: `b9d072d92db353b0763fada60c40a85cdb82ce2b92b7c3c982652d3ef6719b76`
- model/dt/seed: KS / 0.01 s / 12345
- steering limit: 0.288 rad
- speed limit: 1.0 m/s

Raceline regeneration was byte-deterministic on ARM64. Course geometry was
not modified.

## 5. Trial evidence

The frozen initial controller run failed `off_track` at 0.78 simulated seconds
(78 steps). After selecting the canonical 0.02 m raster, controlled trials
failed around the first turnaround:

| Configuration | Steps | Sim s | Terminal fault | CTE RMS m | CTE p95 m |
|---|---:|---:|---|---:|---:|
| lookahead 0.8, Kp 0.5 | 1549 | 15.49 | collision | 0.06163 | 0.17006 |
| lookahead 0.9, Kp 0.5 | 2014 | 20.14 | collision | 0.05692 | 0.12658 |
| lookahead 0.95, Kp 0.5 | 2009 | 20.09 | collision | 0.05835 | 0.13653 |
| lookahead 0.965, Kp 0.5 | 2007 | 20.07 | collision | 0.05874 | 0.13815 |
| lookahead 0.9725, Kp 0.5 | 2006 | 20.06 | collision | 0.05908 | 0.14102 |
| lookahead 0.97625, Kp 0.5 | 2006 | 20.06 | collision | 0.05935 | 0.14285 |
| lookahead 0.978125, Kp 0.5 | 2005 | 20.05 | collision | 0.05945 | 0.14295 |
| lookahead 0.98, Kp 0.5 | 2003 | 20.03 | off_track | 0.05952 | 0.14432 |
| lookahead 1.0, Kp 0.5 | 2000 | 20.00 | off_track | 0.06007 | 0.14716 |
| lookahead 1.0, Kp 0.495 | 2000 | 20.00 | off_track | 0.06013 | 0.14742 |
| lookahead 1.0, Kp 0.49 | 1547 | 15.47 | collision | 0.06637 | 0.19620 |
| lookahead 1.0, speed 0.8 | 2495 | 24.95 | off_track | 0.05884 | 0.13980 |

Additional `K_p=0.4`, `0.45`, `0.48`, and `0.6` trials also failed with
collision or off-track. Complete machine-readable summaries remain in the
isolated Jetson result directory `/tmp/laksa-c1-results`. A private copy of
the 18 complete result sets (93 files, including every `summary.json`,
`trajectory.csv`, `commands.csv`, and `events.csv`) is preserved at
`/private/tmp/laksa-c1-runtime-evidence-a8e65c9`; large trajectories and
console logs were not committed.

## 6. Terminal and shutdown evidence

After the terminal-evidence and executor fixes, every evidenced run published
and recorded `(steering=0, speed=0)`, executed zero simulation steps after the
terminal transition, stopped both C1 processes, and left zero C1 orphans.

## 7. Determinism

Static artifacts regenerate deterministically. Three successful official runs
could not be attempted because Trial 1 never met acceptance. Repeating failed
parameter sets produced the same failure region and terminal class; this is
not promoted to a successful determinism result.

## 8. Production integrity

- physical hardware touched: false
- physical GPIO touched: false
- production source modified: false
- production workspace built: false
- production services modified: false

## 9. Final verdict and C2 readiness

C1 fails. C2 is not authorized or ready. The next work must remain C1-scoped:
review and regenerate the derived raceline with an explicit simulator-aware
full-body clearance, preserving canonical course geometry and all hard safety
limits, then rerun this qualification from Trial 1.
