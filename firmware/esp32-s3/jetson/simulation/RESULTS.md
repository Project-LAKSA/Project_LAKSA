# Ackermann kitchen simulation results

Date: 2026-08-13

## Acceptance campaign

The ROS runtime and simulator imported the same
`AckermannRolloutController`. The campaign contained three kitchen-like layouts
(central island, 90-degree corner/dead end, and bottleneck), two initial poses
per layout, and three deterministic noise seeds per pose: 18 runs total.

Injected non-ideal behavior:

- 12 mm Gaussian LiDAR range noise
- 150 ms command latency
- 220 ms first-order steering response
- 8 mm odometry position noise
- 8 mrad odometry heading noise
- measured asymmetric steering limits: +0.523 / -0.288 rad
- 0.324 m wheelbase and a conservative three-circle Slash footprint

| Metric | Result |
|---|---:|
| Runs reaching stable 98% coverage | 18 / 18 |
| Mean observable-area coverage | 99.45% |
| Minimum observable-area coverage | 98.75% |
| Collisions | 0 |
| Mean completion time | 33.77 s |
| Total reverse distance | 6.93 m |
| Reverse recoveries | 16 |

Normal island and bottleneck starts completed without reverse. Reverse was used
near the 90-degree/dead-end geometry. The hardest start completed in 56.1 s for
all three noise seeds. Its bounded K-turn completed from odometry displacement
and heading change rather than assuming instantaneous motor/servo response.

## Rejected candidates

The initial sector-centering controller reached 98.31% mean coverage in the
150-second deterministic campaign but accumulated 118.4 seconds blocked and 19
recoveries; one start remained at 89.8% after 150 seconds. It was rejected.

The first timed-rollout controller initially appeared superior during a short
test but degraded during the 150-second campaign to 572.9 seconds blocked and
88 recoveries. It was rejected. Its first K-turn revision passed ideal tests but
failed all three noisy variants of the difficult start because time-based phases
did not account for command latency and steering dynamics. It was rejected.

## Remaining physical validation

This result is evidence about the modeled algorithm, not proof that the physical
vehicle is safe or correctly wired. Deployment remains blocked until the VESC
fault is identified. The required physical sequence is:

1. read and clear the named VESC fault with traction mechanically unloaded;
2. verify forward/reverse and steering signs with driven wheels raised;
3. verify LiDAR front/rear transform using a single known obstacle;
4. perform an emergency-stop and communication-loss test;
5. run a low-speed open-floor trial before entering the kitchen.

Tire slip, moving people, glass/reflective returns, battery sag, VESC thermal or
electrical faults, and incorrect hardware transforms are outside the simulation.
