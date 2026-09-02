# LAKSA Ackermann simulation

This dependency-free simulator executes the same
`lidar_rollout_core.AckermannRolloutController` imported by the ROS 2 runtime.
It models the measured 0.324 m wheelbase, asymmetric steering endpoints, three
circle Slash footprint, rear-axle bicycle kinematics, forward-mounted 360-degree
LiDAR, command latency, steering response, sensor noise, collisions, and visited
space.

Run the deterministic parameter sweep and six-scenario comparison:

```bash
python3 jetson/simulation/ackermann_kitchen_sim.py \
  --tune-duration 30 --duration 150 --output /tmp/laksa-simulation
```

Run the noisy 90-degree/dead-end regression:

```bash
python3 -m unittest jetson/simulation/test_ackermann_kitchen_sim.py
```

Reproduce the complete 18-run noisy acceptance campaign:

```bash
python3 jetson/simulation/run_robust_campaign.py \
  --output /tmp/laksa-robust-campaign.json
```

The command exits nonzero unless all 18 runs reach the 98% coverage target,
minimum coverage is at least 98%, and the modeled collision count is zero.

The August 2026 robust campaign used three layouts, two starts, and three noise
seeds per start. It added 12 mm LiDAR noise, 150 ms command latency, a 220 ms
steering time constant, 8 mm pose noise, and 8 mrad heading noise. All 18 runs
reached stable 98% observable-area coverage, with 99.45% mean coverage, 98.75%
minimum coverage, and zero collisions. The difficult 90-degree/dead-end start
completed in 56.1 seconds for all three seeds.

Simulation is a regression gate, not proof of physical safety. The real vehicle
still requires a wheels-up command/sign check followed by a low-speed open-floor
test. Tire slip, moving people, reflective LiDAR returns, battery sag, VESC
faults, and an incorrect physical transform are outside this model.
