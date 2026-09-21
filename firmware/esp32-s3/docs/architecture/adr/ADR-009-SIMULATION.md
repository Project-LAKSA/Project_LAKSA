# ADR-009: simulation parity

Use Nav2 Loopback Simulator for fast deterministic CI and modern Gazebo/ros_gz
for dynamic V004 twin qualification. Both consume the canonical robot contract
and V2 Nav2/BT/validator/controller parameters. Only sensors, clock, and
hardware interface differ. V004 keeps measured values and explicit uncertainty;
it is not claimed as physically final. No simulation node may publish to the
physical authority domain.
