# ADR-005: actuation and authority

Retain the ESP32 safety boundary and existing supervisor as the sole physical
command bridge during V2. V2 evaluates `ros2_control` with the
`bicycle_steering_controller`, because LAKSA exposes one traction command and
one steering command rather than independent left/right wheel actuation.
`ackermann_steering_controller` is rejected unless real joint interfaces are
added.

No controller manager may bypass Xbox/manual priority, ESP32 watchdog, or
emergency stop. ros2_control is an adapter experiment first; the supervisor
remains authority arbiter. Rollback is the current low-level adapter.
