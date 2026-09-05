# A028 Manual Mapping Cockpit

The cockpit is a mapping-only application. It has no publishers or clients for
`/laksa/command`, `/laksa/brake`, `cmd_vel`, Nav2 actions, throttle, or steering.
The existing Xbox → drive supervisor → ESP32 path remains the only manual motion
path.

## Runtime

- Dashboard: `http://<jetson-ip>:8090`
- Camera preview: `web_video_server` on port `8080`
- Mapping state: `/laksa/mapping/state`
- Start: `/laksa/mapping/start`
- Stop: `/laksa/mapping/stop`
- Health: `/laksa/health/summary` and `/diagnostics`
- Normalized battery: `/laksa/battery_state`

Each session is isolated under `/home/ubuntu/laksa_mapping_sessions/<UTC stamp>`.
The manager owns its launch process group, uses bounded graceful shutdown, saves
the live occupancy map, checks the SQLite database, exports a PLY, and writes
metadata. Optional SVO recording uses the ZED wrapper services.

## Geometry boundary

The `0.57 × 0.36 × 0.30 m` chassis is a provisional visualization envelope.
It is forbidden as an implicit source for navigation, collision, Ackermann, or
simulation geometry. The BNO08X is not rendered because its extrinsic is unknown.

## Mapping profiles

- `indoor_live`: validated C1 GEN_3 + NEURAL_LIGHT effective configuration.
- `indoor_high_quality`: validated C3 GEN_3 + NEURAL effective configuration.
- Outdoor profiles are explicitly provisional and require an outdoor SVO.
