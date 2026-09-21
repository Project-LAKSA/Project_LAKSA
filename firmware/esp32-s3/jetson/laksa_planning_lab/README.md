# PROJECT LAKSA Planning Lab V1.2

This package is an offline, deterministic global-planning laboratory. It reads completed LAKSA occupancy maps, replays the same position-goal scenarios through five planning pipelines, validates every returned path independently, and checkpoints results in SQLite. It has no controller, vehicle-command publisher, sensor dependency, or hardware interface.

## Isolation and operation

Every invocation is rejected unless `ROS_DOMAIN_ID=71` and `ROS_LOCALHOST_ONLY=1`. V1.2 preserves one persistent Nav2 worker per method/configuration, verifies lifecycle/action/map/costmap/TF readiness, and additionally requires the canonical footprint and `base_footprint -> base_link` TF before constrained smoothing. It runs every scenario through that worker and uses `map_server/load_map` only when the map changes.

The map importer scans `/home/ubuntu/laksa_mapping_sessions/*/occupancy/map.yaml`. These are the map-server artifacts already written by `laksa_mapping` through `map_saver_cli`; source session data is never changed. V1 accepts the PGM output produced by that exporter. The generated manifest records the YAML path, session ID, resolution, dimensions, and a SHA-256 over YAML plus image bytes.

If no compatible map exists, the runner exits explicitly instead of substituting a synthetic map.

## Methods

- `HYBRID_PRODUCTION`: deployed Field Lab Smac Hybrid settings, including its built-in smoother.
- `HYBRID_RAW`: identical search with `smooth_path=false`.
- `HYBRID_CONSTRAINED`: raw Hybrid result followed by `ConstrainedSmoother`.
- `LATTICE_CONSERVATIVE`: official Humble Ackermann primitives at the map resolution and 1.09 m radius, with reverse expansion.
- `LATTICE_ASYMMETRIC_FORWARD`: official generator outputs merged deterministically: left primitives use the measured left radius, right primitives use the measured right radius, straight primitives are deduplicated, metadata remains conservative, and reverse expansion is disabled.

Stock Humble `NodeLattice` implements reverse expansion by fetching primitives for the opposite heading and reusing them backwards. That swaps steering-side semantics for an asymmetric vehicle. Therefore V1 records:

`EXACT_ASYMMETRIC_REVERSE_LATTICE_UNSUPPORTED_BY_STOCK_HUMBLE_NODELATTICE`

## Reproducibility

- seed: `2906`
- presets: balanced smoke 8, quick 250, baseline 1000, nightly 5000
- one persisted scenario corpus per preset/seed
- Field Lab `AUTO_HEADING` with conservative radius 1.09 m
- footprint-valid starts and goals; unknown space is invalid
- distance and bearing strata persisted with every scenario
- stable configuration hashes and map SHA-256 in every result key
- SQLite WAL checkpoint after every scenario; reruns resume without duplicates

## Production baseline pinned from Humble

The deployed preview explicitly sets REEDS_SHEPP, radius 1.09, analytic maximum length 5.0, `allow_unknown=false`, tolerance 0, no costmap downsampling, and built-in smoothing. Values omitted by production are pinned to Humble source defaults: reverse 2.0, change 0.0, non-straight 1.2, cost 2.0, retrospective 0.015, analytic ratio 3.5, 72 bins, one-million iterations, 1000 approach iterations, 5 s planning time, 20 m lookup table, and obstacle-heuristic caching off. The built-in smoother defaults are also pinned.

The constrained baseline uses the Humble reference values: curve 30, distance 0, smoothness 2,000,000, cost 0.015, cusp multiplier 3, cusp zone 2.5 m, 70 iterations, gradient tolerance 5000, function tolerance 1e-15, and parameter tolerance 1e-20. LAKSA overrides only the required 1.09 m radius, endpoint orientation retention, reversing, and down/up-sampling factors of one.

## Metrics and ranking

Each result stores planner/smoother/wall time, length, min/mean clearance, reverse length/fraction, direction changes/cusps, self-intersections, cusp-aware Reeds-Shepp curvature evidence, endpoint error, and independent interpolated oriented-rectangle collision evidence. Start/goal generation retains the stricter circumscribed-radius rule; full paths use the measured rectangular footprint.

Ranking is lexicographic: collisions, kinematic violations, failures, success, cusps/intersections, efficiency, then runtime. Unsafe configurations cannot win due to a shorter path.

Nightly optimization uses a seeded standard-library randomized search and successive halving: smoke representatives, 80 coarse configurations over 250 scenarios, 16 semifinalists over 1000, and 5 finalists over all 5000. It is sequential and suitable for `nice`; no optional optimizer framework is required.

## Result files

The default directory is `/home/ubuntu/laksa_planning_lab_results`:

- `planning_lab.sqlite3` (resumable source of truth)
- `map_manifest.yaml`
- `scenarios_<preset>_2906.json`
- `lattices/*.json`
- `runtime/*.yaml`
- `logs/*.log`
- `summary.csv`
- `summary.md`
- `configuration_comparison.csv` (constraint-first tradeoff table)
- `failure_summary.csv`
- `validation.csv` (compact per-scenario validator result)
- `validation.json` (collision and kinematic reproduction evidence)

No result is promoted to production by this package.
