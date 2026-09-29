# C1.2e-R1 forensic evidence

This directory is the canonical, repository-contained input set for the
offline C1.2e causal analysis of the first full Nav2 MPPI Ackermann trial.
Trial 1 stopped at its first off-track event, step 1326. The single authorized
instrumented replay reproduced the normalized dynamic trace exactly and
stopped at the same step.

No new simulation is needed or authorized to analyze this evidence. The CSV
and JSON files here are authoritative; plots or prose summaries are secondary.

## Authoritative identities

- Public Trial-1 baseline: `7e280465090ee98e4470082cd8dff898e30d8377`
- Instrumented replay commit: `f490b8805747f518c69abdb2d76c7efaf8c5c768`
- Nav2 MPPI source: `a097086719c88f781aa59788eca29ac6ca5e56db`
- F1TENTH Gym source: `bdaec1420c3b0f103858d289866d0d4e2e597c30`
- Replay result: `PASS_EXACT`, 1326 steps, first off-track at step 1326

Run the repository-only integrity check from the repository root:

```text
python3 docs/speed_race/forensics/c1_2e_r1/verify_forensics.py
```

The checker uses only the Python standard library and never invokes ROS,
Nav2, Gym, or a simulator.

## Evidence files

- `replay_raw_telemetry.csv`: one row per causal controller/Gym cycle. It
  contains pre- and post-step state, tracking errors, command semantics,
  full-body clearance, safety state, causality stamps, and measured latency.
- `critic_telemetry.csv`: eight rows per evaluation, one for each configured
  critic. Contributions are measured on the diagnostic minimum-total-cost
  candidate. Nav2 MPPI applies a weighted batch update; this candidate is not
  a claim that MPPI chooses one discrete trajectory.
- `path_pipeline_telemetry.csv`: one row per evaluation describing the local
  transformed path and PathAlignCritic activation gate.
- `replay_summary.json`: reproduction verdict and aggregate facts.
- `provenance.json`: dependency SHAs, immutable configuration hashes, seed
  policy, geometry, and passive-instrumentation identity.
- `instrumentation_manifest.txt`: the observational-instrumentation contract.
- `instrumentation_nav2.diff`: exact passive changes applied to the pinned
  Nav2 `critic_manager.cpp` for this replay. It is evidence, not production
  controller code.
- `FORENSIC_MANIFEST.txt`: byte hashes, sizes, and record counts.
- `UPSTREAM_NAV2_MPPI_PROVENANCE.md`: exact upstream source locations and
  semantics needed to interpret the captures.
- `EXTRA_HIGH_ANALYSIS.md`: complete causal-analysis specification.

## Canonical repository context

Interpret the evidence against these files at the replay commit:

- MPPI and critic configuration:
  `firmware/esp32-s3/jetson/laksa_speed_race/config/c1_nav2_mppi.yaml`
- Vehicle and adapter configuration:
  `firmware/esp32-s3/jetson/laksa_speed_race/config/laksa_proxy_v0.yaml`
- Runtime launch wiring:
  `firmware/esp32-s3/jetson/laksa_speed_race/launch/c1_nav2_mppi_three_lap.launch.py`
- Global path publisher:
  `firmware/esp32-s3/jetson/laksa_speed_race/laksa_speed_race/nav2_raceline_node.py`
- Ackermann conversion:
  `firmware/esp32-s3/jetson/laksa_speed_race/laksa_speed_race/nav2_ackermann_adapter_node.py`
- Gym, off-track, and terminal semantics:
  `firmware/esp32-s3/jetson/laksa_speed_race/laksa_speed_race/gym_adapter_node.py`
- Full-body clearance:
  `firmware/esp32-s3/jetson/laksa_speed_race/laksa_speed_race/course_validation.py`
- Three-lap state machine:
  `firmware/esp32-s3/jetson/laksa_speed_race/laksa_speed_race/three_lap_gate.py`
- Metrics and telemetry semantics:
  `firmware/esp32-s3/jetson/laksa_speed_race/laksa_speed_race/metrics.py`
- Lockstep host and independent costmap veto:
  `firmware/esp32-s3/jetson/laksa_speed_race_nav2/src/rpp_lockstep_host.cpp`
- Ackermann feasibility invariant:
  `firmware/esp32-s3/jetson/laksa_speed_race_nav2/include/laksa_speed_race_nav2/ackermann_feasibility.hpp`
- Passive instrumentation generator:
  `firmware/esp32-s3/jetson/laksa_speed_race/docker/instrument_nav2_mppi_forensics.py`
- Evidence assembler and schema construction:
  `firmware/esp32-s3/jetson/laksa_speed_race/tools/c1_2e_forensic_handoff.py`
- Frozen raceline and course contract:
  `firmware/esp32-s3/jetson/laksa_speed_race/course/canonical/speed_course/`
- Full qualification history:
  `docs/speed_race/C1_2D_NAV2_MPPI_ACKERMANN_REPORT.md`

Historical report sections may retain archival runtime paths as evidence of
where a run was executed. They are not dependencies. All files needed for the
analysis are addressed above by repository-relative path.

## Fixed interpretation cautions

1. `diagnostic_candidate_index` is the minimum final-cost sample used for
   observability. Upstream MPPI computes a weighted control update over the
   batch; do not call this index the selected executable trajectory.
2. `INACTIVE_OR_ZERO` means the critic either returned early or contributed
   an exact zero to every sampled trajectory. Only PathAlignCritic has the
   more specific early-return reason captured here.
3. PathAlignCritic being inactive for all 1326 evaluations does not, by
   itself, establish causality. The causal analysis must attempt to falsify
   the proposed objective/path-representation incompatibility.
4. The independent safety veto is a costmap collision veto. Competition
   off-track semantics use the canonical course corridor and full body.
   These boundaries must be compared, not assumed identical.
5. Runtime source-order paths in `provenance.json` are historical provenance,
   not analysis dependencies.

Start with `EXTRA_HIGH_ANALYSIS.md` and do not modify controller parameters or
run another simulation.
