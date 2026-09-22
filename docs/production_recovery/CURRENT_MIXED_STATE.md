# Current mixed LAKSA runtime state

This record preserves the recoverable LAKSA-original source/configuration state
before removal of characterization tooling. It is not a declaration that any
mapping, odometry, planner, or controller configuration is correct.

## Capture provenance

- Capture time: 2026-09-22 (forensic, read-only Jetson inspection).
- Jetson repository: `/home/ubuntu/src/Project_LAKSA`.
- Jetson committed source: `b178a91f3a87f90ae289f62588744bd7936a43f4`
  on `a028-manual-mapping-cockpit`.
- Jetson source state: dirty; 32 tracked modifications and a substantial
  untracked production/configuration tree were captured.
- Branch base: `recovery/pre-characterization-mapping-planner-good`
  (`c9fe6b9487b88753cbae575ac53bef23ec28b2d8`).
- The copied source/configuration excludes `.git`, `build`, `install`, `log`,
  `.pytest_cache`, and Python bytecode.

The first preservation commit is intentionally a source/configuration capture,
including the then-mixed characterization capability. The immediately following
production-only commit removes that active capability while retaining this
commit in history for recovery.

## Deployment divergence observed

`/home/ubuntu/laksa_ws/src` and `/home/ubuntu/laksa_ws/install` were not a
byte-identical mirror of the dirty Jetson repository. In particular,
`laksa_mapping` configuration and launch files in workspace/install matched one
another but differed from the repository capture. The installed entrypoint
scripts matched the captured source scripts exactly. See
`PRODUCTION_SOURCE_DIVERGENCE.json`.

## Characterization boundary

The active mixed tree contains dedicated vehicle-identification, room-scale,
campaign, stage-runner, and supervised-characterization tooling as well as a
lower-priority characterization request path in `drive_supervisor_node.py`.
The forensic boundary supplied for this separation is 2026-09-17. This task
does not decide mapping architecture or reinterpret pre-characterization
runtime behavior.

## Preservation commit

The exact preservation commit is recorded in the final audit metadata after
this commit is created; a Git commit cannot reliably self-identify its own SHA
within its tree.
