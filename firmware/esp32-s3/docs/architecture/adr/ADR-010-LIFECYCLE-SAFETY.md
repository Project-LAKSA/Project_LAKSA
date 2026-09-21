# ADR-010: explicit lifecycle and autonomy state

Nav2 Lifecycle Manager controls navigation servers; a V2 health coordinator
publishes a transparent state machine rather than hidden launch ordering.
`BOOT`, `SENSOR_WAIT`, `LOCALIZATION_READY`, `MAP_READY`, `PATH_READY`,
`AUTONOMY_ARMED`, `EXECUTING`, `DEGRADED`, `FAULT`, and `E_STOP` have explicit
entry predicates. Invalid/stale inputs transition to zero autonomous command
and `DEGRADED`; only manual authority may remain when hardware safety permits.
Rollback stops V2 lifecycle nodes and restores the frozen baseline.
