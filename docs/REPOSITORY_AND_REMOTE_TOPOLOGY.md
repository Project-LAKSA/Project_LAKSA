# Repository and remote topology

## Current state

`jetson` is the intended name for the direct deployment checkout. At this
checkpoint it is still named `origin`; no repository script references the Git
remote name, but it is not renamed until a central remote is configured in all
worktrees together.

There is currently **no configured central review remote**. The only known URL
is the Jetson checkout (`ssh://ubuntu@192.168.0.244/...`), which is not the
canonical shared source of truth and must not be used for review publication.

## Target topology

| Remote | Purpose | Allowed flow |
|---|---|---|
| `jetson` | deployment checkout only | explicit deployment fetch/push |
| `review` | personal/private canonical project repository | review, normal V2 upstream |

Branches: `baseline/legacy-recovered-mapping-2026-09-21` is immutable forensic
reference; `recovery/pre-characterization-mapping-planner-good` contains
forensics/handoffs; `architecture/navigation-v2` is active development.

Once a clearly personal/private repository is discovered or explicitly
provided, add it as `review`, push without force, verify every ref with
`ls-remote`, and set only `architecture/navigation-v2` upstream to `review`.
No credentials belong in this file.
