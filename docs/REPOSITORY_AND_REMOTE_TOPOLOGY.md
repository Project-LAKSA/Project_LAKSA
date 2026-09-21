# Repository and remote topology

## Current state

`jetson` is the direct deployment checkout. It was renamed from `origin` only
after a repository-wide search confirmed no script or deployment configuration
depended on the old remote name. Its deployment role is unchanged.

`github` is the canonical shared review remote:
`git@github.com:Project-LAKSA/Project_LAKSA.git`. It is intentionally distinct
from the Jetson deployment checkout. The central repository is public, so every
branch proposed for publication must pass the repository public-content audit
before it is pushed.

## Target topology

| Remote | Purpose | Allowed flow |
|---|---|---|
| `jetson` | deployment checkout only | explicit deployment fetch/push |
| `github` | canonical `Project-LAKSA/Project_LAKSA` review repository (public) | review, normal V2 upstream |

Branches: `baseline/legacy-recovered-mapping-2026-09-21` is immutable forensic
reference; `recovery/pre-characterization-mapping-planner-good` contains
forensics/handoffs; `architecture/navigation-v2` is active development.

Push without force, verify every ref with `ls-remote`, and set only
`architecture/navigation-v2` upstream to `github`. No credentials belong in
this file.
