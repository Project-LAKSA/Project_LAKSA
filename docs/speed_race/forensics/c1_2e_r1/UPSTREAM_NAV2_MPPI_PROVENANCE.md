# Pinned upstream Nav2 MPPI source context

The replay used Nav2 commit
`a097086719c88f781aa59788eca29ac6ca5e56db`, the Humble-compatible source
identified and qualified for C1.2d. The files under `upstream_nav2_mppi/` are
unaltered copies from that commit except that the separate
`instrumentation_nav2.diff` documents the observational replay patch. The
upstream license is preserved as `upstream_nav2_mppi/LICENSE.md`.

The canonical upstream project is `ros-navigation/navigation2`. These local
copies make the causal analysis independent of network access.

## Included source and purpose

| Repository-relative file | Purpose |
| --- | --- |
| `upstream_nav2_mppi/nav2_mppi_controller/src/path_handler.cpp` | Local-plan selection, transform, costmap-bound truncation, `prune_distance`, and destructive global-plan pruning. |
| `upstream_nav2_mppi/nav2_mppi_controller/src/optimizer.cpp` | MPPI prepare/optimize/update flow, controller-frequency/model-dt sequence shift, constraints, and weighted update architecture. |
| `upstream_nav2_mppi/nav2_mppi_controller/src/critic_manager.cpp` | Unmodified critic evaluation order. |
| `upstream_nav2_mppi/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp` | Ackermann minimum-turning-radius constraint. |
| `upstream_nav2_mppi/nav2_mppi_controller/include/nav2_mppi_controller/critic_data.hpp` | Shared path, trajectory, cost, and lazy-cache state passed to critics. |
| `upstream_nav2_mppi/nav2_mppi_controller/include/nav2_mppi_controller/tools/utils.hpp` | `furthest_reached_path_point`, path validity, and closest-path utility semantics. |
| `upstream_nav2_mppi/nav2_mppi_controller/src/critics/*.cpp` | Exact implementations of all eight configured critics. |

## Semantics that must be preserved in analysis

`PathHandler::getGlobalPlanConsideringBoundsInCostmapFrame()` begins at the
nearest path pose within `max_robot_pose_search_dist`, limits the candidate
range by `prune_distance`, transforms points in order, and returns immediately
when a transformed point lies outside the costmap. `transformPath()` then
prunes the retained global plan through the returned lower bound. The observed
eight-point local path can therefore be limited by the first of integrated
prune distance and costmap bounds; it must be diagnosed from source,
configuration, and telemetry rather than attributed to one mechanism by name.

`PathAlignCritic::score()` first applies its enabled/near-goal gate. It then
sets `furthest_reached_path_point` and returns without adding cost when that
index is less than `offset_from_furthest`. In this configuration the offset is
20. An eight-point local path has maximum index 7, but inactivity alone is not
proof that the critic caused the tracking failure.

The optimizer does not choose and execute a single candidate trajectory.
Critics accumulate a cost for every sampled trajectory and MPPI uses a
cost-weighted batch update of the control sequence. The forensic patch records
the minimum-final-cost candidate only as a stable diagnostic projection for
per-critic contribution comparisons. Therefore:

- `diagnostic_candidate_index` is not an executable trajectory identifier;
- `total_cost`, `raw_contribution`, and `weighted_contribution` describe that
  diagnostic candidate;
- causal claims should also use the actual command and state sequence;
- a critic's exact-zero delta may mean a legitimate zero or an early return,
  except where a specific captured reason disambiguates it.

`instrumentation_nav2.diff` shows the complete replay-only source delta. It
copies the cost tensor before each critic, subtracts afterward, and writes
telemetry after the normal critic loop. It neither changes critic order nor
adds random-number calls. Its measured latency overhead is not a valid basis
for estimating the uninstrumented controller's real-time performance; use the
authoritative uninstrumented Trial-1 latency in the report for that question.
