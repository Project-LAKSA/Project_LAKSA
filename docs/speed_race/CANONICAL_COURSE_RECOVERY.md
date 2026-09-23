# Canonical Speed Course recovery

## Result

`COURSE_GEOMETRY_CONFLICT=NONE`. The canonical Speed Course was recovered from
pre-existing LAKSA artifacts that were absent from the starting competition
branch but present in the project working tree. It is now versioned in
`firmware/esp32-s3/jetson/laksa_speed_race/course/canonical/speed_course/`.

The recovered geometry is not a new approximation: its frozen
`geometry.yaml` SHA-256 is
`2c76075f838a7a1c3e0891385b27f2e6f26e641ad13068280093fda273a84858` and
the existing approval record identifies it as `CANONICAL_APPROVED` after user
visual overlay review.

## Source hierarchy and availability

1. The competition blueprint named **Obstacle and Speed Course Blueprints.png**
   is the authority. A file with that exact combined-image name was not found
   in project-accessible storage: `RAW_BLUEPRINT_LOCAL_COPY=UNAVAILABLE`.
2. `course/source/speed_course_full.jpeg` is an existing LAKSA Speed-Course
   crop derived from the user-supplied blueprint and is retained as the global
   geometry authority. SHA-256:
   `222c897f6835a4877318cfd0ac7d76be98b7173faaeec44e028814ca3c6eb13e`.
3. `speed_course_left_detail.jpeg` and `speed_course_right_detail.jpeg` are
   retained as detailed radius-label references. Their SHA-256 values are in
   `course_manifest.json`.
4. The recovered metric construction, boundaries, centerline and raster maps
   are deterministic derivatives of that source and explicit dimensions.

`EXISTING_DERIVED_GEOMETRY=AVAILABLE`.

## Recovered artifacts

| Recovered artifact | Purpose | Units/frame | Usability |
|---|---|---|---|
| `geometry.yaml` | Frozen analytic centreline primitives and course dimensions | metres, `speed_course_map` | Authoritative geometry |
| `boundaries.geojson` | Left/right physical course boundaries | metres, `speed_course_map` | Direct simulator/validator input |
| `centerline.csv` / `.geojson` | Closed, directed centreline with clearance and curvature | metres/radians | Raceline optimizer input |
| `mission.yaml` / `checkpoints.yaml` | Directed Start/Finish and ordered 32-gate lap definition | metres/radians | Three-lap mission contract |
| `speed_course_nav2.png` / `.yaml` | 0.05 m occupancy export | metres, `speed_course_map` | Gym map asset |
| `speed_course_hires.png` / `.yaml` | 0.02 m validation occupancy export | metres, `speed_course_map` | Validation asset |
| `rebuild.py` plus wrapper | Existing deterministic LAKSA generator | offline only | Reproducible generation |
| `validation_report.json` / overlay | Existing dimension, topology and source-overlay evidence | metres | Audit evidence |

The historical local artifact directory was untracked, so it has no commit
SHA. Its recovery provenance is `PREEXISTING_LAKSA_WORKTREE_ARTIFACT`; the
exact content hashes above bind the imported asset set.

The recovered `centerline.csv` retains its original CRLF record endings so its
approved byte hash remains verifiable. A path-specific `.gitattributes` rule
classifies those preserved endings correctly for `git diff --check`; it does
not normalize or alter the course data.

## Blueprint agreement

The recovered construction explicitly encodes the blueprint dimensions:

- overall length: 135 ft = 41.148 m;
- overall width: 47 ft = 14.3256 m;
- path width: 36 in = 0.9144 m;
- directed parallel straights, opposite travel directions, and connected end
  lobes;
- Start/Finish at `[20.62863090812533, 2.2986877548722693]`, with `+X`
  crossing direction.

The generator validates 24 labelled radii/geometry annotations. The smallest
recovered centreline radius is 1.8288 m (6 ft), which is above the conservative
LAKSA proxy minimum turning radius of 1.093722637 m.

## Metadata note

`geometry.yaml` retains the pre-approval literal
`reconstruction_status: USER_VALIDATION_REQUIRED`; this is not a geometry
conflict. `approval.yaml`, `source_metadata.yaml`, `mission.yaml`, and the
regenerated `validation_report.json` bind the exact unchanged geometry hash to
`CANONICAL_APPROVED`. The importer preserves that historical geometry file
unchanged and uses `approval.yaml` as the approval authority.

## Determinism and safety

`course/scripts/rebuild_speed_course.py` regenerates only the Speed Course;
it does not start ROS, Gym, a controller, or any physical command path.
The recovered generator was run twice in the existing LAKSA offline tool
environment and produced identical output hashes. The course package validator
also checks source/generated hashes, dimensions, closed boundaries/centreline,
vector/raster agreement, Start/Finish direction and conservative curvature.
