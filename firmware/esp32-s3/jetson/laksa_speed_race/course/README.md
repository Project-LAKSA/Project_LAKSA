# Canonical LAKSA Speed Course

This directory packages the existing, user-approved LAKSA Speed Course
reconstruction for the competition C1 simulator. It is a recovered source
asset, not a new course design.

The authoritative machine representation is
[`canonical/speed_course/geometry.yaml`](canonical/speed_course/geometry.yaml),
whose frozen SHA-256 is recorded in
[`approval.yaml`](canonical/speed_course/approval.yaml) and
[`course_manifest.json`](canonical/speed_course/course_manifest.json). The
authoritative reference image is the recovered Speed-Course crop in
[`source/`](source/); the separately named original combined blueprint was not
available as a local file during recovery.

`scripts/rebuild_speed_course.py` regenerates the derived centerline,
boundaries, occupancy maps, reports, and overlay from the frozen geometry. It
requires the documented offline reconstruction dependencies (`numpy`, `Pillow`,
`PyYAML`, and `scipy`) but does not start ROS, a simulator, or any vehicle
interface.
