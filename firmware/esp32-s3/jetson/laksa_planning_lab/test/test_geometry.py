import math

from laksa_planning_lab.geometry import (KAPPA_LEFT_MAX, KAPPA_RIGHT_ABS_MAX, R_LEFT_M, R_RIGHT_M,
                                         auto_heading, reeds_shepp_kinematic_report, signed_curvatures,
                                         signed_segment_lengths, stable_hash, wrap_pi)
from laksa_planning_lab.metrics import direction_changes, path_length, self_intersections


def test_measured_radii_are_not_averaged():
    assert math.isclose(R_LEFT_M, 0.324 / math.tan(0.523))
    assert math.isclose(R_RIGHT_M, 0.324 / math.tan(0.288))
    assert R_LEFT_M < R_RIGHT_M


def test_wrap_and_auto_heading():
    assert -math.pi <= wrap_pi(99.0) < math.pi
    yaw = auto_heading(0, 0, 0, 0, 0.545)
    assert math.isclose(yaw, 0.5, abs_tol=1e-9)


def test_direction_cusps_length_and_curvature():
    poses = [(0, 0, 0), (1, 0, 0), (0.5, 0, 0)]
    assert signed_segment_lengths(poses) == [1.0, -0.5]
    assert direction_changes(poses) == 1
    assert path_length(poses) == 1.5
    left = [(0, 0, 0), (1, 0, 0.2)]
    assert signed_curvatures(left)[0] > 0
    assert KAPPA_LEFT_MAX > KAPPA_RIGHT_ABS_MAX


def test_self_intersection_and_hash():
    assert self_intersections([(0, 0, 0), (1, 1, 0), (0, 1, 0), (1, 0, 0)]) == 1
    assert stable_hash({"b": 2, "a": 1}) == stable_hash({"a": 1, "b": 2})


def _arc(radius, samples=21):
    return [
        (radius * math.sin(theta), radius * (1.0 - math.cos(theta)), theta)
        for theta in (index * 0.6 / (samples - 1) for index in range(samples))
    ]


def test_reeds_shepp_minimum_radius_boundary_and_larger_radius_pass():
    boundary = reeds_shepp_kinematic_report(_arc(1.09))
    larger = reeds_shepp_kinematic_report(_arc(1.5))
    assert boundary["kinematically_feasible"]
    assert larger["kinematically_feasible"]
    assert math.isclose(boundary["min_implied_radius_m"], 1.09, rel_tol=1e-6)


def test_reeds_shepp_clearly_tight_arc_fails_with_evidence():
    report = reeds_shepp_kinematic_report(_arc(0.75))
    assert not report["kinematically_feasible"]
    assert report["kinematic_violation_pose_index"] is not None
    assert report["signed_direction"] == 1


def test_reeds_shepp_straight_cusp_wrap_duplicates_and_short_noise():
    straight = [(0.0, 0.0, math.pi - 1e-6), (-1.0, 0.0, -math.pi + 1e-6), (-2.0, 0.0, math.pi - 1e-6)]
    assert reeds_shepp_kinematic_report(straight)["kinematically_feasible"]
    cusp = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.5, 0.0, 0.0)]
    cusp_report = reeds_shepp_kinematic_report(cusp)
    assert cusp_report["cusps"] == 1
    assert cusp_report["max_abs_curvature_1pm"] == 0.0
    noisy = [(0.0, 0.0, 0.0), (0.0002, 0.0001, 0.2), (1.0, 0.0, 0.0), (1.0, 0.0, 1.0), (2.0, 0.0, 0.0)]
    assert reeds_shepp_kinematic_report(noisy)["kinematically_feasible"]
