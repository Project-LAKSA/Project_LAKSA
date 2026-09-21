"""Offline validation of the LAKSA V2 REP-105 TF contract; no ROS graph needed."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class Edge:
    parent: str
    child: str
    authority: str


@dataclass(frozen=True)
class BodyPose:
    x_m: float
    y_m: float
    z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float


@dataclass(frozen=True)
class PlanarProjection:
    """Option B: planar Nav2 state plus the physical body residual."""

    navigation_xyz_rpy: tuple[float, float, float, float, float, float]
    body_from_navigation_xyz_rpy: tuple[float, float, float, float, float, float]


def project_body_to_base_footprint(body: BodyPose) -> PlanarProjection:
    """Keep Nav2 planar while preserving body roll/pitch and height.

    This is the testable semantic contract for the future dynamic adapter,
    not a substitute for that future ROS TF publisher.
    """
    return PlanarProjection(
        navigation_xyz_rpy=(body.x_m, body.y_m, 0.0, 0.0, 0.0, body.yaw_rad),
        body_from_navigation_xyz_rpy=(0.0, 0.0, body.z_m, body.roll_rad, body.pitch_rad, 0.0),
    )


def validate(edges: Iterable[Edge], root: str = "map", required_children: Iterable[str] = ("zed_camera_link", "lidar_link")) -> list[str]:
    edges = list(edges)
    errors: list[str] = []
    parents: dict[str, str] = {}
    graph: dict[str, list[str]] = {}
    authorities: dict[tuple[str, str], str] = {}
    for edge in edges:
        if not edge.parent or not edge.child or not edge.authority:
            errors.append("empty_edge_field")
            continue
        if edge.parent == edge.child:
            errors.append(f"self_loop:{edge.parent}")
        if edge.child in parents:
            errors.append(f"duplicate_child:{edge.child}")
        parents[edge.child] = edge.parent
        key = (edge.parent, edge.child)
        if key in authorities and authorities[key] != edge.authority:
            errors.append(f"duplicate_authority:{edge.parent}->{edge.child}")
        authorities[key] = edge.authority
        graph.setdefault(edge.parent, []).append(edge.child)
    if root in parents:
        errors.append(f"wrong_root:{root}")
    if root == "zed_camera_link":
        errors.append("camera_as_vehicle_root")
    seen: set[str] = set()
    active: set[str] = set()

    def visit(node: str) -> None:
        if node in active:
            errors.append(f"loop:{node}")
            return
        if node in seen:
            return
        seen.add(node)
        active.add(node)
        for child in graph.get(node, []):
            visit(child)
        active.remove(node)

    visit(root)
    for edge in edges:
        if edge.parent not in seen or edge.child not in seen:
            errors.append(f"disconnected:{edge.parent}->{edge.child}")
    for child in required_children:
        if child not in seen:
            errors.append(f"missing_sensor_edge:{child}")
    return errors


def validate_authority_contract(contract: dict) -> list[str]:
    errors: list[str] = []
    edges = [Edge(item["parent"], item["child"], item["authority"]) for item in contract.get("edges", [])]
    errors.extend(validate(edges, root=contract.get("root", "")))
    expected = "OPTION_B_ODOM_TO_BASE_FOOTPRINT_PLANAR__BASE_FOOTPRINT_TO_BASE_LINK_DYNAMIC_BODY_ATTITUDE"
    if contract.get("base_frame_architecture") != expected:
        errors.append("wrong_base_frame_architecture")
    required = {
        ("map", "odom"): "GLOBAL_LOCALIZATION_MODE_SELECTOR",
        ("odom", "base_footprint"): "LOCAL_STATE_ESTIMATOR",
        ("base_footprint", "base_link"): "BODY_ATTITUDE_PROJECTION_ADAPTER",
        ("base_link", "zed_camera_link"): "robot_state_publisher",
        ("base_link", "lidar_link"): "robot_state_publisher",
    }
    actual = {(edge.parent, edge.child): edge.authority for edge in edges}
    for key, authority in required.items():
        if actual.get(key) != authority:
            errors.append(f"wrong_authority:{key[0]}->{key[1]}")
    modes = contract.get("mode_specific_global_authority", {})
    if modes.get("three_d_reconstruction") != "OBSERVATIONAL_NO_NAV2_MAP_TO_ODOM_AUTHORITY":
        errors.append("three_d_reconstruction_authority_violation")
    if "base_link->imu_link" not in contract.get("unmeasured_edges", []):
        errors.append("invented_or_missing_imu_contract")
    return errors
