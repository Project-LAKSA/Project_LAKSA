"""Offline REP-105 TF contract validation; no ROS graph or hardware needed."""
from dataclasses import dataclass
from typing import Iterable

@dataclass(frozen=True)
class Edge:
    parent: str
    child: str
    authority: str

def validate(edges: Iterable[Edge], root: str = "base_link") -> list[str]:
    edges = list(edges); errors: list[str] = []; parents = {}; graph = {}
    for edge in edges:
        if edge.parent == edge.child: errors.append(f"self_loop:{edge.parent}")
        if edge.child in parents: errors.append(f"duplicate_child:{edge.child}")
        parents[edge.child] = edge.parent; graph.setdefault(edge.parent, []).append(edge.child)
    if root in parents: errors.append(f"wrong_root:{root}")
    seen, active = set(), set()
    def visit(node):
        if node in active: errors.append(f"loop:{node}"); return
        if node in seen: return
        seen.add(node); active.add(node)
        for child in graph.get(node, []): visit(child)
        active.remove(node)
    visit(root)
    for required in ("zed_camera_link", "lidar_link"):
        if required not in seen: errors.append(f"missing_sensor_edge:{required}")
    return errors
