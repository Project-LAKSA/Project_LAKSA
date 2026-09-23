"""Minimal policy contract for the single production fused mapping source.

No historical blob for this module remains reachable.  This reconstruction is
therefore intentionally limited to the public A046 tests and session-manager
callers that define the preserved fused mapping contract.
"""

FUSED_MAPPING = "FUSED_MAPPING"
MAPPING_SOURCES = (FUSED_MAPPING,)


def fused_ready(lidar_health: str, scan_age_sec, maximum_age_sec: float) -> bool:
    """Return whether the required validated LaserScan is safe to consume."""
    if lidar_health != "GOOD" or scan_age_sec is None:
        return False
    try:
        age = float(scan_age_sec)
        maximum_age = float(maximum_age_sec)
    except (TypeError, ValueError):
        return False
    return 0.0 <= age <= maximum_age


def output_prefix(source: str = FUSED_MAPPING) -> str:
    """Return the canonical namespace for the only permitted mapping source."""
    if source != FUSED_MAPPING:
        raise ValueError(f"unsupported mapping source: {source}")
    return "/laksa/fused_mapping"


def output_map_topic(source: str = FUSED_MAPPING) -> str:
    """Return the canonical occupancy-map topic for fused mapping."""
    if source != FUSED_MAPPING:
        raise ValueError(f"unsupported mapping source: {source}")
    return "/map"
