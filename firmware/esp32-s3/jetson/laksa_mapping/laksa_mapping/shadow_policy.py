"""Pure policy helpers for the explicit A030A mapping-source selector."""

ZED_ONLY = "ZED_ONLY"
HYBRID_SHADOW = "HYBRID_SHADOW"
MAPPING_SOURCES = (ZED_ONLY, HYBRID_SHADOW)


def hybrid_ready(source: str, lidar_health: str, scan_age_sec, maximum_age_sec: float) -> bool:
    """Return whether an explicitly selected hybrid session may use real scans."""
    if source != HYBRID_SHADOW or lidar_health not in ("GOOD", "DEGRADED"):
        return False
    return scan_age_sec is not None and 0.0 <= float(scan_age_sec) <= float(maximum_age_sec)


def output_prefix(source: str) -> str:
    return "/laksa/mapping_shadow" if source == HYBRID_SHADOW else "/zed_rtabmap"
