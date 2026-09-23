"""Pure, deterministic health contract for the preserved fused mapping stack.

No historical blob for this module remains reachable.  The behavior below is
deliberately constrained to the existing A046 health tests and the current
mapping-session manager; it is not a generalized health framework.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class HealthThresholds:
    input_stale_sec: float = 5.0
    rtab_stale_sec: float = 6.0
    activity_recent_sec: float = 5.0
    startup_grace_sec: float = 45.0
    hard_failure_sec: float = 15.0


def activity_state(*, available: bool, content_age_sec, pipeline_healthy: bool,
                   stationary: bool, threshold_sec: float) -> str:
    """Classify map/cloud content without mistaking stationary quiescence for failure."""
    if not available:
        return "UNAVAILABLE"
    if content_age_sec is not None and float(content_age_sec) <= float(threshold_sec):
        return "UPDATING"
    if pipeline_healthy and stationary:
        return "QUIESCENT"
    return "STALE"


def _age(ages: dict, key: str):
    value = ages.get(key)
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _tf_available(tf_health: dict) -> bool:
    return all(bool(tf_health.get(edge, {}).get("available"))
               for edge in ("map_to_odom", "odom_to_base"))


def _result(fusion_state: str, reason_code: str, *, map_activity: str,
            cloud_activity: str, rtab_state: str) -> dict:
    return {
        "fusion_state": fusion_state,
        "reason_code": reason_code,
        "map_activity": map_activity,
        "cloud_activity": cloud_activity,
        "rtab_state": rtab_state,
    }


def derive_fused_health(*, active: bool, startup: bool, elapsed_sec: float,
                        ages: dict, lidar_health: str, tf_health: dict,
                        process_alive: bool, map_available: bool,
                        cloud_available: bool, stationary: bool,
                        thresholds: HealthThresholds | None = None) -> dict:
    """Derive fail-closed fused-mapping health from existing runtime signals."""
    limits = thresholds or HealthThresholds()
    required = ("rgbd", "fused_odom", "scan")
    rtab_age = _age(ages, "rtab_processed")
    pipeline_healthy = (
        active and process_alive and lidar_health == "GOOD" and
        _tf_available(tf_health) and
        all(_age(ages, key) is not None and _age(ages, key) <= limits.input_stale_sec
            for key in required) and
        rtab_age is not None and rtab_age <= limits.rtab_stale_sec
    )
    map_activity = activity_state(
        available=map_available, content_age_sec=_age(ages, "map_content"),
        pipeline_healthy=pipeline_healthy, stationary=stationary,
        threshold_sec=limits.activity_recent_sec,
    )
    cloud_activity = activity_state(
        available=cloud_available, content_age_sec=_age(ages, "cloud_content"),
        pipeline_healthy=pipeline_healthy, stationary=stationary,
        threshold_sec=limits.activity_recent_sec,
    )
    rtab_state = "PROCESSING" if rtab_age is not None and rtab_age <= limits.rtab_stale_sec else "STALLED"

    if not active:
        return _result("FUSED_IDLE", "INACTIVE", map_activity=map_activity,
                       cloud_activity=cloud_activity, rtab_state=rtab_state)
    if startup and float(elapsed_sec) <= limits.startup_grace_sec:
        return _result("FUSED_STARTING", "STARTUP_GRACE", map_activity=map_activity,
                       cloud_activity=cloud_activity, rtab_state=rtab_state)
    if not process_alive:
        return _result("FUSED_ERROR", "RTAB_PROCESS_STOPPED", map_activity=map_activity,
                       cloud_activity=cloud_activity, rtab_state=rtab_state)
    if lidar_health != "GOOD":
        return _result("FUSED_DEGRADED", "LIDAR_BAD", map_activity=map_activity,
                       cloud_activity=cloud_activity, rtab_state=rtab_state)
    if not _tf_available(tf_health):
        return _result("FUSED_DEGRADED", "TF_UNAVAILABLE", map_activity=map_activity,
                       cloud_activity=cloud_activity, rtab_state=rtab_state)

    stale_reasons = (("rgbd", "RGBD_STALE"), ("fused_odom", "ODOMETRY_STALE"),
                     ("scan", "LIDAR_STALE"))
    for key, reason in stale_reasons:
        age = _age(ages, key)
        if age is None or age > limits.input_stale_sec:
            state = "FUSED_ERROR" if age is None or age > limits.hard_failure_sec else "FUSED_DEGRADED"
            return _result(state, reason, map_activity=map_activity,
                           cloud_activity=cloud_activity, rtab_state=rtab_state)
    if rtab_age is None or rtab_age > limits.rtab_stale_sec:
        state = "FUSED_ERROR" if rtab_age is None or rtab_age > limits.hard_failure_sec else "FUSED_DEGRADED"
        return _result(state, "RTAB_PROCESSING_TIMEOUT", map_activity=map_activity,
                       cloud_activity=cloud_activity, rtab_state=rtab_state)
    if not map_available:
        return _result("FUSED_ERROR", "MAP_NOT_PRODUCED", map_activity=map_activity,
                       cloud_activity=cloud_activity, rtab_state=rtab_state)
    if not cloud_available:
        return _result("FUSED_ERROR", "CLOUD_NOT_PRODUCED", map_activity=map_activity,
                       cloud_activity=cloud_activity, rtab_state=rtab_state)
    if not stationary and (map_activity != "UPDATING" or cloud_activity != "UPDATING"):
        return _result("FUSED_DEGRADED", "MOVING_WITHOUT_MAPPING_PROGRESS",
                       map_activity=map_activity, cloud_activity=cloud_activity,
                       rtab_state=rtab_state)
    reason = "PIPELINE_HEALTHY_UPDATING" if (map_activity == "UPDATING" or cloud_activity == "UPDATING") else "PIPELINE_HEALTHY_MAP_QUIESCENT"
    return _result("FUSED_READY", reason, map_activity=map_activity,
                   cloud_activity=cloud_activity, rtab_state=rtab_state)
