#!/usr/bin/env python3
"""Own one deterministic ZED-to-RTAB manual mapping session at a time."""

from __future__ import annotations

import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import sqlite3
import subprocess
import threading
import time
from typing import Dict
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rtabmap_msgs.msg import Info, MapData, RGBDImage
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .fused_policy import FUSED_MAPPING, MAPPING_SOURCES, fused_ready, output_map_topic, output_prefix
from .health_model import HealthThresholds, derive_fused_health


STATES = ("IDLE", "STARTING", "MAPPING", "DEGRADED", "FINALIZING", "COMPLETE", "ERROR")
PROFILES = ("indoor_live", "indoor_high_quality", "outdoor_structured", "outdoor_open_field")


class MappingSessionManager(Node):
    def __init__(self) -> None:
        super().__init__("mapping_session_manager")
        self.declare_parameter("sessions_root", "/home/ubuntu/laksa_mapping_sessions")
        self.declare_parameter("profile", "indoor_live")
        self.declare_parameter("record_svo", False)
        self.declare_parameter("mapping_source", FUSED_MAPPING)
        self.declare_parameter("compose_rgbd_rtab", False)
        self.declare_parameter("grid_noise_filtering_radius_override", -1.0)
        self.declare_parameter("cloud_output_voxelized_override", -1)
        self.declare_parameter("enable_dense_cloud_map", False)
        self.declare_parameter("hybrid_scan_timeout_sec", 1.0)
        self.declare_parameter("startup_timeout_sec", 45.0)
        self.declare_parameter("data_timeout_sec", 5.0)
        self.declare_parameter("database_growth_timeout_sec", 30.0)
        self.declare_parameter("rtab_processing_timeout_sec", 6.0)
        self.declare_parameter("output_activity_timeout_sec", 5.0)
        self.declare_parameter("hard_failure_timeout_sec", 15.0)
        # RTAB publishes /map when its map content changes, not continuously.
        # Nav2's saver otherwise gives a new subscription only two seconds to
        # observe a sample, which is shorter than a valid low-rate RTAB cycle.
        self.declare_parameter("occupancy_save_timeout_sec", 15.0)
        self.declare_parameter("stationary_window_sec", 5.0)
        self.declare_parameter("stationary_translation_threshold_m", 0.02)
        self.declare_parameter("stationary_yaw_threshold_rad", 0.035)
        self._root = Path(str(self.get_parameter("sessions_root").value))
        self._share = Path(__file__).resolve().parents[1] / "share" / "laksa_mapping"
        if not self._share.exists():
            from ament_index_python.packages import get_package_share_directory
            self._share = Path(get_package_share_directory("laksa_mapping"))
        self._profile = str(self.get_parameter("profile").value)
        self._record_svo = bool(self.get_parameter("record_svo").value)
        self._mapping_source = str(self.get_parameter("mapping_source").value)
        if self._mapping_source not in MAPPING_SOURCES:
            self._mapping_source = FUSED_MAPPING
        self._hybrid_scan_timeout = float(self.get_parameter("hybrid_scan_timeout_sec").value)
        self._startup_timeout = float(self.get_parameter("startup_timeout_sec").value)
        self._data_timeout = float(self.get_parameter("data_timeout_sec").value)
        self._db_timeout = float(self.get_parameter("database_growth_timeout_sec").value)
        self._health_thresholds = HealthThresholds(
            input_stale_sec=self._data_timeout,
            rtab_stale_sec=float(self.get_parameter("rtab_processing_timeout_sec").value),
            activity_recent_sec=float(self.get_parameter("output_activity_timeout_sec").value),
            startup_grace_sec=self._startup_timeout,
            hard_failure_sec=float(self.get_parameter("hard_failure_timeout_sec").value),
        )
        self._stationary_window = float(self.get_parameter("stationary_window_sec").value)
        self._stationary_translation = float(self.get_parameter("stationary_translation_threshold_m").value)
        self._stationary_yaw = float(self.get_parameter("stationary_yaw_threshold_rad").value)
        self._occupancy_save_timeout = float(
            self.get_parameter("occupancy_save_timeout_sec").value
        )
        self._state = "IDLE"
        self._error = ""
        self._compose_rgbd_rtab = False
        self._grid_noise_filtering_radius = 0.0
        self._grid_noise_filtering_radius_overridden = False
        self._cloud_output_voxelized = True
        self._cloud_output_voxelized_overridden = False
        self._dense_cloud_enabled = False
        self._session_id = ""
        self._session_dir: Path | None = None
        self._db_path: Path | None = None
        self._svo_path: Path | None = None
        self._process: subprocess.Popen | None = None
        self._process_log = None
        self._started_monotonic = 0.0
        self._last: Dict[str, float] = {}
        self._lidar_health = "UNKNOWN"
        self._validated_scan_frame = ""
        self._lidar_health_received = 0.0
        self._scan_times = deque(maxlen=200)
        self._scan_header_stamps = deque(maxlen=400)
        self._rgbd_scan_stamp_offsets = deque(maxlen=200)
        self._rgbd_scan_nearest_deltas = deque(maxlen=200)
        self._latest_header_stamps = {}
        self._occupancy_times = deque(maxlen=200)
        self._rates = {key: deque(maxlen=200) for key in (
            "rgb", "depth", "camera_info", "zed_odom", "fused_odom", "rgbd", "rtab",
            "fused_cloud", "dense_cloud", "zed_cloud",
        )}
        self._mapping_metrics = {}
        self._motion_samples = deque(maxlen=400)
        self._map_signature = None
        self._cloud_signature = None
        self._dense_cloud_signature = None
        self._last_rtab_processed_stamp = None
        self._finalizing = False
        self._lock = threading.RLock()
        qos_state = QoSProfile(depth=1)
        qos_state.reliability = ReliabilityPolicy.RELIABLE
        qos_state.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_pub = self.create_publisher(String, "/laksa/mapping/state", qos_state)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        cb = ReentrantCallbackGroup()
        self.create_subscription(String, "/laksa/mapping/profile_request", self._profile_cb, 10, callback_group=cb)
        self.create_subscription(String, "/laksa/mapping/source_request", self._source_cb, 10, callback_group=cb)
        self.create_subscription(Bool, "/laksa/mapping/record_svo_request", self._record_cb, 10, callback_group=cb)
        self.create_subscription(String, "/laksa/lidar/health_state", self._lidar_health_cb, qos_state, callback_group=cb)
        self.create_subscription(LaserScan, "/laksa/lidar/scan_validated", self._validated_scan_cb, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(Image, "/zed/zed_node/rgb/color/rect/image", lambda _: self._touch_rate("rgb"), qos_profile_sensor_data)
        self.create_subscription(Image, "/zed/zed_node/depth/depth_registered", lambda _: self._touch_rate("depth"), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/zed/zed_node/rgb/color/rect/camera_info", self._camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/zed/zed_node/odom", lambda _: self._touch_rate("zed_odom"), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/laksa/odometry/fused", self._fused_odom_cb, qos_profile_sensor_data)
        for source in MAPPING_SOURCES:
            prefix = output_prefix(source)
            self.create_subscription(RGBDImage, prefix + "/rgbd_image", lambda msg, value=source: self._rgbd_cb(value, msg), qos_profile_sensor_data)
            self.create_subscription(MapData, prefix + "/mapData", lambda msg, value=source: self._map_data_cb(value, msg), qos_profile_sensor_data)
            self.create_subscription(Info, prefix + "/info", lambda msg, value=source: self._info_cb(value, msg), qos_profile_sensor_data)
            self.create_subscription(PointCloud2, prefix + "/cloud_map", lambda msg, value=source: self._cloud_cb(value, msg), qos_profile_sensor_data)
            if source == FUSED_MAPPING:
                self.create_subscription(PointCloud2, prefix + "/dense_cloud_map", self._dense_cloud_cb, qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, output_map_topic(source), lambda msg, value=source: self._occupancy_cb(value, msg), 10)
        self.create_service(Trigger, "/laksa/mapping/start", self._start_cb, callback_group=cb)
        self.create_service(Trigger, "/laksa/mapping/stop", self._stop_cb, callback_group=cb)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.5, self._watchdog, callback_group=cb)
        self.create_timer(1.0, self._publish)
        self._root.mkdir(parents=True, exist_ok=True)
        self._publish()

    def _touch(self, key: str) -> None:
        self._last[key] = time.monotonic()

    def _touch_rate(self, key: str) -> None:
        now=time.monotonic(); self._last[key]=now; self._rates[key].append(now)

    def _profile_cb(self, msg: String) -> None:
        if msg.data in PROFILES and self._state in ("IDLE", "COMPLETE", "ERROR"):
            self._profile = msg.data

    def _source_cb(self, msg: String) -> None:
        if msg.data in MAPPING_SOURCES and self._state in ("IDLE", "COMPLETE", "ERROR"):
            self._mapping_source = msg.data

    def _record_cb(self, msg: Bool) -> None:
        if self._state in ("IDLE", "COMPLETE", "ERROR"):
            self._record_svo = bool(msg.data)

    @staticmethod
    def _rate(samples) -> float:
        return 0.0 if len(samples) < 2 else (len(samples) - 1) / max(1.0e-6, samples[-1] - samples[0])

    @staticmethod
    def _percentile(samples, fraction: float) -> float | None:
        if not samples:
            return None
        ordered = sorted(samples)
        return ordered[min(len(ordered) - 1, int(math.ceil(fraction * len(ordered))) - 1)]

    def _rgbd_scan_stamp_metrics(self) -> dict:
        with self._lock:
            offsets = tuple(self._rgbd_scan_stamp_offsets)
            nearest = tuple(self._rgbd_scan_nearest_deltas)
            stamps = dict(self._latest_header_stamps)
        ros_now = self.get_clock().now().nanoseconds * 1.0e-9
        return {
            "samples": len(offsets),
            "rgbd_header_stamp_sec": stamps.get("rgbd"),
            "scan_header_stamp_sec": stamps.get("scan"),
            "camera_info_header_stamp_sec": stamps.get("camera_info"),
            "rgbd_header_age_sec": None if "rgbd" not in stamps else round(ros_now - stamps["rgbd"], 6),
            "scan_header_age_sec": None if "scan" not in stamps else round(ros_now - stamps["scan"], 6),
            "latest_scan_minus_rgbd_sec": None if not offsets else round(offsets[-1], 6),
            "latest_scan_minus_rgbd_p95_sec": None if not offsets else round(self._percentile(offsets, 0.95), 6),
            "nearest_scan_delta_latest_sec": None if not nearest else round(nearest[-1], 6),
            "nearest_scan_delta_p95_sec": None if not nearest else round(self._percentile(nearest, 0.95), 6),
            "nearest_scan_delta_max_sec": None if not nearest else round(max(nearest), 6),
            "nearest_scan_within_100ms_fraction": None if not nearest else round(sum(value <= 0.1 for value in nearest) / len(nearest), 6),
        }

    def _lidar_health_cb(self, msg: String) -> None:
        self._lidar_health = msg.data
        self._lidar_health_received = time.monotonic()

    @staticmethod
    def _stamp_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._touch_rate("camera_info")
        with self._lock:
            self._latest_header_stamps["camera_info"] = self._stamp_sec(msg.header.stamp)

    def _rgbd_cb(self, source: str, msg: RGBDImage) -> None:
        if source != self._mapping_source:
            return
        self._touch_rate("rgbd")
        stamp = self._stamp_sec(msg.header.stamp)
        with self._lock:
            self._latest_header_stamps["rgbd"] = stamp
            if self._scan_header_stamps:
                latest_scan_stamp = self._scan_header_stamps[-1]
                self._rgbd_scan_stamp_offsets.append(latest_scan_stamp - stamp)
                self._rgbd_scan_nearest_deltas.append(
                    min(abs(scan_stamp - stamp) for scan_stamp in self._scan_header_stamps)
                )

    def _validated_scan_cb(self, msg: LaserScan) -> None:
        self._validated_scan_frame = msg.header.frame_id
        if msg.header.frame_id != "lidar_link":
            return
        now = time.monotonic()
        stamp = self._stamp_sec(msg.header.stamp)
        with self._lock:
            self._scan_times.append(now)
            self._scan_header_stamps.append(stamp)
            self._latest_header_stamps["scan"] = stamp
            self._last["scan"] = now

    def _fused_odom_cb(self, msg: Odometry) -> None:
        self._touch_rate("fused_odom")
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        sample = (time.monotonic(), float(pose.position.x), float(pose.position.y), yaw)
        with self._lock:
            self._motion_samples.append(sample)

    def _stationary(self) -> bool:
        # The odometry callback and watchdog run concurrently in the
        # MultiThreadedExecutor.  Snapshot under the session lock so iterating
        # cannot race a deque append ("deque mutated during iteration").
        with self._lock:
            samples = tuple(self._motion_samples)
        if len(samples) < 2:
            return False
        newest = samples[-1]
        cutoff = newest[0] - self._stationary_window
        oldest = next((sample for sample in samples if sample[0] >= cutoff), samples[0])
        if newest[0] - oldest[0] < self._stationary_window * 0.8:
            return False
        translation = math.hypot(newest[1] - oldest[1], newest[2] - oldest[2])
        yaw_delta = abs((newest[3] - oldest[3] + math.pi) % (2.0 * math.pi) - math.pi)
        self._mapping_metrics.update({
            "stationary_translation_delta_m": round(translation, 6),
            "stationary_yaw_delta_rad": round(yaw_delta, 6),
            "stationary_window_sec": round(newest[0] - oldest[0], 3),
        })
        return translation <= self._stationary_translation and yaw_delta <= self._stationary_yaw

    def _output_touch(self, source: str, key: str) -> bool:
        if source != self._mapping_source or self._state not in ("STARTING", "MAPPING", "DEGRADED"):
            return False
        self._touch(key)
        return True

    def _map_data_cb(self, source: str, msg: MapData) -> None:
        if not self._output_touch(source, "map"):
            return
        poses = getattr(getattr(msg, "graph", None), "poses", ())
        self._mapping_metrics["rtab_node_count_live"] = len(poses)

    def _info_cb(self, source: str, msg: Info) -> None:
        if source != self._mapping_source or self._state not in ("STARTING", "MAPPING", "DEGRADED"):
            return
        self._touch_rate("rtab")
        self._last["rtab_processed"] = time.monotonic()
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is not None:
            self._last_rtab_processed_stamp = int(stamp.sec) + int(stamp.nanosec) / 1.0e9
        timings = []
        for key, value in zip(msg.stats_keys, msg.stats_values):
            if "Timing/Total" in key:
                timings.append(float(value))
            if any(token in key for token in ("Loop/Closure", "Proximity", "Icp/Inliers", "Icp/Correspondence")):
                self._mapping_metrics["rtab_" + key.replace("/", "_").replace(" ", "_").lower()] = float(value)
        if timings:
            current = max(timings)
            count = int(self._mapping_metrics.get("rtab_timing_samples", 0)) + 1
            prior = float(self._mapping_metrics.get("rtab_processing_ms_mean", 0.0))
            self._mapping_metrics.update({
                "rtab_processing_ms_latest": round(current, 3),
                "rtab_processing_ms_mean": round(prior + (current - prior) / count, 3),
                "rtab_processing_ms_max": round(max(current, float(self._mapping_metrics.get("rtab_processing_ms_max", 0.0))), 3),
                "rtab_timing_samples": count,
            })

    def _cloud_cb(self, source: str, msg: PointCloud2) -> None:
        if self._output_touch(source,"cloud"):
            self._rates["fused_cloud"].append(time.monotonic())
            signature = (int(msg.width), int(msg.height), int(msg.row_step),
                         hashlib.blake2b(memoryview(msg.data), digest_size=8).digest())
            if signature != self._cloud_signature:
                self._cloud_signature = signature
                self._touch("cloud_content")
            self._mapping_metrics["cloud_point_count"] = int(msg.width) * int(msg.height)

    def _dense_cloud_cb(self, msg: PointCloud2) -> None:
        if not self._dense_cloud_enabled or not self._output_touch(FUSED_MAPPING, "dense_cloud"):
            return
        self._rates["dense_cloud"].append(time.monotonic())
        signature = (int(msg.width), int(msg.height), int(msg.row_step),
                     hashlib.blake2b(memoryview(msg.data), digest_size=8).digest())
        if signature != self._dense_cloud_signature:
            self._dense_cloud_signature = signature
            self._touch("dense_cloud_content")
        self._mapping_metrics.update({
            "dense_cloud_point_count": int(msg.width) * int(msg.height),
            "dense_cloud_serialized_bytes": len(msg.data),
        })

    def _occupancy_cb(self, source: str, msg: OccupancyGrid) -> None:
        if not self._output_touch(source, "occupancy"):
            return
        now = time.monotonic()
        self._occupancy_times.append(now)
        signature = hashlib.blake2b(bytes(value + 1 for value in msg.data), digest_size=8).digest()
        if signature != self._map_signature:
            self._map_signature = signature
            self._touch("map_content")
        unknown = sum(1 for value in msg.data if value < 0)
        occupied = sum(1 for value in msg.data if value >= 50)
        self._mapping_metrics.update({
            "map_width_cells": int(msg.info.width), "map_height_cells": int(msg.info.height),
            "map_resolution_m": float(msg.info.resolution), "unknown_cells": unknown,
            "occupied_cells": occupied, "free_cells": len(msg.data) - unknown - occupied,
            "observed_fraction": round((len(msg.data) - unknown) / max(1, len(msg.data)), 6),
            "unknown_fraction": round(unknown / max(1, len(msg.data)), 6),
            "observed_area_m2": round((len(msg.data) - unknown) * float(msg.info.resolution) ** 2, 3),
            "occupancy_update_rate_hz": round(self._rate(self._occupancy_times), 3),
        })

    def _tf_health(self) -> dict:
        health = {}
        for parent, child, key in (("map", "odom", "map_to_odom"), ("odom", "base_footprint", "odom_to_base")):
            try:
                transform = self._tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                stamp = transform.header.stamp
                health[key] = {"available": True, "authority_count_expected": 1,
                               "stamp_sec": int(stamp.sec) + int(stamp.nanosec) / 1.0e9}
            except TransformException as exc:
                health[key] = {"available": False, "error": str(exc)}
        return health

    def _scan_age(self):
        return None if "scan" not in self._last else time.monotonic() - self._last["scan"]

    def _fused_available(self) -> bool:
        return fused_ready(self._lidar_health, self._scan_age(), self._hybrid_scan_timeout)

    def _conflicting_nodes(self) -> list[str]:
        names = {f"{ns.rstrip('/')}/{name}".replace("//", "/") for name, ns in self.get_node_names_and_namespaces()}
        guarded = (
            "/zed/zed_node", "/laksa/fused_mapping/rgbd_sync", "/laksa/fused_mapping/rtabmap",
            "/zed_rtabmap/rtabmap", "/laksa/mapping_shadow/rtabmap",
        )
        return [name for name in guarded if name in names]

    def _start_cb(self, _request, response):
        with self._lock:
            if self._state not in ("IDLE", "COMPLETE", "ERROR"):
                response.success = False
                response.message = f"Mapping is already {self._state.lower()}"
                return response
            conflicts = self._conflicting_nodes()
            if conflicts:
                response.success = False
                response.message = "Conflicting mapping nodes are active: " + ", ".join(conflicts)
                return response
            if self._profile not in PROFILES:
                response.success = False
                response.message = f"Unknown profile: {self._profile}"
                return response
            radius_override = float(self.get_parameter("grid_noise_filtering_radius_override").value)
            if radius_override not in (-1.0, 0.0, 0.05):
                response.success = False
                response.message = "Grid noise radius override must be -1.0 (production), 0.0, or 0.05"
                return response
            cloud_voxel_override = int(self.get_parameter("cloud_output_voxelized_override").value)
            if cloud_voxel_override not in (-1, 0, 1):
                response.success = False
                response.message = "Cloud output voxel override must be -1 (production), 0 (off), or 1 (on)"
                return response
            if not self._fused_available():
                response.success = False
                response.message = "FUSED_UNAVAILABLE: validated LiDAR must be fresh and GOOD"
                return response
            self._begin_session()
            response.success = True
            response.message = f"Starting {self._mapping_source} mapping session {self._session_id}"
            return response

    def _begin_session(self) -> None:
        now = dt.datetime.now(dt.timezone.utc)
        self._session_id = now.strftime("%Y%m%dT%H%M%SZ")
        self._session_dir = self._root / self._session_id
        for child in ("logs", "database", "svo", "export", "occupancy", "runtime"):
            (self._session_dir / child).mkdir(parents=True, exist_ok=False)
        self._db_path = self._session_dir / "database" / "map.db"
        self._svo_path = self._session_dir / "svo" / "session.svo2" if self._record_svo else None
        zed_name = "indoor_high_quality_zed.yaml" if self._profile == "indoor_high_quality" else "indoor_live_zed.yaml"
        zed_config = self._session_dir / "runtime" / zed_name
        shutil.copy2(self._share / "config" / zed_name, zed_config)
        rtab_config = self._session_dir / "runtime" / "rtabmap.yaml"
        rtab_name = "rtabmap_fused.yaml"
        launch_name = "mapping_stack.launch.py"
        text = (self._share / "config" / rtab_name).read_text(encoding="utf-8")
        radius_override = float(self.get_parameter("grid_noise_filtering_radius_override").value)
        radius_match = re.search(r"(?m)^[ \t]*Grid/NoiseFilteringRadius:[ \t]*['\"]?([0-9.]+)['\"]?[ \t]*$", text)
        if radius_match is None:
            raise RuntimeError("Grid/NoiseFilteringRadius is missing from the fused RTAB configuration")
        production_radius = float(radius_match.group(1))
        self._grid_noise_filtering_radius_overridden = radius_override >= 0.0
        self._grid_noise_filtering_radius = radius_override if self._grid_noise_filtering_radius_overridden else production_radius
        if self._grid_noise_filtering_radius_overridden:
            value = "0.0" if self._grid_noise_filtering_radius == 0.0 else "0.05"
            text, replacements = re.subn(
                r"(?m)^([ \t]*Grid/NoiseFilteringRadius:[ \t]*)['\"]?[0-9.]+['\"]?[ \t]*$",
                rf"\1'{value}'",
                text,
                count=1,
            )
            if replacements != 1:
                raise RuntimeError("Could not apply Grid/NoiseFilteringRadius override")
        cloud_voxel_override = int(self.get_parameter("cloud_output_voxelized_override").value)
        cloud_voxel_match = re.search(r"(?m)^[ \t]*cloud_output_voxelized:[ \t]*(true|false)[ \t]*$", text)
        if cloud_voxel_match is None:
            raise RuntimeError("cloud_output_voxelized is missing from the fused RTAB configuration")
        production_cloud_voxelized = cloud_voxel_match.group(1) == "true"
        self._cloud_output_voxelized_overridden = cloud_voxel_override >= 0
        self._cloud_output_voxelized = bool(cloud_voxel_override) if self._cloud_output_voxelized_overridden else production_cloud_voxelized
        if self._cloud_output_voxelized_overridden:
            text, replacements = re.subn(
                r"(?m)^([ \t]*cloud_output_voxelized:[ \t]*)(?:true|false)[ \t]*$",
                rf"\1{'true' if self._cloud_output_voxelized else 'false'}",
                text,
                count=1,
            )
            if replacements != 1:
                raise RuntimeError("Could not apply cloud_output_voxelized override")
        text += f"    database_path: {self._db_path}\n"
        rtab_config.write_text(text, encoding="utf-8")
        self._process_log = open(self._session_dir / "logs" / "mapping.log", "a", encoding="utf-8", buffering=1)
        self._compose_rgbd_rtab = bool(self.get_parameter("compose_rgbd_rtab").value)
        self._dense_cloud_enabled = bool(self.get_parameter("enable_dense_cloud_map").value)
        cmd = [
            "ros2", "launch", "laksa_mapping", launch_name,
            f"zed_config:={zed_config}", f"rtab_config:={rtab_config}",
            f"compose_rgbd_rtab:={'true' if self._compose_rgbd_rtab else 'false'}",
            f"enable_dense_cloud_map:={'true' if self._dense_cloud_enabled else 'false'}",
        ]
        self._process = subprocess.Popen(cmd, stdout=self._process_log, stderr=subprocess.STDOUT, env=os.environ.copy(), start_new_session=True)
        self._started_monotonic = time.monotonic()
        self._last.clear()
        self._scan_times.clear()
        self._scan_header_stamps.clear()
        self._rgbd_scan_stamp_offsets.clear()
        self._rgbd_scan_nearest_deltas.clear()
        self._latest_header_stamps.clear()
        self._occupancy_times.clear()
        [samples.clear() for samples in self._rates.values()]
        self._mapping_metrics = {}
        self._motion_samples.clear()
        self._map_signature = None
        self._cloud_signature = None
        self._dense_cloud_signature = None
        self._last_rtab_processed_stamp = None
        self._error = ""
        self._state = "STARTING"
        self._write_metadata("STARTING")
        self._publish()

    def _stop_cb(self, _request, response):
        with self._lock:
            if self._state not in ("STARTING", "MAPPING", "DEGRADED"):
                response.success = False
                response.message = f"Cannot stop mapping from {self._state}"
                return response
            if self._finalizing:
                response.success = False
                response.message = "Finalization already in progress"
                return response
            self._state = "FINALIZING"
            self._finalizing = True
            threading.Thread(target=self._finalize, args=(False,), daemon=True).start()
            response.success = True
            response.message = "Mapping finalization started"
            return response

    def _call_svo(self, start: bool) -> bool:
        if not self._record_svo:
            return True
        if start:
            request = "{bitrate: 0, compression_mode: 5, target_framerate: 30, input_transcode: false, svo_filename: '%s'}" % self._svo_path
            cmd = ["ros2", "service", "call", "/zed/zed_node/start_svo_rec", "zed_msgs/srv/StartSvoRec", request]
        else:
            cmd = ["ros2", "service", "call", "/zed/zed_node/stop_svo_rec", "std_srvs/srv/Trigger", "{}"]
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=20)
        if self._session_dir:
            with open(self._session_dir / "logs" / "svo.log", "a", encoding="utf-8") as log:
                log.write(result.stdout)
        return result.returncode == 0 and "success=True" in result.stdout.replace(" ", "")

    def _save_occupancy(self) -> bool:
        if not self._session_dir:
            return False
        target = self._session_dir / "occupancy" / "map"
        cmd = [
            "ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(target),
            "--ros-args",
            "-p", f"save_map_timeout:={self._occupancy_save_timeout}",
            "-r", f"map:={output_map_topic(self._mapping_source)}",
        ]
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=self._occupancy_save_timeout + 10.0,
        )
        (self._session_dir / "logs" / "occupancy_export.log").write_text(result.stdout, encoding="utf-8")
        return result.returncode == 0 and target.with_suffix(".yaml").is_file()

    def _terminate_owned_process(self) -> None:
        proc = self._process
        if proc is None or proc.poll() is not None:
            return
        for sig, timeout in ((signal.SIGINT, 20), (signal.SIGTERM, 8), (signal.SIGKILL, 3)):
            try:
                os.killpg(proc.pid, sig)
                proc.wait(timeout=timeout)
                return
            except subprocess.TimeoutExpired:
                continue
            except ProcessLookupError:
                return

    def _validate_db(self) -> tuple[bool, str, int]:
        if not self._db_path or not self._db_path.is_file() or self._db_path.stat().st_size < 100000:
            return False, "database missing or trivial", 0
        try:
            with sqlite3.connect(f"file:{self._db_path}?mode=ro", uri=True) as conn:
                quick = str(conn.execute("PRAGMA quick_check").fetchone()[0])
                nodes = int(conn.execute("SELECT COUNT(*) FROM Node").fetchone()[0])
            return quick == "ok" and nodes > 0, quick, nodes
        except Exception as exc:
            return False, str(exc), 0

    def _export_ply(self) -> bool:
        if not self._session_dir or not self._db_path:
            return False
        out = self._session_dir / "export"
        cmd = ["rtabmap-export", "--cloud", "--poses", "--poses_camera", "--opt", "2", "--output", "map_cloud", "--output_dir", str(out), str(self._db_path)]
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=300)
        (self._session_dir / "logs" / "cloud_export.log").write_text(result.stdout, encoding="utf-8")
        return result.returncode == 0 and any(out.glob("*.ply"))

    def _finalize(self, hard_failure: bool) -> None:
        errors = []
        original_error = self._error
        occupancy_ok = False
        svo_ok = True
        try:
            if not hard_failure:
                occupancy_ok = self._save_occupancy()
                if not occupancy_ok:
                    errors.append("2D occupancy export failed")
            if self._record_svo:
                svo_ok = self._call_svo(False)
                if not svo_ok:
                    errors.append("SVO stop/finalization failed")
            self._terminate_owned_process()
            valid, check, nodes = self._validate_db()
            if not valid:
                errors.append(f"RTAB database invalid: {check}, nodes={nodes}")
            ply_ok = self._export_ply() if valid else False
            if valid and not ply_ok:
                errors.append("PLY export failed")
            self._error = "; ".join(([original_error] if original_error else []) + errors)
            self._state = "ERROR" if errors or hard_failure else "COMPLETE"
            self._write_metadata(self._state, {"database_valid": valid, "database_check": check, "database_nodes": nodes, "occupancy_exported": occupancy_ok, "ply_exported": ply_ok, "svo_finalized": svo_ok})
        except Exception as exc:
            self._error = f"Finalization exception: {exc}"
            self._state = "ERROR"
            self._terminate_owned_process()
            self._write_metadata("ERROR")
        finally:
            if self._process_log:
                self._process_log.close()
                self._process_log = None
            self._finalizing = False
            self._publish()

    def _watchdog(self) -> None:
        with self._lock:
            if self._state not in ("STARTING", "MAPPING", "DEGRADED"):
                return
            now = time.monotonic()
            if self._process and self._process.poll() is not None:
                self._hard_fail(f"Mapping launch exited with code {self._process.returncode}")
                return
            if self._state == "STARTING":
                inputs_ready = all(now - self._last.get(key, 0.0) < 3.0 for key in ("rgb", "depth", "zed_odom", "fused_odom"))
                inputs_ready = inputs_ready and self._fused_available() and now-self._last.get("rgbd",0.0)<3.0
                map_ready = any(now - self._last.get(key, 0.0) < 3.0 for key in ("map", "cloud", "occupancy"))
                if inputs_ready and map_ready:
                    if self._record_svo and not self._call_svo(True):
                        self._hard_fail("ZED SVO recording failed to start")
                        return
                    self._state = "MAPPING"
                    self._publish()
                elif now - self._started_monotonic > self._startup_timeout:
                    self._hard_fail("Mapping topics did not become healthy before startup timeout")
                return
            stale_inputs = [key for key in ("rgb", "depth", "zed_odom", "fused_odom") if now - self._last.get(key, 0.0) > self._data_timeout]
            if stale_inputs:
                self._hard_fail("Stale required mapping inputs: " + ", ".join(stale_inputs))
                return
            health = self._health_snapshot()
            if health["fusion_state"] == "FUSED_ERROR":
                self._hard_fail(health["reason_code"])
                return
            self._state = "MAPPING" if health["fusion_state"] == "FUSED_READY" else "DEGRADED"
            if self._db_path and now - self._started_monotonic > self._db_timeout and (not self._db_path.exists() or self._db_path.stat().st_size < 100000):
                self._hard_fail("RTAB database did not grow after mapping startup")

    def _hard_fail(self, message: str) -> None:
        if self._finalizing:
            return
        self._error = message
        self._state = "FINALIZING"
        self._finalizing = True
        threading.Thread(target=self._finalize, args=(True,), daemon=True).start()

    def _health_snapshot(self) -> dict:
        now = time.monotonic()
        keys = ("rgb", "depth", "rgbd", "fused_odom", "scan", "rtab_processed", "map_content", "cloud_content")
        ages = {key: (now - self._last[key] if key in self._last else None) for key in keys}
        process_alive = self._process is not None and self._process.poll() is None
        active = self._state in ("STARTING", "MAPPING", "DEGRADED", "FINALIZING")
        return derive_fused_health(
            active=active, startup=self._state == "STARTING",
            elapsed_sec=0.0 if not self._started_monotonic else now - self._started_monotonic,
            ages=ages, lidar_health=self._lidar_health, tf_health=self._tf_health(),
            process_alive=process_alive, map_available="occupancy" in self._last,
            cloud_available="cloud" in self._last, stationary=self._stationary(),
            thresholds=self._health_thresholds,
        )

    def _payload(self) -> dict:
        elapsed = 0.0 if not self._started_monotonic else time.monotonic() - self._started_monotonic
        now = time.monotonic()
        signal_ages = {
            key: (round(now - self._last[key], 2) if key in self._last else None)
            for key in ("rgb", "depth", "zed_odom", "fused_odom", "map", "cloud", "occupancy")
        }
        zed_ages = [signal_ages[key] for key in ("rgb", "depth", "zed_odom") if signal_ages[key] is not None]
        progress_ages = [signal_ages[key] for key in ("map", "cloud", "occupancy") if signal_ages[key] is not None]
        health = self._health_snapshot()
        return {
            "state": self._state, "session_id": self._session_id, "profile": self._profile,
            "mapping_source": self._mapping_source,
            "rgbd_rtab_topology": "composed_intra_process" if self._compose_rgbd_rtab else "legacy_separate_processes",
            "grid_noise_filtering_radius": self._grid_noise_filtering_radius,
            "grid_noise_filtering_radius_overridden": self._grid_noise_filtering_radius_overridden,
            "cloud_output_voxelized": self._cloud_output_voxelized,
            "cloud_output_voxelized_overridden": self._cloud_output_voxelized_overridden,
            "dense_cloud_enabled": self._dense_cloud_enabled,
            "field_lab_3d_source": "live_rgbd_preview" if self._dense_cloud_enabled else "occupancy_cloud",
            "experimental": self._grid_noise_filtering_radius_overridden or self._cloud_output_voxelized_overridden or self._dense_cloud_enabled,
            "fusion_state": "FUSED_ERROR" if self._state == "ERROR" else health["fusion_state"],
            "fusion_reason_code": self._error or health["reason_code"],
            "profile_status": "provisional_unvalidated" if self._profile.startswith("outdoor_") else "validated",
            "elapsed_sec": round(elapsed, 1), "db_path": str(self._db_path or ""),
            "svo_enabled": self._record_svo, "svo_path": str(self._svo_path or ""), "error": self._error,
            "signal_ages_sec": signal_ages,
            "zed_input_age_sec": max(zed_ages) if len(zed_ages) == 3 else None,
            "rtab_progress_age_sec": min(progress_ages) if progress_ages else None,
            "last_rtab_processed_stamp": self._last_rtab_processed_stamp,
            "rtab_processing_age_sec": None if "rtab_processed" not in self._last else round(now - self._last["rtab_processed"], 2),
            "map_update_age_sec": signal_ages["occupancy"],
            "validated_scan_age_sec": None if self._scan_age() is None else round(self._scan_age(), 2),
            "validated_scan_rate_hz": round(self._rate(self._scan_times), 3),
            "lidar_health": self._lidar_health,
            "validated_scan_frame": self._validated_scan_frame,
            "rgbd_scan_stamp_metrics": self._rgbd_scan_stamp_metrics(),
            "tf_health": self._tf_health(),
            "health_layers": health,
            "rates_hz": {key:round(self._rate(samples),3) for key,samples in self._rates.items()} | {"validated_scan":round(self._rate(self._scan_times),3),"occupancy":round(self._rate(self._occupancy_times),3)},
            "topics":{"map":"/map","rgbd":"/laksa/fused_mapping/rgbd_image","scan":"/laksa/lidar/scan_validated","odom":"/laksa/odometry/fused","occupancy_cloud":"/laksa/fused_mapping/cloud_map","dense_cloud":"/zed/zed_node/point_cloud/cloud_registered","cloud":"/zed/zed_node/point_cloud/cloud_registered" if self._dense_cloud_enabled else "/laksa/fused_mapping/cloud_map","zed_cloud":"/zed/zed_node/point_cloud/cloud_registered"},
            "comparison_metrics": dict(self._mapping_metrics),
        }

    def _publish(self) -> None:
        payload = self._payload()
        self._state_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        level = DiagnosticStatus.ERROR if self._state == "ERROR" else (DiagnosticStatus.WARN if self._state in ("STARTING", "DEGRADED", "FINALIZING") else DiagnosticStatus.OK)
        status = DiagnosticStatus(level=level, name="laksa_mapping/session", hardware_id="zed2i-rtabmap", message=self._error or self._state)
        status.values = [KeyValue(key=k, value=str(v)) for k, v in payload.items()]
        array = DiagnosticArray(); array.header.stamp = self.get_clock().now().to_msg(); array.status = [status]
        self._diag_pub.publish(array)

    def _write_metadata(self, result: str, extra: dict | None = None) -> None:
        if not self._session_dir:
            return
        architecture = "ONE ZED RGB-D + validated A2M12 LaserScan + fused odom -> ONE RTAB-Map -> /map"
        config_hashes = {}
        for name in ("rtabmap.yaml", "indoor_live_zed.yaml", "indoor_high_quality_zed.yaml"):
            path = self._session_dir / "runtime" / name
            if path.is_file():
                config_hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        data = self._payload(); data.update({"result": result, "updated_utc": dt.datetime.now(dt.timezone.utc).isoformat(), "architecture": architecture, "autonomous_motion": False, "configuration_hashes": config_hashes})
        if extra: data.update(extra)
        tmp = self._session_dir / "metadata.json.tmp"
        tmp.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        tmp.replace(self._session_dir / "metadata.json")
        if result in ("COMPLETE", "ERROR"):
            metrics = data.get("comparison_metrics", {})
            rows = [
                "# LAKSA Fused Mapping Session", "",
                f"- Session: `{data['session_id']}`", f"- Mode: `{data['mapping_source']}`",
                f"- Result: `{result}`", f"- Duration: {data['elapsed_sec']} s",
                f"- RTAB nodes: {data.get('database_nodes', metrics.get('rtab_node_count_live', 'n/a'))}",
                f"- Map cells: {metrics.get('map_width_cells', 'n/a')} × {metrics.get('map_height_cells', 'n/a')}",
                f"- Occupied / free / unknown: {metrics.get('occupied_cells', 'n/a')} / {metrics.get('free_cells', 'n/a')} / {metrics.get('unknown_cells', 'n/a')}",
                f"- Occupancy update rate: {metrics.get('occupancy_update_rate_hz', 'n/a')} Hz",
                f"- RTAB processing mean / max: {metrics.get('rtab_processing_ms_mean', 'n/a')} / {metrics.get('rtab_processing_ms_max', 'n/a')} ms",
                f"- Validated scan rate: {data.get('validated_scan_rate_hz', 'n/a')} Hz",
                f"- LiDAR health: `{data.get('lidar_health', 'UNKNOWN')}`", "",
                "CPU usage is intentionally not synthesized by the mapping process; measure it externally during acceptance.",
            ]
            (self._session_dir / "comparison.md").write_text("\n".join(rows) + "\n", encoding="utf-8")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingSessionManager()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node._state in ("STARTING", "MAPPING", "DEGRADED", "FINALIZING"):
            node._terminate_owned_process()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
