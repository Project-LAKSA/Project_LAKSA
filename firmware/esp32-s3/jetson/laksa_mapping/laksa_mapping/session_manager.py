#!/usr/bin/env python3
"""Own one deterministic ZED-to-RTAB manual mapping session at a time."""

from __future__ import annotations

import datetime as dt
import json
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import threading
import time
from typing import Dict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rtabmap_msgs.msg import MapData
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


STATES = ("IDLE", "STARTING", "MAPPING", "FINALIZING", "COMPLETE", "ERROR")
PROFILES = ("indoor_live", "indoor_high_quality", "outdoor_structured", "outdoor_open_field")


class MappingSessionManager(Node):
    def __init__(self) -> None:
        super().__init__("mapping_session_manager")
        self.declare_parameter("sessions_root", "/home/ubuntu/laksa_mapping_sessions")
        self.declare_parameter("profile", "indoor_live")
        self.declare_parameter("record_svo", False)
        self.declare_parameter("startup_timeout_sec", 45.0)
        self.declare_parameter("data_timeout_sec", 5.0)
        self.declare_parameter("database_growth_timeout_sec", 30.0)
        self._root = Path(str(self.get_parameter("sessions_root").value))
        self._share = Path(__file__).resolve().parents[1] / "share" / "laksa_mapping"
        if not self._share.exists():
            from ament_index_python.packages import get_package_share_directory
            self._share = Path(get_package_share_directory("laksa_mapping"))
        self._profile = str(self.get_parameter("profile").value)
        self._record_svo = bool(self.get_parameter("record_svo").value)
        self._startup_timeout = float(self.get_parameter("startup_timeout_sec").value)
        self._data_timeout = float(self.get_parameter("data_timeout_sec").value)
        self._db_timeout = float(self.get_parameter("database_growth_timeout_sec").value)
        self._state = "IDLE"
        self._error = ""
        self._session_id = ""
        self._session_dir: Path | None = None
        self._db_path: Path | None = None
        self._svo_path: Path | None = None
        self._process: subprocess.Popen | None = None
        self._process_log = None
        self._started_monotonic = 0.0
        self._last: Dict[str, float] = {}
        self._finalizing = False
        self._lock = threading.RLock()
        qos_state = QoSProfile(depth=1)
        qos_state.reliability = ReliabilityPolicy.RELIABLE
        qos_state.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_pub = self.create_publisher(String, "/laksa/mapping/state", qos_state)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        cb = ReentrantCallbackGroup()
        self.create_subscription(String, "/laksa/mapping/profile_request", self._profile_cb, 10, callback_group=cb)
        self.create_subscription(Bool, "/laksa/mapping/record_svo_request", self._record_cb, 10, callback_group=cb)
        self.create_subscription(Image, "/zed/zed_node/rgb/color/rect/image", lambda _: self._touch("rgb"), qos_profile_sensor_data)
        self.create_subscription(Image, "/zed/zed_node/depth/depth_registered", lambda _: self._touch("depth"), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/zed/zed_node/odom", lambda _: self._touch("odom"), qos_profile_sensor_data)
        self.create_subscription(MapData, "/zed_rtabmap/mapData", lambda _: self._touch("map"), qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/zed_rtabmap/map", lambda _: self._touch("occupancy"), 10)
        self.create_service(Trigger, "/laksa/mapping/start", self._start_cb, callback_group=cb)
        self.create_service(Trigger, "/laksa/mapping/stop", self._stop_cb, callback_group=cb)
        self.create_timer(0.5, self._watchdog, callback_group=cb)
        self.create_timer(1.0, self._publish)
        self._root.mkdir(parents=True, exist_ok=True)
        self._publish()

    def _touch(self, key: str) -> None:
        self._last[key] = time.monotonic()

    def _profile_cb(self, msg: String) -> None:
        if msg.data in PROFILES and self._state in ("IDLE", "COMPLETE", "ERROR"):
            self._profile = msg.data

    def _record_cb(self, msg: Bool) -> None:
        if self._state in ("IDLE", "COMPLETE", "ERROR"):
            self._record_svo = bool(msg.data)

    def _conflicting_nodes(self) -> list[str]:
        names = {f"{ns.rstrip('/')}/{name}".replace("//", "/") for name, ns in self.get_node_names_and_namespaces()}
        guarded = ("/zed/zed_node", "/zed_rtabmap/rgbd_sync", "/zed_rtabmap/rtabmap")
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
            self._begin_session()
            response.success = True
            response.message = f"Starting mapping session {self._session_id}"
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
        text = (self._share / "config" / "rtabmap_common.yaml").read_text(encoding="utf-8")
        text += f"    database_path: {self._db_path}\n"
        rtab_config.write_text(text, encoding="utf-8")
        self._process_log = open(self._session_dir / "logs" / "mapping.log", "a", encoding="utf-8", buffering=1)
        cmd = ["ros2", "launch", "laksa_mapping", "mapping_stack.launch.py", f"zed_config:={zed_config}", f"rtab_config:={rtab_config}"]
        self._process = subprocess.Popen(cmd, stdout=self._process_log, stderr=subprocess.STDOUT, env=os.environ.copy(), start_new_session=True)
        self._started_monotonic = time.monotonic()
        self._last.clear()
        self._error = ""
        self._state = "STARTING"
        self._write_metadata("STARTING")
        self._publish()

    def _stop_cb(self, _request, response):
        with self._lock:
            if self._state not in ("STARTING", "MAPPING"):
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
        cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(target), "--ros-args", "-r", "map:=/zed_rtabmap/map"]
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=25)
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
            if self._state not in ("STARTING", "MAPPING"):
                return
            now = time.monotonic()
            if self._process and self._process.poll() is not None:
                self._hard_fail(f"Mapping launch exited with code {self._process.returncode}")
                return
            if self._state == "STARTING":
                required = ("rgb", "depth", "odom", "map")
                if all(now - self._last.get(key, 0.0) < 3.0 for key in required):
                    if self._record_svo and not self._call_svo(True):
                        self._hard_fail("ZED SVO recording failed to start")
                        return
                    self._state = "MAPPING"
                    self._publish()
                elif now - self._started_monotonic > self._startup_timeout:
                    self._hard_fail("Mapping topics did not become healthy before startup timeout")
                return
            stale = [key for key in ("rgb", "depth", "odom", "map") if now - self._last.get(key, 0.0) > self._data_timeout]
            if stale:
                self._hard_fail("Stale mapping signals: " + ", ".join(stale))
                return
            if self._db_path and now - self._started_monotonic > self._db_timeout and (not self._db_path.exists() or self._db_path.stat().st_size < 100000):
                self._hard_fail("RTAB database did not grow after mapping startup")

    def _hard_fail(self, message: str) -> None:
        if self._finalizing:
            return
        self._error = message
        self._state = "FINALIZING"
        self._finalizing = True
        threading.Thread(target=self._finalize, args=(True,), daemon=True).start()

    def _payload(self) -> dict:
        elapsed = 0.0 if not self._started_monotonic else time.monotonic() - self._started_monotonic
        return {
            "state": self._state, "session_id": self._session_id, "profile": self._profile,
            "profile_status": "provisional_unvalidated" if self._profile.startswith("outdoor_") else "validated",
            "elapsed_sec": round(elapsed, 1), "db_path": str(self._db_path or ""),
            "svo_enabled": self._record_svo, "svo_path": str(self._svo_path or ""), "error": self._error,
        }

    def _publish(self) -> None:
        payload = self._payload()
        self._state_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        level = DiagnosticStatus.ERROR if self._state == "ERROR" else (DiagnosticStatus.WARN if self._state in ("STARTING", "FINALIZING") else DiagnosticStatus.OK)
        status = DiagnosticStatus(level=level, name="laksa_mapping/session", hardware_id="zed2i-rtabmap", message=self._error or self._state)
        status.values = [KeyValue(key=k, value=str(v)) for k, v in payload.items()]
        array = DiagnosticArray(); array.header.stamp = self.get_clock().now().to_msg(); array.status = [status]
        self._diag_pub.publish(array)

    def _write_metadata(self, result: str, extra: dict | None = None) -> None:
        if not self._session_dir:
            return
        data = self._payload(); data.update({"result": result, "updated_utc": dt.datetime.now(dt.timezone.utc).isoformat(), "architecture": "ZED2i VIO + RGB-D -> rgbd_sync -> RTAB-Map", "autonomous_motion": False})
        if extra: data.update(extra)
        tmp = self._session_dir / "metadata.json.tmp"
        tmp.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        tmp.replace(self._session_dir / "metadata.json")


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
        if node._state in ("STARTING", "MAPPING", "FINALIZING"):
            node._terminate_owned_process()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
