#!/usr/bin/env python3
"""Publish measurable LAKSA subsystem health without entering the drive loop."""

from __future__ import annotations

import glob
import json
import math
import os
from pathlib import Path
import socket
import time

import psutil
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from laksa_interfaces.msg import VehicleState, VescState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Image, Imu, Joy, LaserScan
from std_msgs.msg import String


class HealthMonitor(Node):
    def __init__(self) -> None:
        super().__init__("laksa_health_monitor")
        self._last = {}
        self._counts = {}
        self._state = None
        self._vesc = None
        self._mapping = {"state": "IDLE"}
        qos = QoSProfile(depth=1); qos.reliability = ReliabilityPolicy.RELIABLE; qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._summary_pub = self.create_publisher(String, "/laksa/health/summary", qos)
        self._battery_pub = self.create_publisher(BatteryState, "/laksa/battery_state", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(VehicleState, "/laksa/state", self._state_cb, qos_profile_sensor_data)
        self.create_subscription(VescState, "/laksa/vesc/state", self._vesc_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, "/laksa/imu/data", lambda _: self._touch("bno08x"), qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", lambda _: self._touch("lidar"), qos_profile_sensor_data)
        self.create_subscription(Joy, "/joy", lambda _: self._touch("xbox"), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/zed/zed_node/odom", lambda _: self._touch("zed_odom"), qos_profile_sensor_data)
        self.create_subscription(Image, "/zed/zed_node/rgb/color/rect/image", lambda _: self._touch("zed_rgb"), qos_profile_sensor_data)
        self.create_subscription(String, "/laksa/mapping/state", self._mapping_cb, 10)
        self.create_timer(1.0, self._publish)
        psutil.cpu_percent(None)

    def _touch(self, key: str) -> None:
        self._last[key] = time.monotonic(); self._counts[key] = self._counts.get(key, 0) + 1

    def _state_cb(self, msg: VehicleState) -> None:
        self._state = msg; self._touch("esp32")

    def _vesc_cb(self, msg: VescState) -> None:
        self._vesc = msg; self._touch("vesc_topic")

    def _mapping_cb(self, msg: String) -> None:
        try: self._mapping = json.loads(msg.data)
        except json.JSONDecodeError: self._mapping = {"state": "ERROR", "error": "invalid mapping state payload"}
        self._touch("mapping")

    def _age(self, key: str) -> float | None:
        return None if key not in self._last else max(0.0, time.monotonic() - self._last[key])

    def _fresh(self, key: str, timeout: float) -> bool:
        age = self._age(key); return age is not None and age <= timeout

    @staticmethod
    def _temperatures() -> dict:
        values = {}
        try:
            groups = psutil.sensors_temperatures()
        except (AttributeError, OSError, TypeError, ValueError):
            groups = {}
        for group, entries in groups.items():
            for index, entry in enumerate(entries):
                if math.isfinite(entry.current): values[f"{group}:{entry.label or index}"] = round(entry.current, 1)
        if not values:
            for temp_path in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
                try:
                    zone = Path(temp_path).parent
                    name = (zone / "type").read_text().strip()
                    value = float(Path(temp_path).read_text().strip()) / 1000.0
                    if math.isfinite(value): values[name] = round(value, 1)
                except (OSError, AttributeError, TypeError, ValueError):
                    continue
        return values

    @staticmethod
    def _gpu_percent():
        candidates = [
            "/sys/devices/platform/17000000.gpu/devfreq/17000000.gpu/load",
            "/sys/class/devfreq/17000000.gpu/load",
            "/sys/devices/gpu.0/load",
        ] + glob.glob("/sys/class/devfreq/*/load")
        for path in candidates:
            try:
                raw = float(Path(path).read_text().strip())
                if 0 <= raw <= 1000: return round(raw / 10.0, 1)
                if 0 <= raw <= 100: return round(raw, 1)
            except (OSError, ValueError): pass
        return None

    @staticmethod
    def _network() -> dict:
        interfaces = []
        for name, addresses in psutil.net_if_addrs().items():
            ipv4 = [a.address for a in addresses if a.family == socket.AF_INET and not a.address.startswith("127.")]
            if ipv4 and psutil.net_if_stats().get(name) and psutil.net_if_stats()[name].isup:
                interfaces.append({"name": name, "ipv4": ipv4})
        return {"connected": bool(interfaces), "interfaces": interfaces, "hostname": socket.gethostname()}

    def _battery(self):
        source = self._vesc or (self._state.vesc if self._state is not None else None)
        if source is None or not bool(source.telemetry_fresh): return None
        msg = BatteryState(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = "base_footprint"
        msg.voltage = float(source.input_voltage_v); msg.current = -float(source.input_current_a)
        msg.percentage = float("nan"); msg.capacity = float("nan"); msg.design_capacity = float("nan"); msg.charge = float("nan")
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if source.input_current_a > 0.1 else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN; msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        self._battery_pub.publish(msg)
        return {"available": True, "voltage_v": round(float(source.input_voltage_v), 2), "input_current_a": round(float(source.input_current_a), 2), "motor_current_a": round(float(source.motor_current_a), 2), "soc_percent": None, "soc_note": "Unavailable: no validated battery model"}

    def _publish(self) -> None:
        disk = psutil.disk_usage("/")
        mem = psutil.virtual_memory(); swap = psutil.swap_memory()
        battery = self._battery() or {"available": False, "soc_percent": None}
        vesc = self._vesc or (self._state.vesc if self._state is not None else None)
        health = {
            "zed": {"connected": self._fresh("zed_rgb", 2.0) and self._fresh("zed_odom", 2.0), "rgb_age_sec": self._age("zed_rgb"), "odom_age_sec": self._age("zed_odom")},
            "lidar": {"connected": self._fresh("lidar", 2.0), "age_sec": self._age("lidar")},
            "bno08x": {"connected": self._fresh("bno08x", 1.0), "age_sec": self._age("bno08x"), "imu_available": bool(self._state.imu_available) if self._state else False},
            "esp32": {"connected": self._fresh("esp32", 1.0), "age_sec": self._age("esp32")},
            "vesc": {"connected": bool(vesc and vesc.telemetry_fresh and self._fresh("vesc_topic", 2.0)), "age_sec": self._age("vesc_topic"), "fault_code": int(vesc.fault_code) if vesc else None, "brake_active": bool(vesc.brake_active) if vesc else None, "measured_erpm": round(float(vesc.measured_erpm), 1) if vesc else None},
            "xbox": {"connected": self._fresh("xbox", 1.0), "age_sec": self._age("xbox")},
            "mapping": self._mapping,
        }
        payload = {
            "stamp": time.time(), "health": health, "battery": battery,
            "vehicle": {"steering_target_rad": float(self._state.steering_target_rad) if self._state else None, "steering_current_rad": float(self._state.steering_current_rad) if self._state else None, "linear_velocity_mps": float(vesc.vehicle_linear_velocity_mps) if vesc else None},
            "jetson": {"cpu_percent": psutil.cpu_percent(None), "ram_percent": mem.percent, "ram_used_gb": round(mem.used / 1e9, 2), "swap_percent": swap.percent, "disk_free_gb": round(disk.free / 1e9, 1), "disk_percent": disk.percent, "gpu_percent": self._gpu_percent(), "temperatures_c": self._temperatures()},
            "network": self._network(),
        }
        try:
            encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        except ValueError:
            encoded = json.dumps(self._finite_payload(payload), separators=(",", ":"), allow_nan=False)
        self._summary_pub.publish(String(data=encoded))
        statuses = []
        for name, details in health.items():
            connected = details.get("connected", details.get("state") not in ("ERROR", None))
            level = DiagnosticStatus.OK if connected else DiagnosticStatus.WARN
            status = DiagnosticStatus(level=level, name=f"laksa_health/{name}", hardware_id=name, message="healthy" if connected else "unavailable or stale")
            status.values = [KeyValue(key=k, value=str(v)) for k, v in details.items() if not isinstance(v, (dict, list))]
            statuses.append(status)
        array = DiagnosticArray(); array.header.stamp = self.get_clock().now().to_msg(); array.status = statuses
        self._diag_pub.publish(array)

    @classmethod
    def _finite_payload(cls, value):
        if isinstance(value, dict): return {key: cls._finite_payload(item) for key, item in value.items()}
        if isinstance(value, list): return [cls._finite_payload(item) for item in value]
        if isinstance(value, float) and not math.isfinite(value): return None
        return value


def main(args=None):
    rclpy.init(args=args); node = HealthMonitor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
