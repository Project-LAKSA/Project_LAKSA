#!/usr/bin/env python3
"""Verify the supervisor characterization gate without publishing DriveCommand.

This is deliberately a Bool-only lifecycle probe.  It never imports the
vehicle command message and never publishes to a motion, VESC, servo, action,
or service endpoint.  The short arm-expiry observation also sends no
characterization request: the supervisor must revoke the enabled gate itself.
"""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool


ENABLE_TOPIC = "/laksa/characterization_enable"
STATE_TOPIC = "/laksa/characterization_enabled"


class GateProbe(Node):
    def __init__(self) -> None:
        super().__init__("characterization_gate_probe")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state: bool | None = None
        self._history: list[bool] = []
        self._enable = self.create_publisher(Bool, ENABLE_TOPIC, qos)
        self.create_subscription(Bool, STATE_TOPIC, self._on_state, qos)

    def _on_state(self, message: Bool) -> None:
        self._state = bool(message.data)
        self._history.append(self._state)

    @property
    def state(self) -> bool | None:
        return self._state

    @property
    def history(self) -> list[bool]:
        return list(self._history)

    def publish(self, enabled: bool) -> None:
        message = Bool()
        message.data = enabled
        self._enable.publish(message)

    def wait_for(self, expected: bool, timeout_sec: float, *, repeat: bool = False) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if repeat:
                self.publish(expected)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state is expected:
                return True
        return self.state is expected


def verify_gate() -> dict[str, object]:
    """Exercise only Bool gate transitions and return audit-ready evidence."""
    # Let Python deliver Ctrl-C before rclpy tears down the context.  The
    # finally block below can then transmit the mandatory false cleanup sample.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    probe = GateProbe()
    report: dict[str, object] = {}
    try:
        # ROS endpoint discovery can be slow immediately after a systemd ROS
        # restart.  Do not classify a late transient-local sample as a failed
        # default-off state; wait longer before issuing any enable publication.
        report["default_disabled"] = probe.wait_for(False, 10.0)
        report["enable_observed"] = probe.wait_for(True, 2.0, repeat=True)
        report["cleanup_disabled"] = probe.wait_for(False, 2.0, repeat=True)

        # No request heartbeat is published here.  This proves the supervisor's
        # own arm watchdog revokes an otherwise enabled gate after 10 seconds.
        probe.publish(True)
        if not probe.wait_for(True, 2.0, repeat=False):
            report["arm_timeout_disabled"] = False
        else:
            report["arm_timeout_disabled"] = probe.wait_for(False, 12.0)
        report["state_history"] = probe.history
        report["motion_command_published"] = False
        report["status"] = (
            "PASS" if all(report[key] for key in (
                "default_disabled", "enable_observed", "cleanup_disabled", "arm_timeout_disabled"
            )) else "FAIL"
        )
        return report
    finally:
        # Repeated false samples make cleanup robust to endpoint discovery or a
        # terminal interrupt; this remains a non-motion Bool publication.
        for _ in range(10):
            probe.publish(False)
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        result = verify_gate()
    except KeyboardInterrupt:
        print(json.dumps({"status": "INTERRUPTED_CLEANUP_SENT"}, sort_keys=True), flush=True)
        raise SystemExit(130)
    print(json.dumps(result, sort_keys=True), flush=True)
    raise SystemExit(0 if result["status"] == "PASS" else 2)
