#!/usr/bin/env python3
"""Keep ROS callbacks alive while a characterization operator is at stdin."""
from __future__ import annotations

import threading


class RosCallbackPump:
    """Dedicated executor thread for human-paced, safety-checked runners."""
    def __init__(self, node):
        from rclpy.executors import MultiThreadedExecutor
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.executor.shutdown()
        if self.thread.is_alive(): self.thread.join(timeout=2.0)

