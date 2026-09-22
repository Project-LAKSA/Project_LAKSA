#!/usr/bin/env python3
"""Record the permanent LAKSA mapping regression dataset.

Run this only with a physically present operator driving manually.  Type event
labels at the prompt (early, mid, kitchen, ramp_start, ramp_end, return) and
type ``stop`` to finish the bag cleanly.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


TOPICS = (
    "/zed/zed_node/rgb/color/rect/image",
    "/zed/zed_node/depth/depth_registered",
    "/zed/zed_node/rgb/color/rect/camera_info",
    "/zed/zed_node/odom",
    "/zed/zed_node/imu/data",
    "/laksa/odometry/fused",
    "/laksa/lidar/scan_validated",
    "/tf",
    "/tf_static",
    "/laksa/fused_mapping/info",
    "/laksa/fused_mapping/mapData",
    "/map",
    "/laksa/mapping/state",
    "/joy",
    "/laksa/autonomous_enabled",
    "/laksa/brake",
)
PARAMETER_NODES = (
    "/zed/zed_node",
    "/laksa/fused_mapping/rtabmap",
    "/ekf_filter_node",
)


def run(command: list[str], *, timeout: float = 15.0) -> subprocess.CompletedProcess:
    return subprocess.run(command, capture_output=True, text=True, timeout=timeout, check=False)


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    root = Path("/home/ubuntu/laksa_datasets") / f"mapping_golden_{stamp}"
    root.mkdir(parents=True, exist_ok=False)
    parameters = root / "active_parameters"
    parameters.mkdir()

    topic_result = run(["ros2", "topic", "list", "-t", "--no-daemon", "--spin-time", "5"])
    (root / "topic_graph.txt").write_text(topic_result.stdout + topic_result.stderr, encoding="utf-8")
    available = {line.split()[0] for line in topic_result.stdout.splitlines() if line.startswith("/")}
    missing = [topic for topic in TOPICS if topic not in available]
    if missing:
        write_json(root / "INCOMPLETE.json", {"reason": "required topics missing", "topics": missing})
        raise RuntimeError(f"mapping is not ready; missing topics: {', '.join(missing)}")

    for node in PARAMETER_NODES:
        result = run(["ros2", "param", "dump", node, "--no-daemon", "--spin-time", "5"])
        name = node.strip("/").replace("/", "__") + ".yaml"
        (parameters / name).write_text(result.stdout + result.stderr, encoding="utf-8")

    repository = Path(__file__).resolve().parents[2]
    commit = run(["git", "-C", str(repository), "rev-parse", "HEAD"]).stdout.strip()
    status = run(["git", "-C", str(repository), "status", "--short"]).stdout
    packages = run([
        "dpkg-query", "-W",
        "ros-humble-rtabmap-ros", "ros-humble-zed-wrapper",
        "ros-humble-robot-localization", "ros-humble-rmw-cyclonedds-cpp",
    ]).stdout
    metadata = {
        "schema": "laksa-mapping-golden-v1",
        "created_utc": stamp,
        "topics": TOPICS,
        "git_commit": commit,
        "git_status_short": status.splitlines(),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        "cyclonedds_uri": os.environ.get("CYCLONEDDS_URI"),
        "package_versions": packages.splitlines(),
        "operator_contract": "manual Xbox only; no autonomous motion",
        "ramp": "record marker events only when operator is physically present",
    }
    write_json(root / "metadata.json", metadata)

    events = (root / "events.ndjson").open("w", encoding="utf-8", buffering=1)
    command = [
        "ros2", "bag", "record", "--output", str(root / "bag"),
        "--compression-mode", "file", "--compression-format", "zstd", *TOPICS,
    ]
    process = subprocess.Popen(command, start_new_session=True)
    print(f"Recording {root}")
    print("Enter event labels; use 'stop' to finish cleanly.")
    try:
        while process.poll() is None:
            label = input("event> ").strip()
            if not label:
                continue
            events.write(json.dumps({"utc": time.time(), "label": label}) + "\n")
            if label == "stop":
                break
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        events.close()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            return_code = process.wait(timeout=60)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            return_code = process.wait(timeout=10)

    hashes = []
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        if path.name == "SHA256SUMS":
            continue
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        hashes.append(f"{digest}  {path.relative_to(root)}")
    (root / "SHA256SUMS").write_text("\n".join(hashes) + "\n", encoding="utf-8")
    if return_code != 0:
        raise RuntimeError(f"ros2 bag record exited {return_code}")
    print(f"Golden dataset complete: {root}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
