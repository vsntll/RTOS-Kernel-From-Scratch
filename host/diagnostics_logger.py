#!/usr/bin/env python3
"""Logs /rtos/diagnostics (Phase 8) to disk in two structured formats
alongside whatever's already consuming it live (host/rtos_top.py, `ros2
topic echo`, etc.) -- so there's a real, growing dataset on disk any time
one of this project's demos is running, ready for offline analysis or
training something later (a scheduler-behavior classifier, an anomaly
detector over task stats, whatever), without having to re-run anything or
capture traffic after the fact.

Same rclpy-over-a-real-topic approach as host/rtos_top.py (Phase 9), same
reasoning: /rtos/diagnostics is a real, standard
diagnostic_msgs/msg/DiagnosticArray topic since Phase 8's "piggyback on
ROS2" choice, so consuming it with the real ROS2 client library is the
idiomatic choice here too, not a compromise -- this tool has no
involvement in the RTOS's own embedded protocol stack.

Two output formats, both append-only and safe to tail while a demo is
still running:

  <prefix>.jsonl -- one JSON object per line, one line per DiagnosticArray
    message received, preserving the full nested structure (every
    status's name/level/message/hardware_id and its full key/value dict).
    Best when you want the complete snapshot, or a variable/unknown set of
    keys per status (this project's different demos publish different
    task names and different key sets).

  <prefix>.csv -- the same data flattened into "tidy" (long) form: one row
    per (timestamp, status, key, value) tuple, rather than one row per
    snapshot with a fixed column per key. A fixed-width CSV would need a
    column schema decided in advance and would silently break (or need
    re-deciding) whenever a different demo publishes a different set of
    task names/keys -- tidy form sidesteps that entirely and is also the
    layout pandas/most ML tooling wants directly (`df.pivot_table(...)`
    turns this into a wide per-timestamp table trivially whenever that's
    what's actually needed, but going the other direction from a
    fixed-schema CSV is much harder).

Usage:
    source /opt/ros/*/setup.bash
    python3 host/diagnostics_logger.py [output_prefix]
    # default output_prefix: /tmp/rtos_diagnostics_<unix_timestamp>
    # elsewhere: run any demo that publishes /rtos/diagnostics
    #   (host/live_diagnostics_demo.c, host/live_priority_demo.c,
    #    host/live_fault_demo.c, ...)
"""
import csv
import json
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node


class DiagnosticsLogger(Node):
    def __init__(self, jsonl_path, csv_path):
        super().__init__("diagnostics_logger")
        self.jsonl_file = open(jsonl_path, "a", buffering=1)
        self.csv_file = open(csv_path, "a", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        if self.csv_file.tell() == 0:
            self.csv_writer.writerow(
                ["timestamp", "hardware_id", "status_name", "level", "message", "key", "value"])
            self.csv_file.flush()
        self.message_count = 0
        self.create_subscription(DiagnosticArray, "/rtos/diagnostics", self._on_diagnostics, 10)
        self.get_logger().info(f"logging to {jsonl_path} and {csv_path}")

    def _on_diagnostics(self, msg):
        timestamp = time.time()

        record = {
            "timestamp": timestamp,
            "status": [
                {
                    "name": s.name,
                    "level": int(s.level[0]) if isinstance(s.level, (bytes, bytearray)) else int(s.level),
                    "message": s.message,
                    "hardware_id": s.hardware_id,
                    "values": {kv.key: kv.value for kv in s.values},
                }
                for s in msg.status
            ],
        }
        self.jsonl_file.write(json.dumps(record) + "\n")

        for s in record["status"]:
            for key, value in s["values"].items():
                self.csv_writer.writerow(
                    [timestamp, s["hardware_id"], s["name"], s["level"], s["message"], key, value])
        self.csv_file.flush()

        self.message_count += 1
        if self.message_count % 20 == 0:
            self.get_logger().info(f"{self.message_count} messages logged")

    def close(self):
        self.jsonl_file.close()
        self.csv_file.close()


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else f"/tmp/rtos_diagnostics_{int(time.time())}"
    jsonl_path = prefix + ".jsonl"
    csv_path = prefix + ".csv"

    rclpy.init()
    logger = DiagnosticsLogger(jsonl_path, csv_path)
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.close()
        logger.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
