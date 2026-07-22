#!/usr/bin/env python3
"""Phase 13: measures real host<->micro-ROS round-trip latency against the
*official, unmodified* micro-ROS `ping_pong` demo
(micro-ROS-demos/rclc/ping_pong, built via `micro_ros_setup`'s "host"
platform target -- the standard, hardware-free way to validate/benchmark
real micro-ROS), using the exact same methodology
host/bench_latency.c already uses against this project's own from-scratch
RTOS implementation: publish a timestamped message, wait for the matching
echo, time the round trip, repeat, report p50/p95/mean/max.

Same real agent, same UDP transport, same port (127.0.0.1:8888) both
implementations were measured through -- the ping_pong demo's own
colcon.meta bakes in exactly that address, confirmed in
xrce/docs/design.md's Phase 13 section, so this is a genuinely
apples-to-apples comparison of "the real thing" vs. this project's
from-scratch implementation, not an approximation of one.

The real ping_pong demo doesn't correlate replies to a specific outside
caller -- it echoes any ping it didn't send itself onto /microROS/pong.
This script exploits exactly that: publish our own uniquely-tagged
std_msgs/Header ping, and the running (unmodified) ping_pong process
echoes it back, timestamped from our own send.

Usage:
    source /opt/ros/*/setup.bash
    # agent already running: MicroXRCEAgent udp4 -p 8888
    # ping_pong already running: RMW_IMPLEMENTATION=rmw_microxrcedds \\
    #   ~/microros_ws/install/micro_ros_demos_rclc/lib/micro_ros_demos_rclc/ping_pong/ping_pong
    python3 host/bench_microros.py [num_trials]
"""
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Header


class MicroRosBench(Node):
    def __init__(self):
        super().__init__("bench_microros")
        self.pending_id = None
        self.sent_wall = None
        self.latencies_ms = []
        self.pub = self.create_publisher(Header, "/microROS/ping",
                                          QoSPresetProfiles.SENSOR_DATA.value)
        self.create_subscription(Header, "/microROS/pong", self._on_pong,
                                  QoSPresetProfiles.SENSOR_DATA.value)

    def _on_pong(self, msg):
        if self.pending_id is not None and msg.frame_id == self.pending_id:
            elapsed_ms = (time.monotonic() - self.sent_wall) * 1000.0
            self.latencies_ms.append(elapsed_ms)
            self.pending_id = None

    def ping_once(self, seq, timeout_s=2.0):
        msg = Header()
        msg.frame_id = f"bench_{seq}"
        self.pending_id = msg.frame_id
        self.sent_wall = time.monotonic()
        self.pub.publish(msg)

        deadline = time.monotonic() + timeout_s
        while self.pending_id is not None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.pending_id is not None:
            self.pending_id = None
            return False
        return True


def main():
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    rclpy.init()
    node = MicroRosBench()

    print(f"pinging real micro-ROS ping_pong over {trials} trials...")
    lost = 0
    for i in range(trials):
        if not node.ping_once(i):
            lost += 1
        time.sleep(0.05)

    lat = sorted(node.latencies_ms)
    if lat:
        p50 = statistics.median(lat)
        p95 = lat[min(len(lat) - 1, int(len(lat) * 0.95))]
        print(f"received: {len(lat)}/{trials} (lost: {lost})")
        print(f"p50={p50:.2f}ms p95={p95:.2f}ms mean={statistics.mean(lat):.2f}ms "
              f"min={min(lat):.2f}ms max={max(lat):.2f}ms")
    else:
        print(f"received: 0/{trials} -- no pongs arrived, check ping_pong is running")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
