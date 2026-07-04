#!/usr/bin/env python3
"""Phase 9 deliverable: a live, htop-style terminal UI for the RTOS,
rendering /rtos/diagnostics (Phase 8) in real time -- task states,
priorities, stack high-water marks, scheduler stats, queue depths, and
middleware stats, all updating live as host/live_diagnostics_demo.c
publishes them.

Python + the standard-library `curses` module (a real binding to the same
ncurses library the phase brief names, not a from-scratch reimplementation)
rather than C: this project's WSL image has the ncurses runtime libraries
but not libncurses-dev (no non-interactive sudo available to install it in
this environment), while Python's curses module needs no separate
package. Uses `rclpy` directly rather than this project's own hand-rolled
xrce/ client: Phase 8 already chose "Option A -- piggyback on ROS2" for
/rtos/diagnostics, meaning it's a real, standard
diagnostic_msgs/msg/DiagnosticArray topic at this point, and consuming a
real ROS2 topic with the real ROS2 client library is the idiomatic choice,
not a compromise -- this tool has no involvement in the RTOS's own
embedded protocol stack the way host/*.c demos do.

Usage:
    source /opt/ros/*/setup.bash
    python3 host/rtos_top.py
    # elsewhere: run host/live_diagnostics_demo.c (Phase 8) and, for the
    # Phase 7d priority-load scenario, host/live_priority_demo.c plus real
    # `ros2 topic pub -r` load, so there's something live to watch update.
"""
import curses
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node

LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
LEVEL_COLOR_PAIR = {0: 1, 1: 2, 2: 3, 3: 2}


class DiagnosticsWatcher(Node):
    def __init__(self):
        super().__init__("rtos_top")
        self.lock = threading.Lock()
        self.latest = None
        self.last_update_wall = None
        self.message_count = 0
        self.create_subscription(DiagnosticArray, "/rtos/diagnostics", self._on_diagnostics, 10)

    def _on_diagnostics(self, msg):
        with self.lock:
            self.latest = msg
            self.last_update_wall = time.time()
            self.message_count += 1

    def snapshot(self):
        with self.lock:
            return self.latest, self.last_update_wall, self.message_count


def kv_dict(status):
    return {kv.key: kv.value for kv in status.values}


def draw(stdscr, watcher):
    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)   # OK
    curses.init_pair(2, curses.COLOR_YELLOW, -1)  # WARN/STALE
    curses.init_pair(3, curses.COLOR_RED, -1)     # ERROR
    curses.init_pair(4, curses.COLOR_CYAN, -1)    # headers

    while True:
        c = stdscr.getch()
        if c in (ord("q"), ord("Q")):
            return

        msg, last_wall, count = watcher.snapshot()
        stdscr.erase()
        max_y, max_x = stdscr.getmaxyx()

        stdscr.addstr(0, 0, "rtos_top", curses.color_pair(4) | curses.A_BOLD)
        age = f"{time.time() - last_wall:.1f}s ago" if last_wall else "never"
        stdscr.addstr(0, 12, f"messages received: {count}   last update: {age}   (q to quit)")

        if msg is None:
            stdscr.addstr(2, 0, "waiting for /rtos/diagnostics ...")
            stdscr.refresh()
            time.sleep(0.05)
            continue

        row = 2
        tasks = [s for s in msg.status if s.name.startswith("task/")]
        others = [s for s in msg.status if not s.name.startswith("task/")]

        if tasks and row < max_y:
            header = f"{'TASK':<20}{'STATE':<12}{'PRIO':>6}{'STACK HWM':>12}{'TICKS RUN':>12}{'TICKS RDY':>12}"
            stdscr.addstr(row, 0, header[:max_x], curses.color_pair(4) | curses.A_BOLD)
            row += 1
            for st in tasks:
                if row >= max_y:
                    break
                kv = kv_dict(st)
                name = st.name[len("task/"):]
                line = (
                    f"{name:<20}{st.message:<12}{kv.get('priority', '?'):>6}"
                    f"{kv.get('stack_high_water_mark_bytes', '?'):>12}"
                    f"{kv.get('ticks_run', '?'):>12}{kv.get('ticks_ready', '?'):>12}"
                )
                pair = LEVEL_COLOR_PAIR.get(st.level, 0)
                stdscr.addstr(row, 0, line[:max_x], curses.color_pair(pair))
                row += 1
            row += 1

        for st in others:
            if row >= max_y:
                break
            kv = kv_dict(st)
            pair = LEVEL_COLOR_PAIR.get(st.level, 0)
            stdscr.addstr(row, 0, f"{st.name}: {st.message}"[:max_x],
                          curses.color_pair(4) | curses.A_BOLD)
            row += 1
            for k, v in kv.items():
                if row >= max_y:
                    break
                stdscr.addstr(row, 2, f"{k}: {v}"[: max_x - 2], curses.color_pair(pair))
                row += 1
            row += 1

        stdscr.refresh()
        time.sleep(0.05)


def spin_until_shutdown(node):
    # rclpy.spin() raises if rclpy.shutdown() runs on the main thread while
    # this (daemon) thread is mid-spin_once() -- expected on exit, not a
    # real error, so it's swallowed here rather than left to print an ugly
    # traceback after the terminal's already been torn down.
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    except Exception as exc:  # noqa: BLE001 -- deliberately broad, see comment above
        if "context is not valid" not in str(exc):
            raise


def main():
    rclpy.init()
    watcher = DiagnosticsWatcher()
    spin_thread = threading.Thread(target=spin_until_shutdown, args=(watcher,), daemon=True)
    spin_thread.start()
    try:
        curses.wrapper(draw, watcher)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
