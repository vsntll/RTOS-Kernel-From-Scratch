#!/usr/bin/env python3
"""Phase 12 (fault injection test suite): a byte-corrupting relay for the
QEMU<->agent serial link, proving live -- against a real, unmodified
agent, over a continuous run -- what xrce/tests/test_serial_transport.c
already proves at the unit-test level: that xrce_serial_reader_feed()'s
CRC-16/ARC check detects corrupted frames and resyncs cleanly, instead of
wedging or delivering garbled data, when corruption happens on a real
wire under real timing rather than fed to the state machine directly by
a test.

Same reasoning as pty_bridge.py (Phase 5) and udp_loss_proxy.py (Phase
7c): the property under test is what happens *on the wire* between two
real endpoints, not something either endpoint can be made to simulate
internally.

Deliberately dumb relative to host/secure_gateway.c: this proxy has no
notion of frame boundaries at all, and doesn't need one -- it flips random
bits in the raw byte stream as bytes cross it, exactly modeling a noisy
physical link (not "this project's proxy code understanding and
selectively breaking frames", which would prove nothing about real
corruption resilience).

Usage:
    python3 host/fault_corruption_proxy.py <board-pty-path> [corrupt-pct]

<board-pty-path> is the /dev/pts/N QEMU printed for its `-serial pty`
(same value that would otherwise go straight to `MicroXRCEAgent serial
-D`). [corrupt-pct] (default 2.0) is the independent per-byte probability
of a random bit flip, applied in both directions.

Prints "AGENT_PTY:/dev/pts/M" once ready; point a real `MicroXRCEAgent
serial -D /dev/pts/M -b 115200` at that path. Every corrupted byte and a
periodic summary are logged to stderr.
"""
import os
import pty
import random
import sys
import threading
import time
import tty


def relay(src, dst, tag, corrupt_pct, counters, lock):
    while True:
        try:
            data = os.read(src, 4096)
        except OSError:
            break
        if not data:
            break

        out = bytearray(data)
        corrupted_here = 0
        for i in range(len(out)):
            if random.random() * 100.0 < corrupt_pct:
                bit = 1 << random.randint(0, 7)
                out[i] ^= bit
                corrupted_here += 1

        with lock:
            counters[tag]["bytes"] += len(out)
            counters[tag]["corrupted"] += corrupted_here

        try:
            os.write(dst, bytes(out))
        except OSError:
            break


def report_loop(counters, lock, stop_event):
    while not stop_event.wait(5.0):
        with lock:
            snapshot = {tag: dict(c) for tag, c in counters.items()}
        for tag, c in snapshot.items():
            pct = (100.0 * c["corrupted"] / c["bytes"]) if c["bytes"] else 0.0
            print(f"[{tag}] {c['bytes']} bytes relayed, {c['corrupted']} corrupted "
                  f"({pct:.2f}%)", file=sys.stderr, flush=True)


def main():
    if len(sys.argv) not in (2, 3):
        print(f"usage: {sys.argv[0]} <board-pty-path> [corrupt-pct]", file=sys.stderr)
        return 2

    board_path = sys.argv[1]
    corrupt_pct = float(sys.argv[2]) if len(sys.argv) == 3 else 2.0

    board_fd = os.open(board_path, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(board_fd)

    agent_master, agent_slave = pty.openpty()
    tty.setraw(agent_slave)

    print(f"AGENT_PTY:{os.ttyname(agent_slave)}", flush=True)
    print(f"fault_corruption_proxy: board={board_path} corrupt_pct={corrupt_pct}",
          file=sys.stderr, flush=True)

    counters = {
        "uplink (board->agent)": {"bytes": 0, "corrupted": 0},
        "downlink (agent->board)": {"bytes": 0, "corrupted": 0},
    }
    lock = threading.Lock()
    stop_event = threading.Event()

    threading.Thread(target=relay, args=(board_fd, agent_master, "uplink (board->agent)",
                                          corrupt_pct, counters, lock), daemon=True).start()
    threading.Thread(target=relay, args=(agent_master, board_fd, "downlink (agent->board)",
                                          corrupt_pct, counters, lock), daemon=True).start()
    threading.Thread(target=report_loop, args=(counters, lock, stop_event), daemon=True).start()

    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        stop_event.set()
    return 0


if __name__ == "__main__":
    sys.exit(main())
