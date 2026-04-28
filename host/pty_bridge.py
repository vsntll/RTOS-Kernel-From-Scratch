#!/usr/bin/env python3
"""Bidirectional pty relay/tee for debugging QEMU <-> MicroXRCEAgent serial
traffic. Not needed for normal use (QEMU's -serial pty output can point
straight at the agent's -D argument) -- this exists because the agent
holds its end of the pty exclusively, so there's normally no way to also
observe the raw bytes (including this project's human-readable UART
prints, interleaved with the framed protocol bytes) while the agent is
attached. Used while investigating the still-open Phase 5 QEMU-serial
subscribe issue -- see xrce/docs/design.md's Phase 5 section.

Creates two pty pairs and relays QEMU's traffic to the agent's and back,
logging both directions (as Python bytes reprs, one line per read()) to
/tmp/bridge.log.

Usage:
    python3 host/pty_bridge.py
    # prints two paths, e.g.:
    #   QEMU_PTY:/dev/pts/4
    #   AGENT_PTY:/dev/pts/5

    # in another terminal:
    qemu-system-arm -M netduinoplus2 -nographic -kernel rtos/arm/build/ros2_demo.elf \\
        -serial /dev/pts/4 -monitor none

    # in a third terminal:
    MicroXRCEAgent serial -D /dev/pts/5 -b 115200

    # then: tail -f /tmp/bridge.log   (or grep for specific text/bytes)
"""
import os
import pty
import threading
import time
import tty

master_a, slave_a = pty.openpty()  # QEMU's end
master_b, slave_b = pty.openpty()  # the agent's end

# Critical: pty.openpty() leaves both slaves in the default cooked/canonical
# mode (ECHO + ICANON + software flow control), not raw passthrough. Found
# the hard way: with a real agent attached, QEMU's own boot banner showed
# up echoed back on the *agent* side of the relay -- i.e. this bridge was
# silently corrupting/duplicating the binary protocol traffic it exists to
# observe, calling into question every earlier finding about "the agent
# never sends a DATA submessage" (see xrce/docs/design.md's Phase 5
# section). tty.setraw() disables all of that -- 8-bit clean passthrough,
# no echo, no line buffering, no XON/XOFF interception of control-code-like
# bytes our own framing can legitimately contain.
tty.setraw(slave_a)
tty.setraw(slave_b)

print("QEMU_PTY:" + os.ttyname(slave_a), flush=True)
print("AGENT_PTY:" + os.ttyname(slave_b), flush=True)

log = open("/tmp/bridge.log", "ab", buffering=0)


def relay(src, dst, tag):
    while True:
        try:
            data = os.read(src, 4096)
        except OSError:
            break
        if not data:
            break
        try:
            os.write(dst, data)
        except OSError:
            pass
        try:
            log.write(("[" + tag + "] ").encode() + repr(data).encode() + b"\n")
        except Exception:
            pass


threading.Thread(target=relay, args=(master_a, master_b, "Q->A"), daemon=True).start()
threading.Thread(target=relay, args=(master_b, master_a, "A->Q"), daemon=True).start()

while True:
    time.sleep(3600)
