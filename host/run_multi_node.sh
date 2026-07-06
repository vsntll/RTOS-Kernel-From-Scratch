#!/usr/bin/env bash
# Phase 10: N separately-built rtos/arm/ros2_demo.c firmware images, each
# booted under its own QEMU (netduinoplus2) instance, all talking to ONE
# `MicroXRCEAgent multiserial` process -- N distinct ROS2 nodes sharing one
# agent, not N separate agents each with its own happy-path client.
#
# What makes this work, ground-truthed against the real agent's own
# argument parser (include/uxr/agent/utils/ArgumentParser.hpp in the
# eProsima source this project builds against -- not guessed):
#   - `multiserial`'s `-D` takes ONE quoted, space-separated string of
#     device paths (`iss >> s` word-splitting), NOT repeated `-D` flags
#     (those silently keep only one) and NOT comma-separated (parsed as
#     one literal bogus path). See xrce/docs/design.md's Phase 10 section.
#   - Each QEMU instance's `-serial pty` already allocates its own
#     independent pty pair; no host-side proxy/relay is needed to fan N
#     boards into one agent, unlike the 1:1-only `host/udp_loss_proxy.py`.
#   - Firmware identity (client_key, participant name, every topic name)
#     comes from `rtos/arm/ros2_demo.c`'s NODE_ID, threaded through by
#     `rtos/arm/Makefile`'s `NODE_ID=` build variable -- see that file.
#
# Usage:
#   bash host/run_multi_node.sh [N]        # N boards, default 3
#
# Prints each node's topics (rt/chatter_<id>, rt/setpoint_<id>,
# rt/pong_<id>) and leaves everything running in the foreground; Ctrl-C
# tears down every QEMU instance and the agent.
set -euo pipefail

N="${1:-3}"
if ! [[ "$N" =~ ^[0-9]+$ ]] || [ "$N" -lt 1 ]; then
    echo "usage: $0 [N]  (N boards, positive integer, default 3)" >&2
    exit 1
fi

ARM_DIR="$(cd "$(dirname "$0")/../rtos/arm" && pwd)"
QEMU_PIDS=()
AGENT_PID=""
QEMU_LOGS=()

cleanup() {
    echo "-- tearing down agent + $N QEMU instance(s) --" >&2
    [ -n "$AGENT_PID" ] && kill "$AGENT_PID" 2>/dev/null || true
    for pid in "${QEMU_PIDS[@]:-}"; do
        [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

echo "== building $N firmware variants (NODE_ID=1..$N) =="
for i in $(seq 1 "$N"); do
    make -C "$ARM_DIR" ros2_demo "NODE_ID=$i" >/dev/null
done

echo "== booting $N QEMU instance(s), each on its own pty =="
for i in $(seq 1 "$N"); do
    log="$(mktemp)"
    QEMU_LOGS+=("$log")
    # setsid: -nographic's monitor otherwise dies with the launching shell
    # (same fix host/run_qemu.sh and the README's manual instructions use).
    setsid qemu-system-arm -M netduinoplus2 -nographic \
        -kernel "$ARM_DIR/build/ros2_demo_node$i.elf" -serial pty -monitor none \
        < /dev/null > "$log" 2>&1 &
    QEMU_PIDS+=("$!")
done

echo "== waiting for QEMU to allocate each pty =="
DEVS=()
for i in $(seq 1 "$N"); do
    log="${QEMU_LOGS[$((i-1))]}"
    dev=""
    for _ in $(seq 1 50); do
        dev="$(grep -oE '/dev/pts/[0-9]+' "$log" 2>/dev/null | head -1 || true)"
        [ -n "$dev" ] && break
        sleep 0.2
    done
    if [ -z "$dev" ]; then
        echo "ERROR: node $i's QEMU never allocated a pty (see $log)" >&2
        exit 1
    fi
    echo "  node $i -> $dev"
    DEVS+=("$dev")
done

DEV_LIST="${DEVS[*]}"
echo "== starting one MicroXRCEAgent multiserial process for all $N boards =="
echo "   MicroXRCEAgent multiserial -D \"$DEV_LIST\" -b 115200"
MicroXRCEAgent multiserial -D "$DEV_LIST" -b 115200 &
AGENT_PID=$!

echo "== running. Try in another terminal: =="
echo "     ros2 topic list                 # rt/chatter_1..${N}, rt/setpoint_1..${N}, rt/pong_1..${N}"
echo "     ros2 topic echo /chatter_1      # node 1's independent counter"
echo "     ros2 topic pub /setpoint_2 std_msgs/msg/Int32 '{data: 42}' --once   # only /pong_2 replies"
echo "Ctrl-C to stop everything."
wait "$AGENT_PID"
