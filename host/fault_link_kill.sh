#!/usr/bin/env bash
# Phase 12 (fault injection test suite): automated version of the ad hoc
# finding already recorded in xrce/docs/design.md's Phase 6 section
# ("killing QEMU itself mid-session leaves the agent process running,
# unharmed") -- turned into a real, repeatable, assertion-based test
# instead of a one-off manual observation, and extended to the reverse
# direction (kill the agent, not just the firmware) plus automatic
# recovery.
#
# Three checks:
#   1. Live data is flowing before anything is killed (sanity baseline).
#   2. Killing the AGENT mid-session (-9, no chance to clean up) does not
#      crash or hang the firmware -- QEMU keeps running.
#   3. A freshly-started agent, pointed at the *same still-running*
#      firmware with no firmware-side changes, automatically recovers
#      live data flow -- this is ros2_demo.c's periodic re-announce
#      (xrce/docs/design.md's Phase 5 section) doing exactly the job it
#      was built for, proven under a real kill/restart rather than just
#      "the agent happened to attach late at boot".
#   4. Killing the FIRMWARE (QEMU) mid-session does not crash or hang the
#      agent.
#
# Requires ROS2 sourced and a real MicroXRCEAgent on PATH (host/setup_wsl.sh);
# run from the repo root after `make ros2_demo` in rtos/arm/.
#
# Usage:
#   bash host/fault_link_kill.sh
set -uo pipefail

ARM_DIR="$(cd "$(dirname "$0")/../rtos/arm" && pwd)"
FIRMWARE="$ARM_DIR/build/ros2_demo_node0.elf"
if [ ! -f "$FIRMWARE" ]; then
    echo "error: $FIRMWARE not found -- run 'make ros2_demo' in rtos/arm first" >&2
    exit 1
fi

PASS=0
FAIL=0
check() {
    if [ "$1" = "0" ]; then
        echo "PASS: $2"
        PASS=$((PASS + 1))
    else
        echo "FAIL: $2"
        FAIL=$((FAIL + 1))
    fi
}

QEMU_PID=""
AGENT_PID=""
cleanup() {
    [ -n "$QEMU_PID" ] && kill -9 "$QEMU_PID" 2>/dev/null
    [ -n "$AGENT_PID" ] && kill -9 "$AGENT_PID" 2>/dev/null
}
trap cleanup EXIT

read_chatter() {
    timeout 4 ros2 topic echo /chatter --once 2>/dev/null | grep -oE 'data: [0-9]+' | grep -oE '[0-9]+'
}

echo "== booting firmware + agent =="
qemu_log="$(mktemp)"
setsid qemu-system-arm -M netduinoplus2 -nographic -kernel "$FIRMWARE" -serial pty -monitor none \
    < /dev/null > "$qemu_log" 2>&1 &
QEMU_PID=$!
sleep 1
BOARD_PTY="$(grep -oE '/dev/pts/[0-9]+' "$qemu_log" | head -1)"
if [ -z "$BOARD_PTY" ]; then
    check 1 "QEMU allocated a pty"
    echo "$PASS passed, $FAIL failed"
    exit 1
fi

agent_log="$(mktemp)"
setsid MicroXRCEAgent serial -D "$BOARD_PTY" -b 115200 > "$agent_log" 2>&1 &
AGENT_PID=$!
sleep 8

before="$(read_chatter)"
if [ -n "$before" ]; then
    check 0 "baseline: live data flowing before any fault (data=$before)"
else
    check 1 "baseline: live data flowing before any fault"
fi

echo "== TEST 1: kill the agent mid-session (SIGKILL, no cleanup chance) =="
kill -9 "$AGENT_PID" 2>/dev/null
sleep 2
if kill -0 "$QEMU_PID" 2>/dev/null; then
    check 0 "firmware (QEMU) survives agent being killed mid-session"
else
    check 1 "firmware (QEMU) survives agent being killed mid-session"
fi

echo "== restarting agent against the SAME still-running firmware (no firmware changes) =="
agent_log2="$(mktemp)"
setsid MicroXRCEAgent serial -D "$BOARD_PTY" -b 115200 > "$agent_log2" 2>&1 &
AGENT_PID=$!

recovered=""
for _ in $(seq 1 12); do
    sleep 5
    v="$(read_chatter)"
    if [ -n "$v" ]; then
        recovered="$v"
        break
    fi
done
if [ -n "$recovered" ]; then
    check 0 "automatic recovery after agent restart, no firmware intervention (data=$recovered)"
else
    check 1 "automatic recovery after agent restart"
fi

echo "== TEST 2: kill the firmware (QEMU) mid-session =="
kill -9 "$QEMU_PID" 2>/dev/null
QEMU_PID=""
sleep 2
if kill -0 "$AGENT_PID" 2>/dev/null; then
    check 0 "agent survives firmware being killed mid-session"
else
    check 1 "agent survives firmware being killed mid-session"
fi

echo ""
echo "$PASS passed, $FAIL failed"
[ "$FAIL" = "0" ]
