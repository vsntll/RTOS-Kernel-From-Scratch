#!/usr/bin/env bash
# Boots rtos/arm/build/kernel.elf under QEMU's netduinoplus2 (STM32F405)
# machine, same chip family Renode's stm32f4_discovery platform models.
# USART1 (see rtos/arm/uart.c) is wired to the first -serial chardev.
#
# Usage:
#   bash host/run_qemu.sh              # UART on this terminal (stdio)
#   bash host/run_qemu.sh pty          # UART on a pty QEMU prints the path
#                                       # to -- what the Phase 1+ host peer
#                                       # / micro-ros-agent connects to
set -euo pipefail

cd "$(dirname "$0")/../rtos/arm"
ELF="build/kernel.elf"
if [ ! -f "$ELF" ]; then
    echo "build/kernel.elf not found -- run 'make' in rtos/arm first" >&2
    exit 1
fi

if [ "${1:-}" = "pty" ]; then
    # -serial pty hands the UART to its own backend, separate from
    # -nographic's monitor-on-stdio -- QEMU prints the allocated pty path
    # (e.g. /dev/pts/N) to stderr on startup.
    qemu-system-arm -M netduinoplus2 -nographic -kernel "$ELF" -serial pty
else
    # No explicit -serial here: -nographic already multiplexes the UART
    # onto this terminal's stdio. Passing -serial stdio on top of that is
    # the same backend attached twice and QEMU refuses to start.
    qemu-system-arm -M netduinoplus2 -nographic -kernel "$ELF"
fi
