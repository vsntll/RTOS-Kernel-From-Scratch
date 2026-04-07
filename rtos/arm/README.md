# Phase 8 (stretch): Cortex-M port

Retargets the context switch to real Cortex-M4 assembly and boots it
bare-metal instead of running host-native on `ucontext.h`. Same demo as
Phase 1/3 -- two tasks taking turns -- except the "switch" is now a real
`PendSV_Handler` doing manual register save/restore against actual
exception-frame layout, and preemption is a real `SysTick` ISR instead of
`SIGALRM`.

Originally booted only under Renode; QEMU is now the primary target (see
the ROS2 project plan) since the follow-on ROS2 client layer needs a real
serial link out to a host process, and `-serial` is QEMU's standard way to
do that. The firmware itself didn't need to change to move emulators, only
which UART peripheral it drives -- see the USART1-vs-USART2 note below.
Renode still boots the same ELF; its instructions are kept below as a
secondary path, not actively re-verified after the USART1 switch.

## Building

Requires `arm-none-eabi-gcc` (e.g. `sudo apt install gcc-arm-none-eabi` on
Debian/Ubuntu/WSL, or `host/setup_wsl.sh` from the repo root, which installs
it alongside the rest of the ROS2-layer toolchain):

```sh
make
```

Produces `build/kernel.elf`.

## Running in QEMU (primary)

Requires `qemu-system-arm`. Targets the `netduinoplus2` machine QEMU ships
out of the box -- an STM32F405RGT6, the same chip family (and superset of
peripherals) as Renode's `stm32f4_discovery` platform.

```sh
bash ../../host/run_qemu.sh          # UART on this terminal
bash ../../host/run_qemu.sh pty      # UART on a pty, for the ROS2-layer
                                      # host peer / micro-ros-agent to open
```

Prints `[task A] tick` / `[task B] tick` alternating every SysTick period,
same as the Renode output below.

## Running in Renode (secondary)

Requires [Renode](https://renode.io/). Targets the `stm32f4_discovery`
platform Renode ships out of the box (only USART1 and the core peripherals
are actually used -- this isn't real STM32F4 firmware, just a convenient
existing platform description with a UART already wired up).

```sh
renode boot.resc
```

`showAnalyzer sysbus.usart1` in the script opens a terminal window showing
`[task A] tick` / `[task B] tick` alternating every SysTick period. In
headless/`--console` mode it prints straight to Renode's own output
instead.

## Why USART1, not USART2

The Phase 8 demo originally drove USART2 (PA2/PA3), the pins wired to a
real STM32F4 Discovery board's ST-Link virtual COM port, which is also what
Renode's `stm32f4_discovery` platform exposes an analyzer for. QEMU's
`netduinoplus2` model instead wires its **USART1** instance to the first
`-serial` chardev (`serial_hd(0)`); there's no way to target USART2 without
a second `-serial` argument QEMU's simpler invocation doesn't need. Since
one UART is all this demo (or the ROS2 transport layer built on top of it)
needs, `uart.c` moved to USART1 (PA9/PA10, same AF7) so the same firmware
image boots identically under both emulators -- see `uart.c` for the
register-level diff (notably: pins 8-15 live in `AFRH`, not the `AFRL` the
USART2 version used, and USART1 is gated by `RCC_APB2ENR` rather than
`RCC_APB1ENR`).

## What's different from the host-native kernel

| | Host-native (`src/`) | ARM (`arm/`) |
|---|---|---|
| Context switch | `swapcontext()` (`task.c`) | `PendSV_Handler` (`context_switch.s`) |
| Tick source | `setitimer` + `SIGALRM` | `SysTick` |
| Task stack | `malloc()`'d buffer + `ucontext_t` | fixed array + hand-built exception frame (`kernel_arm.c`) |
| Scheduling | priority-aware round robin, sync primitives, sleep, canaries | round robin only -- a minimal 2-task demo, not the full kernel |

This is deliberately a small proof that the design ports to real hardware,
not a re-implementation of Phases 2-7 on top of it.
