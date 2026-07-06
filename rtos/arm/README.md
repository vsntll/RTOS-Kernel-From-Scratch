# Phase 8 (stretch): Cortex-M port

Retargets the context switch to real Cortex-M4 assembly and boots it in
Renode instead of running host-native on `ucontext.h`. Same demo as
Phase 1/3 -- two tasks taking turns -- except the "switch" is now a real
`PendSV_Handler` doing manual register save/restore against actual
exception-frame layout, and preemption is a real `SysTick` ISR instead of
`SIGALRM`.

## Building

Requires `arm-none-eabi-gcc` (e.g. `sudo apt install gcc-arm-none-eabi` on
Debian/Ubuntu/WSL):

```sh
make
```

Produces `build/kernel.elf`.

## Running in Renode

Requires [Renode](https://renode.io/). Targets the `stm32f4_discovery`
platform Renode ships out of the box (only USART2 and the core peripherals
are actually used -- this isn't real STM32F4 firmware, just a convenient
existing platform description with a UART already wired up).

```sh
renode boot.resc
```

`showAnalyzer sysbus.usart2` in the script opens a terminal window showing
`[task A] tick` / `[task B] tick` alternating every SysTick period. In
headless/`--console` mode it prints straight to Renode's own output
instead.

## What's different from the host-native kernel

| | Host-native (`src/`) | ARM (`arm/`) |
|---|---|---|
| Context switch | `swapcontext()` (`task.c`) | `PendSV_Handler` (`context_switch.s`) |
| Tick source | `setitimer` + `SIGALRM` | `SysTick` |
| Task stack | `malloc()`'d buffer + `ucontext_t` | fixed array + hand-built exception frame (`kernel_arm.c`) |
| Scheduling | priority-aware round robin, sync primitives, sleep, canaries | round robin only -- a minimal 2-task demo, not the full kernel |

This is deliberately a small proof that the design ports to real hardware,
not a re-implementation of Phases 2-7 on top of it.
