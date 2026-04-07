# Design notes

## Phase 0 — scheduling model

Going cooperative first. A cooperative scheduler is a much smaller surface
area to get right (no signal handlers, no reentrancy concerns in the
context switch path), and it gives a working, testable kernel loop before
any preemption logic exists. Preemption gets layered on top in Phase 3 once
the ready queue and context switch primitives are proven correct.

## Platform notes

Building host-native on top of POSIX primitives (`ucontext.h`, `setitimer`/
`SIGALRM`). Developed and tested under WSL (Ubuntu, glibc) since these
APIs aren't available on native Windows.

## Repo layout

Following the structure from the project plan: everything lives under
`rtos/`, with `src/`, `tests/`, `examples/`, `docs/` beneath it.

More notes get added as later phases land.

## Phase 8 — Cortex-M port (stretch)

`arm/` retargets the context switch to real Cortex-M4 assembly
(`PendSV_Handler` in `context_switch.s`) and boots under Renode instead of
running host-native on `ucontext.h`. See `arm/README.md` for the build/run
instructions and a summary of what's different from the host-native
kernel (short version: it's a minimal 2-task round-robin proof that the
design ports to real hardware, not a reimplementation of Phases 2-7).

Two bugs surfaced only on real hardware semantics, not host-native:

- **Boot scratch stack undersized.** The very first `PendSV` needs
  somewhere valid to "save" a nonexistent outgoing context before the
  first real task is picked. That scratch area was originally sized to
  the bare 16-word exception frame (r4-r11 + hardware-pushed r0-r3, r12,
  lr, pc, xpsr) — exactly enough on paper, but it silently overflowed into
  the very next static in `.bss` (`task_sp[]`), corrupting `task_sp[1]` to
  0 before it was ever read. Found by bisecting with a memory-mapped debug
  print (`kernel_arm.c`'s `scheduler_switch_context()` printing the actual
  pointer values over UART) rather than guessing — the corruption was
  invisible until the second switch actually dereferenced the clobbered
  pointer. Fixed by giving it real headroom (64 words) rather than sizing
  it to the theoretical minimum.
- **QEMU became the primary emulation target (still Phase 8's demo).**
  `arm/uart.c` moved from USART2 to USART1 so the same firmware boots
  identically under both QEMU's `netduinoplus2` machine and Renode's
  `stm32f4_discovery` platform -- see the "Why USART1, not USART2" note in
  `arm/README.md`. This landed as groundwork for the ROS2 client layer
  (`xrce/docs/design.md`), which needs a real host-facing serial link.
- **An unrelated-looking main() edit reintroduced a hang under Renode.**
  Renaming a helper function and dropping a diagnostic UART print — pure
  refactoring, no logic change — brought back a `PendSV`/`SysTick`
  pending-but-never-serviced hang that only reproduces under Renode's
  specific emulation, not on the earlier working build. The exact
  mechanism wasn't tracked down (plausibly something in how Renode's
  block-translation checks for pending interrupts, sensitive to code size
  or layout right around the first switch); what's confirmed is that the
  verified-working `main()` structure has to stay as-is. Noted here so a
  future edit to `arm/main.c` that reintroduces a hang isn't a mystery.
