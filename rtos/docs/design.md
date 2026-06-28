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
