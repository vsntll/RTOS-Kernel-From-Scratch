# RTOS Kernel From Scratch

## What this is

A preemptive, priority-based RTOS kernel written in C, running entirely as
a host-native process — no board, no simulator, no cross-compiler required
to build or run it. Context switching is implemented using POSIX primitives
(`ucontext.h`) instead of real hardware registers, and a POSIX interval
timer (`setitimer` + `SIGALRM`) stands in for a hardware tick interrupt
(the host-native equivalent of a SysTick ISR triggering `PendSV` on a real
Cortex-M part).

An optional stretch phase ports the context-switch code to real Cortex-M
assembly and boots it under Renode/QEMU — see `docs/design.md` for details
— but everything in this repo works standalone on Linux/macOS/WSL first.

## Why it exists

Most embedded engineers "use FreeRTOS" without ever having implemented a
scheduler, a context switch, or a priority-inheriting mutex themselves.
This project builds all of that from first principles: task structs and
stacks, cooperative and preemptive scheduling, priority inversion (and the
fix for it), and blocking synchronization primitives (mutex, semaphore,
message queue). `docs/design.md` documents the internal architecture in
full; this README covers how to build, run, and test it.

```
main() --> scheduler_run() --> [Task A] <--context_switch--> [Task B] <--> [Task C]
                                    |
                              SIGALRM (tick) forces preemption
```

## Prerequisites

| Tool | Required version | Notes |
|------|-------------------|-------|
| GCC or Clang | Any recent version | POSIX (`ucontext.h`, `setitimer`) support required — Linux/macOS/WSL |
| Make | Any | Builds kernel, tests, and examples |
| Valgrind | Any | For leak/stack-corruption checks (Phase 7) |
| ThreadSanitizer | Bundled with GCC/Clang | For data-race checks on sync primitives (`-fsanitize=thread`) |
| Renode / QEMU + arm-none-eabi-gcc | Optional | Only needed for the Phase 8 stretch goal (real ARM target) |

> **Note on `ucontext.h`:** deprecated on macOS but still functional; fully
> supported on Linux/WSL. If porting to a platform without it, `setjmp`/
> `longjmp` with manually managed stacks is a documented fallback (see
> `docs/design.md`).

## Building

```bash
git clone <your-repo-url>
cd rtos
make clean
make
```

Produces:
- `build/librtos.a` — the kernel itself (task/scheduler/sync primitives)
- `build/examples/producer_consumer` — demo of queue-based inter-task communication
- `build/examples/priority_inversion_demo` — demo showing inversion, then the fix
- `build/tests/*` — the test binaries described below

## Running the examples

```bash
./build/examples/producer_consumer
```

Expect interleaved output from a producer and consumer task communicating
over a bounded queue, with no dropped or duplicated items.

```bash
./build/examples/priority_inversion_demo
```

Runs once with priority inheritance disabled (demonstrating the high-priority
task getting starved by a low-priority task holding a shared lock), then
once with inheritance enabled (showing the fix). Expected output and timing
are documented in `docs/design.md`.

## Running the tests

```bash
make test
```

This builds and runs, in order:

1. `test_scheduling` — round-robin fairness, preemption under an
   intentionally non-yielding task, tick accounting.
2. `test_priority_inversion` — asserts inversion occurs without
   inheritance and is resolved with it, using timing bounds.
3. `test_sync_primitives` — mutex/semaphore/queue correctness under
   concurrent producer/consumer load.

All tests should pass with a clean exit code; failures print which
assertion tripped and the task/tick state at that point.

## Memory and concurrency validation

```bash
# Leak / stack-corruption check
valgrind --leak-check=full ./build/examples/producer_consumer

# Data race check (requires building with -fsanitize=thread)
make clean && make SANITIZE=thread
./build/tests/test_sync_primitives
```

Stack canaries are written at both ends of every task's stack at creation
and checked on every context switch; a corrupted canary aborts with the
offending task ID printed, rather than silently corrupting adjacent memory.

## Project structure

```
rtos/
  src/
    task.c / task.h        # task struct, task_create, context_switch
    scheduler.c / .h       # ready queue, scheduler_run, preemption (SIGALRM)
    sync.c / .h            # mutex, semaphore, message queue
    kernel.c               # public API: task_create, task_yield, task_sleep, etc.
  tests/
    test_scheduling.c
    test_priority_inversion.c
    test_sync_primitives.c
  examples/
    producer_consumer.c
    priority_inversion_demo.c
  docs/
    design.md              # architecture + internals writeup
  Makefile
```

## Kernel API (summary)

| Function | What it does |
|---|---|
| `task_create(fn, priority, stack_size)` | Allocates a stack, sets up initial context, adds task to the ready queue |
| `task_yield()` | Current task voluntarily gives up the CPU |
| `task_sleep(ms)` | Blocks the current task until the given number of ticks has elapsed |
| `scheduler_run()` | Starts the kernel's main loop; does not return |
| `mutex_lock(m)` / `mutex_unlock(m)` | Blocking mutual exclusion, with optional priority inheritance |
| `sem_wait(s)` / `sem_post(s)` | Counting semaphore operations |
| `queue_send(q, item)` / `queue_receive(q)` | Blocking bounded message queue |

Full parameter details, return codes, and internal data structures are in
`docs/design.md`.

## Roadmap / build phases

This kernel was built incrementally; each phase is a working, testable
milestone rather than a big-bang implementation:

- [x] Phase 1 — Task representation and context switching
- [x] Phase 2 — Cooperative scheduler
- [x] Phase 3 — Preemption via timer interrupt
- [x] Phase 4 — Priority scheduling + priority inversion fix
- [x] Phase 5 — Synchronization primitives (mutex, semaphore, queue)
- [x] Phase 6 — Timing and delays (`task_sleep`)
- [x] Phase 7 — Testing and hardening (fuzzing, stack canaries)
- [x] Phase 8 (stretch) — Port to real Cortex-M target under Renode/QEMU

## Troubleshooting

- **Tasks never seem to preempt:** confirm `SIGALRM` isn't being blocked or
  ignored elsewhere in your environment (some debuggers intercept it by
  default).
- **Random crashes only under high task counts:** almost always a stack
  size that's too small for a given task — increase the `stack_size`
  argument to `task_create()` and re-check canaries.
- **`ucontext.h` warnings on macOS:** expected; the functions are
  deprecated but still present and functional. No action needed unless
  targeting a platform where they're removed entirely.

See `docs/design.md` for a deeper architecture reference and rationale
behind each design decision (why `ucontext.h` over `setjmp`/`longjmp`, how
priority inheritance is implemented, etc).