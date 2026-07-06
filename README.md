# RTOS Kernel From Scratch

## What this is

A preemptive, priority-based RTOS kernel written in C, running entirely as
a host-native process — no board, no simulator, no cross-compiler required
to build or run it. Context switching is implemented using POSIX primitives
(`ucontext.h`) instead of real hardware registers, and a POSIX interval
timer (`setitimer` + `SIGALRM`) stands in for a hardware tick interrupt
(the host-native equivalent of a SysTick ISR triggering `PendSV` on a real
Cortex-M part).

The context-switch code is also ported to real Cortex-M4 assembly and
boots under Renode (`rtos/arm/`) — a `PendSV_Handler` doing manual
register save/restore and a real `SysTick` ISR driving preemption,
instead of `ucontext.h`/`SIGALRM`. See `rtos/arm/README.md` for details.
Everything else in this repo is host-native and needs no board, simulator,
or cross-compiler.

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

| Tool | Notes |
|------|-------|
| `gcc` | Makefile invokes `gcc` directly. Needs POSIX (`ucontext.h`, `setitimer`) — Linux or WSL. Developed and tested under WSL (Ubuntu); not tested on macOS, where `ucontext.h` is present but deprecated. |
| `make` | Builds the kernel, tests, and examples. |
| `arm-none-eabi-gcc` + [Renode](https://renode.io/) | Only for the Phase 8 Cortex-M port (`rtos/arm/`) — not needed for anything else in this repo. |

AddressSanitizer/UndefinedBehaviorSanitizer (`-fsanitize=address,undefined`,
bundled with GCC — just pass `SANITIZE=address,undefined`) is what this
project was actually validated with; see [Memory and concurrency
validation](#memory-and-concurrency-validation) for why Valgrind and
ThreadSanitizer aren't the right tools here despite being the "usual"
answer.

## Building

```bash
git clone <your-repo-url>
cd RTOS-Kernel-From-Scratch/rtos
make clean
make
```

Builds `build/librtos.a` (the kernel: task/scheduler/sync primitives)
plus every test and example binary, under `build/tests/` and
`build/examples/` respectively. `make` only builds — it doesn't run
anything; see below for that.

## Running the examples

```bash
make examples
./build/examples/manual_switch_demo
./build/examples/producer_consumer
./build/examples/priority_inversion_demo
```

- **`manual_switch_demo`** — Phase 1: two tasks incrementing a counter and
  printing, switching between each other by calling `context_switch()`
  directly. No scheduler involved yet.
- **`producer_consumer`** — one producer and one consumer moving 20 items
  through a capacity-4 queue, blocking in both directions (producer blocks
  when full, consumer blocks when empty). Expect interleaved
  `[producer] sending N` / `[consumer] received N` output, nothing dropped
  or duplicated.
- **`priority_inversion_demo`** — runs the classic scenario once with
  priority inheritance disabled (a medium-priority task starves the low
  task holding a mutex the high task needs, so high is stuck too) and once
  with it enabled (low inherits high's priority the moment high blocks, so
  it finishes almost immediately). Prints a narrated play-by-play with
  actual tick counts for each run.

## Running the tests

```bash
make test
```

Builds and runs every test in `tests/`, in this order:

1. **`test_harness`** — a small harness that boots the scheduler fresh
   (via `scheduler_reset()`) for each of three independent cases: round
   robin fairness, priority ordering, and confirming a reset really does
   start clean.
2. **`test_preemption`** — a non-yielding busy-loop task still gets
   preempted by the tick timer, and another task still gets to run.
3. **`test_priority_inversion`** — asserts inversion occurs without
   priority inheritance and is resolved with it, using tick-count bounds.
4. **`test_scheduling`** — 4 tasks round-robin fairly under `task_yield()`.
5. **`test_stress`** — 600 short-lived tasks across 30 waves; nothing
   crashes and canaries stay intact throughout.
6. **`test_sync_primitives`** — a producer/consumer pair through a queue
   smaller than the item count, asserting every item arrives exactly once
   and in order.
7. **`test_timing`** — `task_sleep()` wakes a task on schedule, and
   per-task run/ready tick stats accumulate sensibly.

Each prints a `PASS: ...` line and exits 0 on success; a failed `assert()`
aborts with the file/line and the C library's usual `Assertion failed`
message.

## Memory and concurrency validation

```bash
make clean && make SANITIZE=address,undefined
make test
./build/examples/manual_switch_demo   # etc. for the other examples
```

This is what actually stood in for Valgrind and ThreadSanitizer in this
project's environment (no Valgrind installed, and no passwordless sudo to
add it):

- **AddressSanitizer** catches the leak/overflow/use-after-free class of
  bug Valgrind would (`test_stress.c`'s 600 create/destroy cycles are the
  main leak-check target). ASan warns that it "doesn't fully support
  makecontext/swapcontext" — expected, and no false positives were seen
  in practice across many runs.
- **ThreadSanitizer doesn't apply here.** Every "task" is a cooperative or
  signal-preempted green thread on a single real OS thread — there's no
  actual concurrent memory access for TSan to instrument, so a clean TSan
  run wouldn't prove much. `test_sync_primitives.c` documents this; the
  correctness check that actually matters (every item arrives exactly
  once, in order, through a queue too small to avoid blocking) is done via
  plain assertions instead.

Each task's stack has a guard region written at its bottom (low-address
end) at creation time and checked every time `scheduler_run()`'s dispatch
loop regains control from a task — a corrupted canary aborts immediately
with the offending task's name and ID. There's deliberately no "top"
guard: `task_canary_ok()` explains why one doesn't work at that end of a
downward-growing stack (`makecontext()` itself writes there before the
task ever runs once).

## Running the Cortex-M port (Phase 8)

Separate toolchain, separate directory (`rtos/arm/`), not built by the
top-level Makefile:

```bash
cd rtos/arm
make
renode boot.resc
```

`boot.resc` loads the ELF onto Renode's bundled `stm32f4_discovery`
platform and opens a UART analyzer window showing `[task A] tick` /
`[task B] tick` alternating every ~111ms, switched by a real
`SysTick`-triggered `PendSV_Handler` instead of `SIGALRM`/`swapcontext()`.
In headless (`--console`) mode the same output prints straight to
Renode's own log instead of a window. See `rtos/arm/README.md` for what's
different from the host-native kernel, and `docs/design.md` for two bugs
that only showed up under real hardware exception semantics.

## Project structure

```
rtos/
  src/
    task.c / task.h        # task struct, task_create, context_switch, stack canaries
    scheduler.c / .h       # ready queue, scheduler_run, preemption (SIGALRM), task_sleep
    sync.c / .h            # mutex (+ priority inheritance), semaphore, message queue
    kernel.c / kernel.h    # public API: task_spawn (create + register in one call)
  tests/
    test_harness.c            # fresh-scheduler-boot-per-case harness
    test_preemption.c         # non-yielding task still gets preempted
    test_priority_inversion.c # inversion demonstrated, then fixed
    test_scheduling.c         # cooperative round-robin fairness
    test_stress.c             # 600 short-lived tasks, no leaks/corruption
    test_sync_primitives.c    # producer-consumer queue correctness
    test_timing.c             # task_sleep() + run/ready stats
  examples/
    manual_switch_demo.c      # Phase 1: manual context_switch(), no scheduler
    producer_consumer.c
    priority_inversion_demo.c
  docs/
    design.md              # architecture + internals writeup, including Phase 8 notes
  arm/                      # Phase 8: Cortex-M4 port, boots under Renode (own README/Makefile)
  Makefile
```

## Kernel API (summary)

| Function | What it does |
|---|---|
| `task_spawn(name, entry, arg, priority)` | Creates a task with the default stack size and registers it with the scheduler in one call — the usual way to start a task |
| `task_create(name, entry, arg, priority, stack_size, return_ctx)` | Lower-level: allocates a stack and sets up the initial context without registering it anywhere |
| `task_yield()` | Current task voluntarily gives up the CPU |
| `task_sleep(ms)` | Blocks the current task until the given number of ticks has elapsed |
| `scheduler_run()` | Starts the kernel's main loop; returns once every task has terminated |
| `scheduler_enable_preemption(tick_ms, ticks_per_slice)` / `scheduler_disable_preemption()` | Starts/stops the `SIGALRM`-driven tick that forces context switches |
| `scheduler_reset()` | Clears ready-queue bookkeeping so a test harness can boot the scheduler fresh per case |
| `mutex_init(m, enable_priority_inheritance)`, `mutex_lock(m)` / `mutex_unlock(m)` | Blocking mutual exclusion, with optional priority inheritance |
| `sem_init(s, initial_count)`, `sem_wait(s)` / `sem_post(s)` | Counting semaphore operations |
| `queue_init(q, buffer, capacity)`, `queue_send(q, item)` / `queue_receive(q)` | Blocking bounded message queue |
| `task_canary_ok(task)` | Checks a task's stack guard region; used internally on every dispatch |

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
- [x] Phase 8 (stretch) — Cortex-M4 port, boots under Renode

## Troubleshooting

- **Tasks never seem to preempt:** confirm `SIGALRM` isn't being blocked or
  ignored elsewhere in your environment (some debuggers intercept it by
  default).
- **Random crashes only under high task counts:** almost always a stack
  size that's too small for a given task — increase the `stack_size`
  argument to `task_create()` and re-check canaries.
- **A test hangs intermittently under active preemption:** if you add a
  task that calls `task_yield()` in a very tight loop while
  `scheduler_enable_preemption()` is active, see the note above
  `block_alarm()` in `scheduler.c` — this narrows a real race but doesn't
  provably close it. Existing tests avoid the pattern; new ones should too
  (or should rely on preemption alone, without also yielding).
- **Editing `rtos/arm/main.c` reintroduces a Renode hang:** a
  refactor-only change (renaming a helper, dropping a print) reproduced a
  `PendSV`/`SysTick`-pending-but-never-serviced hang during development,
  for reasons not fully root-caused — see the Phase 8 section of
  `docs/design.md` before restructuring it.

See `docs/design.md` for a deeper architecture reference and rationale
behind each design decision (why `ucontext.h` over `setjmp`/`longjmp`, how
priority inheritance is implemented, etc).