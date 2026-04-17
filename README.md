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
boots under QEMU and Renode (`rtos/arm/`) — a `PendSV_Handler` doing manual
register save/restore and a real `SysTick` ISR driving preemption,
instead of `ucontext.h`/`SIGALRM`. See `rtos/arm/README.md` for details.
Everything else in this repo is host-native and needs no board, simulator,
or cross-compiler.

On top of that ARM port, `xrce/` implements enough of the real **Micro
XRCE-DDS Client protocol** — the same wire protocol actual micro-ROS boards
speak over their debug UART — for this RTOS to interoperate with a real,
unmodified `MicroXRCEAgent` and publish into a real ROS2 graph. See
[ROS2 client layer](#ros2-client-layer-micro-xrce-dds-option-a) below for
what's implemented, what's verified against the live agent, and what isn't
done yet.

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
| `arm-none-eabi-gcc` + `qemu-system-arm` (or [Renode](https://renode.io/)) | Only for the Phase 8 Cortex-M port (`rtos/arm/`) — not needed for anything else in this repo. |
| ROS2 Jazzy + a from-source `Micro-XRCE-DDS-Agent` build | Only for the `xrce/` ROS2 client layer's live-agent tests (`host/live_*.c`) — not needed to build/test `rtos/` or `xrce/`'s own unit tests. `host/setup_wsl.sh` installs all of the above (including `qemu-system-arm`/`arm-none-eabi-gcc`) in one script. |

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

## ROS2 client layer (Micro XRCE-DDS, Option A)

`xrce/` is portable, OS-independent C — no RTOS dependency, host-native
testable via its own `Makefile`, same `src/`+`tests/` pattern as `rtos/`:

```bash
cd xrce
make clean
make test          # or: make SANITIZE=address,undefined test
```

What it implements, ground-truthed against eProsima's actual reference
client source (not just its docs — see `xrce/docs/design.md` for a case
where the two disagree):

- **Serial transport framing** — the same byte-stuffed, CRC-16/ARC-checked
  framing a real micro-ROS board speaks over UART.
- **CDR serialization** — alignment-aware, for `std_msgs/Int32`,
  `std_msgs/String`, `sensor_msgs/Imu`.
- **Session / entity creation / WRITE_DATA** — CREATE_CLIENT handshake,
  CREATE (participant/topic/publisher/datawriter via XML representation),
  WRITE_DATA.

All three are unit-tested (including byte-exact checks against
hand-computed expected wire bytes, not just round-trips) and cross-compile
cleanly for the Cortex-M4 target.

**Verified against a real, unmodified agent** (`host/live_agent_check.c`,
`host/live_publish_demo.c` — manual one-off programs, not part of
`make test`, since they need a live external agent process):

```bash
# in WSL, after host/setup_wsl.sh:
MicroXRCEAgent udp4 -p 8888 &
gcc -Ixrce/include host/live_publish_demo.c xrce/build/libxrce.a -o /tmp/demo
/tmp/demo 127.0.0.1 8888
# separately: ros2 topic echo /chatter
```

The agent's own logs confirm session establishment and participant/topic/
publisher/datawriter creation from this project's hand-built protocol
messages, and `ros2 topic echo`'s RTPS reader actively receives each
published sample. **Not yet resolved:** the reader rejects the sample on a
payload-size mismatch (reproduces under both agent middleware modes).
Checked directly against the agent's own verbose log — it extracts and
forwards this project's bytes to Fast-DDS byte-exact, unmodified — so the
client/agent boundary is now confirmed correct and the mismatch is isolated
to Fast-DDS's own reader-side type-size accounting, most likely because the
topic was declared with a bare type-name string instead of real type
introspection. See `xrce/docs/design.md`'s Phase 4 section for the exact
evidence and the next step.

**Also verified over the actual serial transport, from real QEMU-booted
ARM firmware** — not just a host process over UDP. `rtos/arm/ros2_demo.c`
(`make ros2_demo` in `rtos/arm/`) links `xrce/` directly into a Cortex-M4
image and sends the same protocol, framed, over the emulated UART:

```bash
# terminal 1, in WSL:
cd rtos/arm && make ros2_demo
setsid qemu-system-arm -M netduinoplus2 -nographic -kernel build/ros2_demo.elf \
    -serial pty -monitor none < /dev/null   # prints the allocated /dev/pts/N
# terminal 2:
MicroXRCEAgent serial -D /dev/pts/N -b 115200
# separately: ros2 topic echo /chatter
```

Produces the identical `create_client` → `participant/topic/publisher/
datawriter created` → `[** <<DDS>> **]` sequence in the agent's log as the
UDP path — confirming the serial framing itself (Phase 1), not just the
message construction (Phase 3), works against the real agent from an
actual emulated target. Hits the same Fast-DDS decode gap described above
(expected — that gap is downstream of the transport). Two real bugs
surfaced getting this working (QEMU's `-nographic` monitor dying with the
launching shell; a demo-firmware buffer sized for the wrong message,
silently dropping the datawriter CREATE) — see `xrce/docs/design.md` for
both.

Not yet done: subscriptions/services (later phases), and on-device reply
parsing (this demo has no UART RX, so it re-announces blindly on a timer
instead of retrying on failure).

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
  arm/                      # Phase 8: Cortex-M4 port, boots under QEMU/Renode (own README/Makefile)
    ros2_demo.c              # Phase 4: xrce/ session layer over UART1, `make ros2_demo`
    libc_shim.c              # minimal memcpy/strlen for the freestanding ros2_demo build
  Makefile

xrce/                       # Micro XRCE-DDS client layer (Option A), portable/OS-independent
  include/xrce/              # serial_transport.h, cdr.h, msgs.h, session.h
  src/                        # matching .c implementations
  tests/                      # host-native unit tests, own Makefile (same pattern as rtos/)
  docs/design.md              # ground truth notes, phase-by-phase status

host/                        # host-side scripts and live-agent test programs
  setup_wsl.sh                # installs qemu-system-arm, arm-none-eabi-gcc, ROS2 Jazzy, the agent
  run_qemu.sh                 # boots rtos/arm/build/kernel.elf under QEMU
  live_agent_check.c          # manual: CREATE_CLIENT against a real MicroXRCEAgent
  live_publish_demo.c         # manual: full entity-creation + publish loop against a real agent
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
- [x] Phase 8 (stretch) — Cortex-M4 port, boots under QEMU (primary) and Renode

**ROS2 client layer (`xrce/`), tracked separately since it's a layer on
top of the kernel above, not part of it:**

- [x] Phase 0 — WSL toolchain (`qemu-system-arm`, ROS2 Jazzy, agent build); ARM port moved to QEMU-primary
- [x] Phase 1 — Serial transport framing, unit-tested
- [x] Phase 2 — CDR serialization (Int32/String/Imu), unit-tested
- [x] Phase 3 — Session/entity-creation/WRITE_DATA, unit-tested
- [~] Phase 4 — Host-side bridge: verified against a real agent over both
      UDP and the real serial transport from QEMU-booted ARM firmware
      (session + all four entity types + WRITE_DATA reaching the RTPS
      reader); a Fast-DDS reader-side type-size mismatch remains, isolated
      to downstream of the confirmed-correct client/agent boundary
- [ ] Phase 5 — Bidirectional (subscriptions, services) + realistic demo
- [ ] Phase 6 — Latency/throughput measurement, fault handling, polish

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