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
[ROS2 client layer](#ros2-client-layer-micro-xrce-dds) below for
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

## ROS2 client layer (Micro XRCE-DDS)

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

**Verified end to end against a real, unmodified agent** — both over UDP
from a host process and over the real serial transport from real
QEMU-booted ARM firmware (`host/live_agent_check.c`,
`host/live_publish_demo.c`, `rtos/arm/ros2_demo.c` — manual programs, not
part of `make test`, since they need a live external agent process):

```bash
# UDP, in WSL after host/setup_wsl.sh:
MicroXRCEAgent udp4 -p 8888 &
gcc -Ixrce/include host/live_publish_demo.c xrce/build/libxrce.a -o /tmp/demo
/tmp/demo 127.0.0.1 8888

# or the real serial transport, from QEMU-booted ARM firmware:
cd rtos/arm && make ros2_demo
setsid qemu-system-arm -M netduinoplus2 -nographic -kernel build/ros2_demo.elf \
    -serial pty -monitor none < /dev/null   # prints the allocated /dev/pts/N
MicroXRCEAgent serial -D /dev/pts/N -b 115200

# either way, separately:
ros2 topic echo /chatter
```

**`ros2 topic echo` prints a clean, correct, monotonically increasing
`data: N` sequence** — sustained over hundreds of samples, on both
transports. Getting there meant tracing a real bug past the point of "the
agent accepts our messages": its logs confirmed session/entity creation
and WRITE_DATA reaching the DDS layer, but the RTPS reader initially
rejected every sample with a payload-size mismatch. Reading the agent's
own `TopicPubSubType::serialize()` (not guessing) showed why: its generic,
dynamically-created topic type always prepends its own 4-byte CDR
encapsulation header to whatever bytes it's given — and this project's
client was *also* including its own header, producing a genuine double
header. Fix: send just the field bytes to `WRITE_DATA`, letting the agent
supply the encapsulation. `xrce/cdr.c`/`msgs.c` themselves were never
wrong — this was specifically about what bytes to hand this agent's
generic topic path. Full account, including the exact byte math, in
`xrce/docs/design.md`.

Three other real bugs surfaced building the QEMU-serial path, also
recorded there: QEMU's `-nographic` monitor dying with the launching
shell (fixed with `setsid`), a demo firmware buffer sized for the wrong
message (silently dropping the datawriter CREATE), and a real timing race
between firmware boot and agent-to-pty attachment (worked around with
periodic re-announcement, since this demo has no UART RX to read replies
and retry properly).

**Subscriptions (Phase 5) also work, genuinely bidirectionally — over both
UDP and the real serial transport, into real QEMU-booted firmware.**
`host/live_subscribe_demo.c` creates a participant/topic/subscriber/
datareader and issues `READ_DATA` against a real, unmodified agent; a real
`ros2 topic pub /cmd std_msgs/msg/Int32 "{data: 99}" --once` — the
standard ROS2 CLI, nothing from this project — drove it, and the demo
printed `received /cmd data=99`. `uart.c` gained RX and
`rtos/arm/ros2_demo.c` creates a matching subscription for `rt/setpoint`;
`ros2 topic pub /setpoint std_msgs/msg/Int32 "{data: 4242}" --once`
against a real agent produced exactly `setpoint updated: 4242` in the
firmware's own UART output (confirmed repeatedly, including a negative
value to check sign handling) — the actual "host sends a command, the
RTOS task receives it" scenario from the original brief, running for real
in QEMU.

Two real protocol bugs found and fixed along the way, ground-truthed
against the agent's own source rather than guessed: `DATA`'s sample bytes
are header-less (the mirror of the WRITE_DATA fix above); and `READ_DATA`
originally defaulted to a **one-shot** read (the agent's own
`DataReader::read()` treats an omitted delivery control as
`max_samples(1)`, not "unlimited") — the first published value arrived,
the second silently didn't, until fixed to always request
`max_samples = 0xFFFF` explicitly.

Getting the QEMU-serial case working also meant discovering that **the
debugging tool built to observe it was itself the reason it looked
broken, twice** — worth reading if you're debugging something similar:
see `xrce/docs/design.md`'s Phase 5 section for the full story (a pty
left in echo mode corrupting traffic, then a log-parsing method that
missed text split across lines). Neither was a bug in this project's
protocol implementation.

Not yet done (at Phase 5): services (request/reply), multi-node. Services
and actions land in Phase 7b below; multi-node is still not done.

**Phase 7a — DELETE, verified live: an entity really stops existing, not
just that the agent replies OK.** `xrce_session_build_delete()`
(`session.h/c`) adds the last entity-lifecycle operation on top of
CREATE/WRITE_DATA/READ_DATA — ground-truthed against a fresh clone of the
reference client (`SUBMESSAGE_ID_DELETE = 3`, payload is just
`request_id + object_id`, no XML). `host/live_delete_demo.c` creates
`rt/delete_test`, confirms `/delete_test` in a real `ros2 topic list`,
deletes it, and confirms it's gone from a second real `ros2 topic list` —
full account, including a leaked-process debugging note, in
`xrce/docs/design.md`'s Phase 7a section.

**Phase 7b — services and actions, both verified against real, unmodified
ROS2 tooling.** REQUESTER/REPLIER entities (`host/live_service_demo.c`)
reuse the existing CREATE/WRITE_DATA/READ_DATA machinery almost entirely
unchanged — the one real addition is a 24-byte `SampleIdentity` prefix
(ground-truthed against the agent's own FastDDS middleware source) that
correlates a Replier's reply to its request. `ros2 service call /self_test
std_srvs/srv/Trigger` gets a real, correctly-numbered response every time.
Actions (`host/live_action_demo.c`, `example_interfaces/action/Fibonacci`)
turned out to need no new protocol work at all — a ROS2 action is just
three services and two topics under a naming convention — only new message
types, whose exact wire layout came from the real generated ROS2 headers,
not guesswork. `ros2 action send_goal /fibonacci
example_interfaces/action/Fibonacci "{order: 5}" --feedback` prints real
incrementing Fibonacci feedback and a matching final result; a small
`rclpy` script driving the standard `ActionClient` confirms cancellation
mid-flight actually stops the goal at the right point. Two real bugs (a
GetResult request that needs to be held rather than polled, and a demo
bug that permanently rejected goals after the first one completed) are
recorded in full in `xrce/docs/design.md`'s Phase 7b section.

**Phase 7c — QoS enforced for real, verified under real induced packet
loss.** Adds the reliable-stream half of the protocol (HEARTBEAT/ACKNACK,
`session.h/c`) and a sender-side retransmit window
(`xrce/include/xrce/reliable_stream.h`) — only the sender side, since the
agent's own input reliable stream already implements the reader side
correctly. `host/udp_loss_proxy.py` (new, same idea as `pty_bridge.py`)
sits between the client and a real agent, dropping 35% of datagrams each
direction; `host/bench_qos.c` publishes 20 samples on a best-effort topic
and a reliable one through it. The reliable stream's own delivery
confirmation (`last_acknown` reaching its target) is 100% every run — a
hard proof every sample reached the agent, with real measured cost (e.g.
43 retransmits, 8 heartbeat rounds in one run) — while best-effort has no
recovery mechanism at all. A real, separate `ros2 topic echo` subscriber
downstream (a second, uninstrumented DDS hop) sees the same story with
real numbers: 14/20 for reliable vs. 6/20 for best-effort in one run,
consistently more than double across repeated runs. QoS is also wired
into the real DDS entity via CREATE XML (`<historyQos>` inside `<topic>`,
`<qos><reliability>` as its sibling — a shape found only by reading the
agent's own XML-parser rejection log, since two plausible shapes from
public FastDDS docs were both rejected). Four real bugs surfaced building
this — session/address correlation, replay-protected duplicate CREATEs,
a real ALREADY_EXISTS status this project had assumed away, and a
shared-vs-per-stream sequence counter mismatch — each found by reading the
agent's own log/ACKNACK values rather than assumed, full account in
`xrce/docs/design.md`'s Phase 7c section.

**Phase 6 — latency, throughput, and fault handling, measured rather than
estimated.** `ros2_demo.c` echoes every received `rt/setpoint` value back
out on `rt/pong`; `host/bench_latency.c` times the real round trip through
the actual firmware (not something computed from QEMU's own uncalibrated
sense of time — see `rtos/arm/main.c`):

```bash
gcc -Ixrce/include host/bench_latency.c xrce/build/libxrce.a -o /tmp/bench
/tmp/bench 127.0.0.1 8888 20        # sequential round trips
/tmp/bench 127.0.0.1 8888 burst 30  # back-to-back burst, measures loss
```

- **Latency** (paced one round trip at a time): `min=4.33ms  p50=6.50ms
  mean=6.69ms  p95=12.46ms  max=12.46ms` — single-digit milliseconds
  through two independent agent processes into real DDS and back.
- **Throughput/loss under a true burst**: 40% loss at a 5-message burst,
  80% at 30 — a real, expected limit of this implementation (UART RX is
  polled, not interrupt-driven, with no FIFO absorbing a burst faster than
  the poll loop returns to it), not a bug. Fine for a steady trickle,
  not bursty traffic — stated plainly rather than glossed over.
- **Fault handling, tested directly**: killed the serial agent mid-session
  with the firmware left running, then started a fresh agent — all
  entities reappeared automatically via the same periodic re-announcement
  built for the boot-timing race, with zero firmware restart or code
  change needed, confirmed by running the latency benchmark immediately
  after and getting real round trips back.

**Known limitations vs. real micro-ROS**, in `xrce/docs/design.md`'s Phase
6 section: no reliable streams, no services, no real type introspection
(bare XML name strings, not generated typesupport), no multi-node, polled
RX, no on-device reply correlation, uncalibrated QEMU clock. Every one of
these is a real, specific gap — not a vague disclaimer.

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
  include/xrce/              # serial_transport.h, cdr.h, msgs.h, session.h, reliable_stream.h
  src/                        # matching .c implementations
  tests/                      # host-native unit tests, own Makefile (same pattern as rtos/)
  docs/design.md              # ground truth notes, phase-by-phase status

host/                        # host-side scripts and live-agent test programs
  setup_wsl.sh                # installs qemu-system-arm, arm-none-eabi-gcc, ROS2 Jazzy, the agent
  run_qemu.sh                 # boots rtos/arm/build/kernel.elf under QEMU
  live_agent_check.c          # manual: CREATE_CLIENT against a real MicroXRCEAgent
  live_publish_demo.c         # manual: full entity-creation + publish loop against a real agent
  live_subscribe_demo.c       # manual: subscription, driven live by `ros2 topic pub`
  live_delete_demo.c          # Phase 7a: DELETE, verified live against ros2 topic list
  live_service_demo.c         # Phase 7b: Replier answering a real ros2 service call
  live_action_demo.c          # Phase 7b: full Fibonacci action server (goal/feedback/cancel/result)
  pty_bridge.py               # debug tool: tees QEMU<->agent serial traffic (see xrce/docs/design.md)
  udp_loss_proxy.py           # Phase 7c: induces known packet loss for QoS testing
  bench_latency.c             # Phase 6: real host<->RTOS round-trip latency + burst-loss benchmark
  bench_qos.c                 # Phase 7c: reliable-vs-best-effort delivery under induced loss
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
- [x] Phase 4 — Host-side bridge: `ros2 topic echo` decodes clean, correct
      data end to end, verified against a real, unmodified agent over both
      UDP and the real serial transport from QEMU-booted ARM firmware
- [x] Phase 5 — Subscriptions verified bidirectionally against `ros2 topic
      pub`, over both UDP and real serial into QEMU-booted firmware
      (including a real one-shot-vs-continuous bug found and fixed);
      services (request/reply) not yet done
- [x] Phase 6 — Latency (p50 6.5ms, p95 12.5ms) and burst-loss (40%@5,
      80%@30) measured against real firmware; agent disconnect/reconnect
      recovery confirmed; limitations vs. real micro-ROS written up
- [x] Phase 7a — DELETE entity operation, verified live: `ros2 topic list`
      shows a topic while it exists and not after deletion
- [x] Phase 7b — Services (`ros2 service call` answered by a real Replier)
      and actions (`ros2 action send_goal` with real feedback, result, and
      cancellation, verified against the CLI and `rclpy` directly)
- [x] Phase 7c — QoS (reliable streams, history depth) enforced for real:
      verified live under 35% induced packet loss with a real loss proxy,
      100% delivery confirmed to the agent on the reliable stream every
      run vs. no recovery at all for best-effort
- [ ] Phase 7d — Priority-aware executor dispatching callbacks via the
      RTOS's real priority scheduler
- [ ] Phase 8 — Live diagnostics exposed as ROS2 topics/services
- [ ] Phase 9 — `htop`-style live terminal UI over the diagnostics topic

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