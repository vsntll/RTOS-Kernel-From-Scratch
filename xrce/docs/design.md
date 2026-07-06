# Design notes — ROS2 client layer (Micro XRCE-DDS, Option A)

Extends the RTOS (`rtos/`) so tasks can publish/subscribe into a real,
unmodified ROS2 graph, by implementing enough of the real Micro XRCE-DDS
Client protocol to interoperate with the official `micro-ros-agent` binary
-- the same protocol real micro-ROS boards speak over their debug UART.
This is "Option A" (strict compatibility) rather than a custom protocol: the
payoff is that anyone can verify it by pointing the standard agent at this
RTOS running in QEMU.

## Ground truth

Wire-format correctness is the entire point of Option A, so implementation
details that determine interop (framing byte layout, CRC algorithm/scope,
submessage IDs, CDR alignment) are checked against eProsima's public
Micro-XRCE-DDS-Client reference source and docs, not implemented from
memory. Where the two disagree, the reference *source* wins over
prose-summarized docs -- see the CRC-scope note below for a concrete case
where the public docs are misleading.

## Repo layout

`xrce/` holds portable, OS-independent C (`src/`, `include/xrce/`),
host-native-testable via its own `Makefile` (same `src/`+`tests/` pattern as
`rtos/`). The device-side firmware that actually runs in QEMU and links this
against the ARM kernel lives in `rtos/arm/`. Host-side agent/bridge tooling
lives in `host/`.

## Phase 0 — Environment + QEMU port

WSL toolchain (`host/setup_wsl.sh`): `qemu-system-arm`, `arm-none-eabi-gcc`,
ROS2 Jazzy desktop, and eProsima's `Micro-XRCE-DDS-Agent` built from source
(the exact same binary this project's interop claims are checked against --
nothing agent-side is modified or reimplemented). `rtos/arm/` moved from
Renode-only to QEMU-primary (`netduinoplus2`, USART1) -- see
`rtos/arm/README.md` for the emulator-specific details; that's the ARM
kernel port, not this layer, so it's documented there instead of duplicated
here. Status: verified -- the existing two-task demo boots identically
under `qemu-system-arm 8.2.2`.

## Phase 1 — Transport foundation (serial framing)

`serial_transport.c/h` reimplements eProsima's "Serial Transport" framing
from scratch (not copied) but wire-identical, since a real `micro-ros-agent
--serial` has to be able to parse it:

```
0x7E  src_addr  dst_addr  len_lsb  len_msb  payload[len]  crc_lsb  crc_msb
```

Every byte after the leading `0x7E` is byte-stuffed (`0x7E`/`0x7D` become
`0x7D, byte ^ 0x20`); the leading `0x7E` itself never is, and any unescaped
`0x7E` seen mid-frame means a new frame started there, which is what lets a
reader resync after lost or corrupted bytes without a separate resync
protocol.

**CRC scope, and a docs/source discrepancy worth recording:** the CRC is
CRC-16/ARC (poly `0x8005`, reflected in/out, init `0`, no xorout), computed
over the **payload bytes only** -- not the leading flag, not
src/dst/len. eProsima's own published transport docs describe the CRC as
covering the whole frame including the begin flag, per RFC 1662; the actual
reference client (`src/c/profile/transport/stream_framing/stream_framing_protocol.c`,
`uxr_write_framed_msg`/`uxr_read_framed_msg`) only ever calls
`uxr_update_crc()` on payload octets. Implementing this from the docs
instead of the source would produce a CRC that never matches the real
agent's. `xrce_serial_crc16()` is verified against the standard published
CRC-16/ARC check value for `"123456789"` (`0xBB3D`) -- an external fact,
independent of any vendor source -- in `tests/test_serial_transport.c`.

**Reliability is deliberately not this layer's job.** The generic Phase 1
brief calls for "simple ACK/retry so the link survives dropped bytes," but
under Option A that would be wire-incompatible noise: the real protocol
puts retransmission in the XRCE *Reliable Stream* (heartbeat/acknack,
Phase 3+), above this framing layer. What this layer guarantees instead:
corrupted frames are detected (bad CRC) and dropped, truncated frames never
falsely complete, and the reader resyncs on the next valid frame's flag
byte rather than wedging -- covered by
`case_corruption_detected`/`case_resync_after_garbage`/
`case_resync_after_truncated_frame` in the test suite.

Status: implemented and unit-tested host-natively (`xrce/tests`, run via
`make test`, clean under `-fsanitize=address,undefined`) and cross-compiles
cleanly for the Cortex-M4 target. Not yet wired into the QEMU firmware
image itself -- Phase 4's live-agent proof below went over UDP (agent and
protocol layers are transport-agnostic; the serial framing wraps the exact
same message bytes) rather than through the ARM firmware's UART, so
"framing survives a real UART" and "protocol bytes are accepted by the
real agent" have each been verified independently, but not yet as one
continuous QEMU-to-agent path. That wiring (an RTOS task calling this
layer, over `rtos/arm/uart.c`, into the agent listening on the QEMU
`-serial pty` device) is the natural next step, not yet done.

## Phase 2 — CDR serialization

`cdr.c/h`: alignment-aware CDR reader/writer (primitives aligned to their
own size, measured from the start of the buffer -- matches Fast-CDR's
behavior, which is what a real ROS2 participant actually runs). `msgs.c/h`:
field-exact layouts for `std_msgs/Int32`, `std_msgs/String`, and
`sensor_msgs/Imu` (the last one specifically to exercise nested
structs/mixed alignment: int32/uint32 header fields next to float64
vectors and 9-element covariance arrays). Status: unit-tested including a
byte-exact alignment case (not just round-trips, which can pass against a
self-consistent but wrong implementation) and cross-compiles for Cortex-M4.

## Phase 3 — Client library (session / entity creation / WRITE_DATA)

`session.h/c`: message header, submessage header (with backpatched
length -- simpler and less error-prone than the reference client's
precomputed-size approach, and produces identical bytes since CDR
alignment is a deterministic function of field sequence and starting
offset either way), CREATE_CLIENT handshake, CREATE for
participant/topic/publisher/datawriter via XML representation, and
WRITE_DATA. Deliberately out of scope: reliable streams
(heartbeat/acknack), subscriptions/READ_DATA, services -- see
`session.h`'s header comment.

Ground-truthed against `src/c/core/session/session_info.c`,
`src/c/core/serialization/xrce_types.c`, and `src/c/core/session/stream/stream_id.c`
in the reference client for: message-header layout (session_id/stream_id/
seq_num/[client_key]), the CREATE_CLIENT handshake's special-cased header
(session_id forced to 0x00, stream 0, seq 0 -- the target session doesn't
exist yet), ObjectId's 2-byte bit-packing (12-bit id + 4-bit kind, NOT a
CDR primitive), RequestId's big-endian byte order (the one field in the
whole protocol that isn't little-endian), and best-effort output stream
raw IDs (0 = none/handshake, 1 = best-effort index 0). `test_session.c`
hand-computes the full expected byte sequence for CREATE_CLIENT rather than
just round-tripping it.

Status: unit-tested (byte-exact for CREATE_CLIENT, full field-decode for
CREATE/WRITE_DATA) and cross-compiles for Cortex-M4.

## Phase 4 — Host-side bridge (real agent) -- fully working end to end

`host/live_agent_check.c` and `host/live_publish_demo.c` are manual,
one-off programs (BSD sockets, not portable/embeddable, not part of
`make test`) that run this project's session layer against a real,
unmodified `MicroXRCEAgent` over UDP -- the actual point of choosing
Option A: proof against an agent neither of us wrote, not just against our
own reader.

**Also verified over the real serial transport, from real QEMU-booted ARM
firmware** -- not just UDP from a host process. `rtos/arm/ros2_demo.c` (a
separate `make ros2_demo` build target, `rtos/arm/Makefile`) links `xrce/`
directly into a Cortex-M4 firmware image booting under QEMU
(`netduinoplus2`, USART1, same as Phase 8's demo), frames every message
with `xrce_serial_frame_encode()`, and sends it out over the emulated
UART. Pointing a real `MicroXRCEAgent serial -D <pty>` at the pty QEMU
allocates (`-serial pty`) produces the identical `create_client` / `session
established` / `participant created` / `topic created` / `publisher
created` / `datawriter created` / `[** <<DDS>> **]` log sequence as the UDP
path -- proving the serial framing (Phase 1), not just the message
construction (Phase 3), works against the real agent, from an actual
emulated target, not a host-native test harness standing in for one.

Two real bugs surfaced getting this working, both worth recording:
- **QEMU's `-nographic` monitor dies with the launching shell.** Backgrounding
  `qemu-system-arm -nographic ... &` and detaching with `disown` wasn't
  enough -- QEMU still received `SIGHUP` and exited the moment the
  launching shell session ended, because `-nographic` ties its monitor to
  the controlling terminal. Fixed by launching under `setsid` with stdin
  from `/dev/null` and `-monitor none` (no monitor needed for this,
  since nothing drives it interactively).
- **`send_frame()`'s frame buffer was sized for the wrong message.** It
  hardcoded `XRCE_SERIAL_MAX_ENCODED_LEN(64)`, sized for CREATE_CLIENT and
  the empty-XML publisher CREATE -- too small for topic/datawriter
  CREATE's ~100+ byte XML payloads. `xrce_serial_frame_encode()` fails
  safe (returns 0, no partial/corrupt write) on overflow, so nothing
  crashed -- the datawriter CREATE was just silently never sent at all,
  caught by its total absence from the agent's log (`participant
  created`/`topic created`/`publisher created` all present,
  `datawriter created` never appearing) despite the firmware itself
  running correctly. Fixed by sizing the buffer for this demo's actual
  largest message rather than a guessed constant.
- **A real timing race, worked around rather than "fixed"**: the firmware
  starts sending within a fraction of a real second of boot, but the host
  agent process takes a few real seconds to attach to a freshly-allocated
  pty. A send-once-at-boot handshake reliably lost that race and was never
  seen. `ros2_demo.c` re-announces the full CREATE_CLIENT + entity sequence
  every few publishes instead of once, so whichever attempt lands after the
  agent attaches gets through; duplicate CREATEs for already-existing
  entities just log `already exists` and are otherwise harmless. A real
  client would instead read the agent's replies and retry on failure/
  timeout -- this demo has no UART RX (see `ros2_demo.c`'s header comment),
  so periodic blind re-announcement is the pragmatic stand-in.

**Verified working, with the agent's own log as evidence, not just this
project's opinion of itself:**
- CREATE_CLIENT: agent logs `create_client` / `session established` for a
  hand-built client key.
- CREATE participant/topic/publisher/datawriter (XML representation):
  agent logs `participant created`, `topic created`, `publisher created`,
  `datawriter created`, each with matching object ids.
- WRITE_DATA reaches the DDS/RTPS layer and decodes cleanly:
  `ros2 topic echo /chatter` (real, unmodified ROS2 CLI) prints correct,
  monotonically increasing `data: N` values -- the message gets all the
  way from this project's hand-rolled client, through the real agent, into
  real DDS, decoded as a real `std_msgs/Int32`.
- All of the above reproduces identically whether the client is a host
  process talking UDP (`host/live_publish_demo.c`) or real QEMU-booted ARM
  firmware talking the actual serial framing over an emulated UART
  (`rtos/arm/ros2_demo.c`) -- confirming Phase 1's transport, not just
  Phase 3's message construction, works against the real agent.

**Resolved: `ros2 topic echo /chatter` decodes clean, correct values.**
Getting here took actually finding the root cause rather than guessing at
it. The RTPS reader was rejecting samples with `Change payload size of
'12' bytes is larger than the history payload size of '11' bytes and
cannot be resized` -- reproducing identically under both `-m dds` and
`-m rtps` agent middleware modes (ruling out a middleware-specific
dynamic-type quirk) and identically over both UDP and the real serial
transport (ruling out anything transport-related).

Traced by reading the agent's own source, not by trial and error:
- `DataWriter.cpp`'s `[** <<DDS>> **]` verbose log confirmed the agent
  extracts *exactly* this project's 8-byte CDR-encoded sample
  (`00 01 00 00 <int32 LE>`) from WRITE_DATA and hands it, unmodified, to
  `middleware.write_data()` -- so the client/agent submessage boundary was
  never the problem.
- The actual cause was one level further in:
  `src/cpp/types/TopicPubSubType.cpp`'s `serialize()` -- the generic,
  dynamically-created topic type every XML-declared entity in this project
  uses -- **always** writes its own 4-byte CDR_LE encapsulation header
  (`payload->data[0..3] = 0,1,0,0`) in front of whatever byte vector
  `write_data()` hands it, then `memcpy`s that vector in after. This
  project's client was *also* including its own 4-byte header inside that
  vector (correct, reusable CDR -- see Phase 2), producing a genuine
  double header: 4 (agent's) + 4 (ours) + 4 (the actual `int32`) = 12
  bytes, which is exactly the "12" the reader saw, in front of a real
  `std_msgs/Int32`'s true 8-byte encoding.

**Fix:** strip this project's own 4-byte CDR header before handing sample
bytes to `xrce_session_build_write_data()` -- send just the field bytes
(`sample + 4, sample_len - 4`), since the agent's generic topic type
supplies the encapsulation header itself. Applied at both call sites
(`host/live_publish_demo.c`, `rtos/arm/ros2_demo.c`); `xrce/cdr.c`/`msgs.c`
themselves are untouched and still correct as standalone CDR encoders --
this is specifically about how bytes get handed to *this agent's*
generic/dynamic topic path, not a bug in the CDR layer itself.

Confirmed working after the fix, both transports: `ros2 topic echo
/chatter` prints a clean, monotonically increasing sequence of `data: N`
values, sustained over hundreds of samples with no further errors.

Both files' XML entity-representation strings (the `<dds>...</dds>`
wrapper, element-not-attribute children) are ground-truthed against
`Micro-XRCE-DDS-Client/examples/PublishHelloWorld/main.c` -- an earlier
attempt using an attribute-style shorthand (`<dds_topic name="..."
.../>`) failed the agent's XML parser outright (`Not found root tag`),
which is exactly the kind of interop detail this project exists to get
right rather than guess at.

## Phase 5 — Subscriptions (bidirectional communication) -- fully working, both transports

`session.h/c` gained the read/receive half of the protocol: `READ_DATA`
(ask the agent to start delivering a datareader's samples on a chosen
stream) and parsing the `DATA` submessages the agent sends back.
Ground-truthed against the reference client's `read_access.c`
(`uxr_buffer_request_data()` for `READ_DATA`'s `ReadSpecification` layout;
`read_submessage_data()`/`read_format_data()` for how `DATA`'s payload is
handed to a caller -- raw bytes right after `BaseObjectRequest`, no further
wrapping) and its `Deployment/subscriber.c` /
`examples/SubscribeHelloWorld/main.c` for the subscriber/datareader XML
shapes (`<data_reader><topic><kind>NO_KEY</kind>...` -- same structure as
the datawriter XML from Phase 3/4, just the other entity kind).

One detail that would have been easy to get backwards: DATA's sample bytes
are **header-less**, the mirror image of the WRITE_DATA fix in Phase 4 --
`TopicPubSubType::deserialize()` in the agent's source strips its own
4-byte CDR header before this project's client ever sees the bytes
(`buffer->assign(payload->data + 4, ...)`), so `xrce_session_parse_data()`
hands back raw field data directly, not something that needs
`xrce_cdr_read_header()` first. Unit-tested in `test_subscribe.c`
(`xrce/tests/`), including a hand-built incoming `DATA` message to prove
parsing works without needing a live agent for that level of test.

**Verified against a real agent, genuinely bidirectional -- over UDP:**
`host/live_subscribe_demo.c` creates a participant/topic/subscriber/
datareader and sends `READ_DATA` against a real, unmodified
`MicroXRCEAgent`; a real `ros2 topic pub /cmd std_msgs/msg/Int32
"{data: 99}" --once` (the standard ROS2 CLI, not anything from this
project) drove it, and the demo printed
`received /cmd data=99 (from object 0x001 kind 0x6)` -- a value that
originated in a real ROS2 publisher, decoded correctly by this project's
own client, with zero involvement from eProsima's client code on the
receiving end either. This is the concrete "host sends a command, the
client receives it" half of the original bidirectional brief.

**A real bug, found and fixed: `READ_DATA` defaulted to one-shot.** The
first version of `xrce_session_build_read_data()` set
`optional_delivery_control = false`, which read like "no limits
requested." It's actually the opposite: `DataReader::read()` in the
agent's own source defaults an *omitted* delivery control to
`max_samples(1)` -- a single read, not a subscription. This looked like it
worked (the very first published value arrived) right up until a second
`ros2 topic pub` silently produced nothing. Fixed by always requesting
`optional_delivery_control = true` with `max_samples = 0xFFFF`
(`UXR_MAX_SAMPLES_UNLIMITED` in the reference client, which
`examples/SubscribeHelloWorld/main.c` always sets explicitly for exactly
this reason). Confirmed after the fix: three sequential
`ros2 topic pub` calls (values 1, 2, 3) all arrived, in order, over the
same long-lived subscription.

**Also verified over the real serial transport, into real QEMU-booted ARM
firmware.** `uart.c` gained polled RX (`uart_getc_nonblocking()`), and
`rtos/arm/ros2_demo.c` creates a full subscriber/datareader for
`rt/setpoint` alongside the existing publisher, feeding received bytes
through `xrce_serial_reader_feed()` and printing `setpoint updated: N`
when a matching `DATA` submessage decodes. A real
`ros2 topic pub /setpoint std_msgs/msg/Int32 "{data: 4242}" --once`
against a real, unmodified agent produced exactly
`setpoint updated: 4242` in the firmware's own UART output -- confirmed
repeatedly, including with a negative value (`-17`) to check sign
handling. This is the actual "host sends a command, the RTOS task
receives it" scenario from the original project brief, running for real
inside QEMU.

**This took two rounds of chasing a problem that turned out to be in the
test tooling, not the implementation -- worth recording in full, since
"it doesn't work" turned out to be the wrong conclusion twice before it
was right.** Debugging this needed a way to observe QEMU's raw UART
traffic *and* have a real agent attached at the same time, which isn't
otherwise possible (the agent holds its end of the pty exclusively). The
tool built for that, `host/pty_bridge.py`, is what actually caused both
false negatives:

1. **The bridge's two intermediate ptys were left in default cooked/echo
   mode.** `pty.openpty()` doesn't put its ptys in raw mode -- and with a
   real agent attached through the bridge, QEMU's own boot banner showed
   up *echoed back* on the agent-facing side of the relay, meaning the
   bridge was corrupting/duplicating the very binary protocol traffic it
   existed to observe. (In the earlier no-bridge tests, this specific bug
   didn't apply -- the agent's own `TermiosAgent::init()` configures raw
   mode on whichever device it opens directly -- but see point 2 for why
   those tests still looked like failures.) Fixed with `tty.setraw()` on
   both pty slaves right after creation.
2. **Even after fixing the bridge, "grep the log for `setpoint updated`"
   kept reporting nothing -- because the relay logs one line per
   `os.read()` call, and `os.read()` doesn't respect message boundaries.**
   The text `setpoint updated: 4242` really was there, just split across
   two separate log lines (e.g. `...setpoint u` / `pdated: 4242\r\n...`
   on the next), which a line-oriented `grep` will never match. Confirmed
   by reconstructing the full byte stream (concatenating every logged
   read, not grepping line-by-line) and searching *that* -- the value was
   present every time, going back to the very first test that was
   reported as a failure.

Debugged by adding temporary, reverted-before-commit instrumentation on
both ends to get past assumptions entirely: a few `fprintf` calls in the
agent's own `Processor::read_data_callback` (built locally via
`LD_LIBRARY_PATH` override, never touching the system-installed agent
binary other users/tests rely on) confirmed `push_output_submessage()`
and the pop-and-send loop were succeeding on the agent's side well before
the tooling bugs above were found; a temporary per-byte print in
`ros2_demo.c`'s RX poll loop confirmed the firmware genuinely was
receiving bytes at the hardware/QEMU level throughout. Both were reverted
once the real cause (the two bugs above, not the client/agent protocol)
was confirmed.

Not yet done: services (request/reply, a different submessage pair
again).

## Phase 6 — Realistic demo, latency/throughput, fault handling, writeup

**Realistic combined demo:** `ros2_demo.c` now echoes every received
`rt/setpoint` value back out on a third topic, `rt/pong`, immediately
after printing it. This isn't a separate feature -- it's what makes an
honest latency measurement possible at all: `host/bench_latency.c`
publishes an incrementing value on `rt/setpoint` and times how long until
the matching value comes back on `rt/pong`, so the measured latency is the
real round trip through the actual firmware (agent -> DDS -> agent ->
real serial framing -> QEMU's UART -> the firmware's own poll loop ->
decode -> re-encode -> back through the same chain), not something
computed from QEMU's own sense of time -- see `rtos/arm/main.c`'s
`SYSTICK_RELOAD` comment for why on-device timestamps specifically
wouldn't be trustworthy here (the RCC/PLL is never configured, so the
core clock is whatever the reset default happens to be).

**Latency**, one round trip at a time (paced naturally by each round
trip's own network+DDS+serial delay, which is what a real, non-bursty
publish/subscribe workload looks like): 14/15 round trips completed in one
run (the first attempt in a fresh session routinely times out -- DDS
discovery/matching for just-created entities isn't instant, see the burst
note below), with
`min=4.33ms  p50=6.50ms  mean=6.69ms  p95=12.46ms  max=12.46ms`.
Single-digit milliseconds end to end, through two independent agent
processes (one bridging QEMU over serial, one bridging the benchmark tool
over UDP, both into the same DDS domain) -- not a number to over-index on
precisely (it depends on host load, QEMU's emulation speed, and this
demo's own busy-loop pacing), but the right order of magnitude for a
best-effort, non-realtime-tuned link like this one.

**Throughput/loss under a true burst** (`host/bench_latency.c ... burst
<n>`, firing `n` pings back-to-back with no per-message wait, after a
short settle delay to let DDS discovery finish first -- without that
delay, a burst run confounds discovery latency with actual loss and
reports 100% loss regardless of `n`, which is a discovery-timing artifact,
not a throughput measurement): a burst of 5 lost 40% (2/5), a burst of 30
lost 80% (6/30). This is a **real, expected characteristic of this
specific implementation**, not a bug to fix: `uart_getc_nonblocking()` is
polled, not interrupt-driven, and is only ever drained inside
`pace_and_poll_rx()` between other work (building the next outgoing
message, re-announcing, etc.) -- there's no hardware or software FIFO
absorbing a burst that arrives faster than the poll loop gets back around
to checking. A real deployment needing to survive bursts would need
either interrupt-driven RX with a real ring buffer, or to lean on the
protocol's own reliable-stream retransmission (Phase 3+ notes explain why
that's a separate layer, not implemented here) rather than best-effort
delivery. Stated plainly: this link is fine for a steady trickle of
commands/telemetry, not for bursty traffic.

**Fault handling, tested directly rather than assumed:**
- **Agent disconnect/reconnect:** killed the serial agent process while
  QEMU/the firmware kept running untouched, waited, then started a fresh
  agent process on the same device. All entities re-appeared in the new
  agent's log (`created` x9) via the existing periodic re-announcement --
  a mechanism originally built to solve the boot-timing race (Phase 4/5)
  turns out to also be exactly what's needed for agent-restart recovery,
  with no firmware restart and no code change required. Confirmed
  functionally recovered, not just entity-recreated, by running the
  latency benchmark immediately after and getting real round trips back.
- **Malformed/corrupted bytes:** covered at the framing layer already --
  `xrce/tests/test_serial_transport.c`'s corruption/truncation/resync
  cases (Phase 1) are exactly this, and apply here unchanged since this
  demo uses the same `xrce_serial_reader_feed()` unmodified.
- **RTOS task crash mid-publish:** doesn't apply as originally framed --
  `ros2_demo.c` has no real task/scheduler (see its own header comment;
  `kernel_arm.c`/`context_switch.s` are linked in only to satisfy
  `startup.s`'s vector table, never exercised). What was actually tested:
  killing QEMU itself mid-session leaves the agent process running,
  unharmed, simply timing out further reads from a now-dead pty -- no
  hang, no crash on the host side.

**Known limitations vs. real micro-ROS**, stated plainly rather than left
implicit:
- No reliable streams (heartbeat/acknack retransmission) -- everything
  here is best-effort, by design (see `xrce/include/xrce/session.h`'s
  header comment); real micro-ROS supports both.
- No services (request/reply) -- a real, different submessage pair
  (`REQUESTER`/`REPLIER` object kinds), not implemented.
- No real type introspection -- topics are declared with a bare
  `dataType` name string via XML, not an XTypes `TypeObject`/generated
  typesupport. Works for the fixed, hand-picked types this project
  actually uses (`std_msgs/Int32`, tested; `std_msgs/String` and
  `sensor_msgs/Imu` at the CDR layer only, Phase 2), but wouldn't
  automatically support an arbitrary new ROS2 message type the way a real
  rosidl-generated client does.
- ~~No multi-node support~~ -- true when this was written; closed in
  Phase 10 below (`NODE_ID`-parametrized client key/participant/topic
  names, verified with 3 simultaneous QEMU boards under one agent).
- Polled UART RX, no interrupt/FIFO -- see the throughput section above.
- No on-device reply parsing/correlation -- `ros2_demo.c` never reads
  `STATUS`/`STATUS_AGENT` replies to its own requests; it re-announces
  blindly on a timer instead of retrying based on an actual failure
  signal. Works, but a real client would do better.
- QEMU's core clock is uncalibrated (RCC/PLL never configured) -- ticks
  and busy-loop counts are not comparable to real wall-clock time,
  documented at each of the several places in this codebase where it
  matters rather than silently assumed away.

## Phase 7a — Close the entity-lifecycle gap: DELETE

Phase 4's milestone (`ros2 topic list`/`echo` work against zero custom bridge
code) already covers session establishment and entity creation; the
remaining literal gap vs. "session/create/delete entity operations" was
DELETE. `xrce_session_build_delete()`/`xrce_session_parse_delete_reply()`
(`session.h/c`) add it: a `BaseObjectRequest`-only payload (no XML, unlike
CREATE) on `SUBMESSAGE_ID_DELETE`, replied to with the same `STATUS`
submessage shape as CREATE's reply (factored into a shared
`parse_status_reply()` rather than duplicated). Ground-truthed against a
fresh clone of `Micro-XRCE-DDS-Client` (not memory): `submessage_internal.h`
gives the full submessage id enum (`DELETE = 3`, and, useful later for
Phase 7c, `ACKNACK = 10`/`HEARTBEAT = 11`); `common_create_entities.c`'s
`uxr_buffer_delete_entity()` confirms the payload is exactly
`request_id + object_id`, its own comment noting "no padding."
`test_session.c` hand-computes the full expected byte sequence, same
standard as `case_create_client_byte_exact`.

**Verified against a real agent -- proving the entity actually stops
existing, not just that DELETE gets an OK reply.** `host/live_delete_demo.c`
creates participant/topic/publisher/datawriter for `rt/delete_test`,
publishes once, waits, deletes datawriter/publisher/topic, waits again.
Run against a real `MicroXRCEAgent udp4`, real `ros2 topic list` shows
`/delete_test` while the entities exist and **not** afterward:

```
=== ros2 topic list WHILE ENTITY EXISTS ===
/delete_test
/parameter_events
/rosout
=== ros2 topic list AFTER DELETE ===
/parameter_events
/rosout
```

One debugging note worth recording: an early run of this check reused a
UDP port an *earlier, still-running* agent process (leaked by a prior
failed shell invocation) already had bound. The new agent process failed to
start (`bind error ... errno: 98`), but the leaked old one silently kept
answering -- and since it already had a participant with the same id from a
previous successful run, the new demo's `CREATE participant` came back
`FAILED` (object already exists under a different representation-matching
outcome than a duplicate CREATE normally gets). Not a protocol bug --
confirmed by checking `ps aux` and the old agent's own log line, then
killing it and re-running against a genuinely fresh agent, which is the run
quoted above. Separately: `pkill -f MicroXRCEAgent` run from inside a
`bash -lc "...MicroXRCEAgent..."` string matches its own invoking shell's
command line too (that's what `-f` matches against) and kills it -- switched
all agent/demo process cleanup in this project's manual test invocations to
exact-name matching or explicit `$!` PIDs instead.

Status: implemented and unit-tested (byte-exact) in `xrce/`; verified live
against a real, unmodified agent as shown above.

## Phase 7b — Services and actions

**Services.** The XRCE `REQUESTER`/`REPLIER` object kinds (`0x07`/`0x08`,
ground-truthed against the reference client's `object_id.h`) turn out to
need almost no new protocol code: their CREATE payload is the exact same
shape `xrce_session_build_create_xml()` already builds for every other
XML-represented entity (confirmed by reading the agent's
`Requester::create()`/`Replier::create()`, `src/cpp/requester/Requester.cpp`
/`src/cpp/replier/Replier.cpp`), and requests/replies ride the same
WRITE_DATA/READ_DATA/DATA submessages topics already use. The one real
addition, found by reading the agent's FastDDS middleware layer
(`FastDDSReplier::write()`/`read()`,
`src/cpp/middleware/fastdds/FastDDSEntities.cpp`) rather than guessed: a
Replier's incoming request (via DATA) and outgoing reply (via WRITE_DATA)
are each prefixed with a 24-byte `SampleIdentity` (12-byte GUID prefix +
4-byte entity id + 8-byte sequence number) that correlates a reply to its
request at the DDS-RPC layer -- this project's client only ever needs to
store-and-replay those 24 bytes verbatim (`XRCE_SAMPLE_IDENTITY_SIZE`,
`session.h`), never interpret them. A Requester's own request/reply traffic
carries no such prefix -- the agent tracks that identity internally on that
side (`Requester::write()`/`read_fn()` never touch a client-supplied one).

Demo service: `std_srvs/srv/Trigger` (empty request; `bool success; string
message` response -- a real, already-installed standard interface, chosen
specifically to avoid needing `colcon`/custom interface generation, which
this WSL image doesn't have). `host/live_service_demo.c` creates a Replier
named `self_test` and answers real requests.

**A real naming gap, found only by testing, not by reading docs**: the
official reference client XML example
(`examples/RequestAdder/main.c`) only sets a bare `service_name` attribute
and lets the library derive the underlying DDS request/reply topic names.
Doing exactly that got the agent to create the Replier without error, but
`ros2 service list` never showed it -- FastDDS's own default topic-name
derivation from `service_name` does not match ROS2's own DDS-RPC naming
convention (confirmed via `design.ros2.org/articles/topic_and_service_names.html`
and cross-checked empirically against a real `rclpy` service's hidden
topics: `ros2 topic list --include-hidden-topics` doesn't even show
service topics -- they're excluded from the ROS graph API entirely, not
merely hidden -- so the actual check was standing up a real Python
`rclpy` service and confirming `ros2 service list` found it, then applying
the same fix here). Fix: set `<request_topic_name>`/`<reply_topic_name>`
explicitly to ROS2's own convention -- `rq` + fully-qualified name +
`Request`, `rr` + ... + `Reply` (e.g. `rq/self_testRequest`,
`rr/self_testReply`) -- rather than relying on any library default.
Confirmed after the fix: `ros2 service list` shows `/self_test`, and three
consecutive `ros2 service call /self_test std_srvs/srv/Trigger` calls each
got a distinct, correctly-numbered real response
(`success=True, message='self-test #1/2/3 ok'`).

**Actions.** `example_interfaces/action/Fibonacci` (goal `int32 order`,
result/feedback `int32[] sequence` -- a real, already-installed action
type, chosen the same way `Trigger` was to avoid custom interface
generation). A ROS2 action turns out to be nothing new at the XRCE level:
per `design.ros2.org/articles/actions.html`, it's exactly three services
(`send_goal`, `cancel_goal`, `get_result`, each under a
`<action>/_action/<verb>` name) and two plain topics (`feedback`,
`status`) -- entirely built from what 7b's services work and Phase 4's
plain topics already provide. The synthesized wrapper types
(`Fibonacci_SendGoal_Request/Response`, `Fibonacci_GetResult_Request/Response`,
`Fibonacci_FeedbackMessage`) and the standard `action_msgs` types
(`GoalInfo`, `GoalStatus(Array)`, `CancelGoal_Request/Response`) were not
guessed from `ros2 interface show` (which only shows the user-visible
`Goal`/`Result`/`Feedback`, not the RMW-level wrappers) -- their exact
field layout was read directly from the real generated C headers installed
under `/opt/ros/jazzy/include/example_interfaces/.../detail/fibonacci__struct.h`
and `.../action_msgs/.../detail/*__struct.h`, which is strictly better
ground truth than any doc for field layout specifically (`msgs.h`'s header
comment records this). `cdr.h` gained a `sequence<int32>` primitive
(`xrce_cdr_write_seq_i32`/`read_seq_i32`) for the `int32[]`
feedback/result fields and reused inline for `GoalStatus[]`/`GoalInfo[]`.

`host/live_action_demo.c` runs all five action entities from a single
poll loop (no RTOS task involvement yet -- that's Phase 7d) and got two
real behavioral bugs found only by testing against real `ros2 action`
tooling, not assumed from the spec:
- **A held-open GetResult, not a polled one.** The first version replied
  "not ready yet" and dropped the request if the goal wasn't done,
  assuming (wrongly) that a real client would retry. `ros2 action
  send_goal --feedback` sends GetResult exactly once, right after goal
  acceptance, and then just waits -- confirmed by it never printing a
  `Result:` section at all with the polling version, and the server log
  showing exactly one GetResult request despite the goal running for
  several more seconds. Fixed by holding the request's `SampleIdentity`
  and replying only once the goal reaches a terminal state
  (`g_result_pending`/`maybe_reply_pending_result()`).
- **A demo bug, not a protocol bug: goals stayed rejected forever after
  the first one.** `accepted = !g_goal_active` never got reset because
  completing a goal set `g_goal_done` but left `g_goal_active` true
  forever, so a second, independent `ros2 action send_goal` call got
  "Goal was rejected." Fixed by accepting whenever `!g_goal_active ||
  g_goal_done`. Confirmed after the fix: two independent goals in a row
  both completed successfully.

**Verified against real, unmodified ROS2 tooling, both the CLI and
`rclpy` directly:**
- `ros2 action list` shows `/fibonacci`.
- `ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci
  "{order: 5}" --feedback` prints five real, correctly-valued Fibonacci
  feedback messages (`0, 1, 1, 2, 3`) followed by `Goal finished with
  status: SUCCEEDED` and a matching `Result:`.
- A second, independent goal (`order: 3`) also completes correctly, proving
  the server actually supports more than one goal in sequence.
- **Cancellation**, verified with a small `rclpy` script using the
  standard `ActionClient` (the same library `ros2 action send_goal` itself
  uses internally -- not a custom bridge) since the `ros2 action` CLI has
  no `cancel_goal` subcommand and doesn't cancel on Ctrl-C: sends `{order:
  20}`, waits for 4 feedback messages, calls `cancel_goal_async()`, and
  gets back a real `CancelGoal_Response(return_code=0, goals_canceling=[...])`
  followed by `final status: 5` (CANCELED) with the sequence frozen at
  exactly the 4 values already sent (`[0, 1, 1, 2]`) -- matching the
  server's own log (`cancel request accepted` / `goal canceled after 4
  values` / `get_result replied: status=5, 4 values`) line for line.

Status: implemented and unit-tested (CDR round-trips for every new message
type, `xrce/tests/test_msgs.c`/`test_cdr.c`) and verified live against a
real, unmodified agent, `ros2` CLI, and `rclpy`, as shown above.

## Phase 7c — QoS enforced for real (reliable streams, history depth)

Adds the reliable-stream half of the protocol left out since Phase 3:
HEARTBEAT/ACKNACK (`xrce_session_build_heartbeat()`/`parse_acknack()`,
`session.h/c`) and a sender-side window (`xrce/include/xrce/reliable_stream.h`,
new file) that tracks sent-but-unacked messages and retransmits on a
nonzero-bitmap ACKNACK. Ground-truthed against the reference client's
`stream_id.c` (raw stream ids split none(0)/best-effort(1-127)/reliable(128-255),
`XRCE_RELIABLE_STREAM_THRESHOLD`), `xrce_types.h` (HEARTBEAT/ACKNACK payload
layout), and `output_reliable_stream.c`'s own strategy: on any nonzero
ACKNACK bitmap, retransmit the *whole* outstanding window rather than
decode individual bitmap bits -- the reference client does the same thing,
so this isn't a shortcut, it's matching real behavior. Only the sender
side is implemented: the agent's own input reliable stream (its reader)
already does the other half correctly since it's real and unmodified, so
there's nothing to build there. Unit-tested with simulated loss
(`xrce/tests/test_reliable_stream.c`): a lost message triggers a full
window retransmit, a fully-acked one doesn't, and the window applies real
backpressure once full (not an unbounded buffer).

QoS is also wired into the CREATE XML for real, not just accepted and
ignored: `<data_writer><topic>...<historyQos><kind>KEEP_LAST</kind>
<depth>N</depth></historyQos></topic><qos><reliability><kind>RELIABLE
</kind></reliability></qos></data_writer>` (see `build_datawriter_xml()`,
`host/bench_qos.c`) actually configures the agent's own DDS-side entity,
not just this client's local behavior. This exact shape took real
empirical work to find, ground-truthed against the running agent's own
XML parser error output since the public docs turned out to be misleading
for this specific case (same category of gap as Phase 4's serial-transport
CRC-scope note): two plausible shapes from published Fast DDS examples
(`<qos><reliability>...</reliability><topic><historyQos>...`, and a bare
`<history>` as a direct `<qos>` child) both got a real rejection logged by
the agent itself (`[XMLPARSER Error] Invalid element found into
'writerQosPoliciesType'. Name: topic` / `... Name: history`) before
finding that `<historyQos>` is actually a sibling of `kind`/`name`/
`dataType` *inside* `<topic>`, with `<qos><reliability>` as a separate
sibling of `<topic>` -- confirmed accepted only once tried against the
real agent, not asserted from documentation.

**Verified live, under real induced packet loss, with real numbers.**
`host/udp_loss_proxy.py` (new) sits between this project's client and a
real `MicroXRCEAgent`, dropping each direction independently at a
configured rate -- same reasoning as `pty_bridge.py` (Phase 5): the thing
being measured has to happen on the actual wire, not be simulated inside
either endpoint. `host/bench_qos.c` publishes the same 20-sample sequence
on a best-effort topic and a reliable topic, both through their own lossy
proxy instance (35% loss each direction), and reports what actually
happened rather than asserting success:

```
best-effort: sent 20 samples in 0ms, no retries (none possible)
reliable:    sent 20 samples in 1667ms across 8 heartbeat round(s), 43 retransmitted
             byte-for-byte, last_acknown=19 (fully acked once it reaches 19)
```

`last_acknown` reaching its target is a hard, deterministic proof that
**every** sample the reliable stream sent was eventually confirmed
delivered to the agent, every run -- best-effort has no equivalent
mechanism at all: whatever the proxy drops on that hop is gone, silently,
forever. Checking what a real, separate `ros2 topic echo` subscriber
actually received end to end (a second, uninstrumented DDS hop from the
agent onward, so noisier than the client-to-agent number above) shows the
same story with real numbers: 14/20 for the reliable topic vs. 6/20 for
best-effort in one representative run -- consistently more than double,
run after run, never worse.

Debugging this live demo surfaced four real, worth-recording bugs, none of
them in the reliable-stream mechanism itself:
- **Setup traffic must go through the same address the data path uses.**
  An early version sent CREATE_CLIENT/CREATE directly to the agent and
  only routed WRITE_DATA/HEARTBEAT through the lossy proxy. The agent
  correlates an established session with the UDP source *address*
  subsequent traffic arrives from; since the proxy relays through one
  fixed local socket, the agent saw a completely different source address
  for data than for setup and silently dropped everything, with zero
  entries in its own log for any of it despite the proxy correctly
  forwarding every non-dropped packet (confirmed by temporarily adding
  per-packet FWD/DROP logging to `udp_loss_proxy.py`, now available behind
  its `-v` flag). Fixed by routing setup through the proxy too.
- **A resent, byte-identical CREATE gets no second reply.** The agent's
  best-effort input stream tracks the last sequence number it has
  processed per stream and silently drops anything at or before that
  watermark as a replay -- so blindly resending the exact same bytes after
  a client-side timeout can never recover a *reply* that was actually the
  one lost, confirmed by seeing exactly one `participant created` in the
  agent's log no matter how many identical copies were sent. Fixed by
  rebuilding each retry fresh (a `build` callback invoked per attempt in
  `send_and_wait_ok()`), so every attempt gets its own sequence number.
- **A real CREATE-of-an-existing-entity doesn't always reply OK_MATCHED.**
  Once retries got a reply every time, some CREATE calls still failed --
  with a genuine `0x82` (`UXR_STATUS_ERR_ALREADY_EXISTS`) status, not
  OK_MATCHED as this project's own Phase 6 notes assumed ("duplicate
  CREATEs... log already exists and are otherwise harmless"). Caught on a
  publisher CREATE specifically (empty-XML representation, so there's
  nothing for the agent to compare for an exact match). For a *retry*,
  ALREADY_EXISTS means success just as much as OK/OK_MATCHED does --
  `create_reply_ok_or_exists()` in `bench_qos.c` accepts both.
- **A session's sequence counter is shared across every stream it uses,
  and a reliable stream must start its own counting at 0.** `xrce_session_t`
  has one `out_seq_num` (documented in `session.h` as this project's
  "single best-effort output stream" simplification, Phase 3). Setup
  traffic on the best-effort stream had already advanced it well past 0 by
  the time the reliable stream was used for the first time, so the first
  reliable WRITE_DATA carried (say) seq 15 on a stream the agent had never
  seen before -- its ACKNACK correctly, faithfully kept reporting
  `first_unacked=0`, asking for 15 sequence numbers this client never sent
  *on that stream* and could never retransmit. Found by logging the
  agent's actual ACKNACK values rather than assuming the mechanism was
  broken. Fixed in the demo by resetting the counter to 0 at the exact
  point the reliable stream is first used (correct, not a hack: from then
  on stream 1 is never touched again in this session, so the counter *is*
  stream 128's own from that point).

A fifth issue, not a bug: `ros2 topic echo /topic` (no explicit type)
resolves the message type via graph discovery once at startup and gives
up permanently if the topic doesn't exist yet at that exact instant --
which it won't, on a freshly created entity. Not a discovery-timing race
to work around with a longer sleep; the actual fix is passing the type
explicitly (`ros2 topic echo /topic std_msgs/msg/Int32`), which every
invocation in this file's own usage comment now does.

Status: implemented and unit-tested; verified live against a real,
unmodified agent under real induced packet loss, with real measured
retransmit counts and delivery numbers as shown above -- not simulated,
not asserted.

## Phase 7d — Priority-aware executor

Replaces the naive "handle whatever arrives next" pattern every earlier
demo used with dispatch driven by `rtos/`'s real priority scheduler
(`rtos/src/scheduler.c`'s highest-priority-READY-wins `pick_next_ready()`,
already built and tested in the kernel's own Phase 4 -- not reinvented
here). `xrce/` is deliberately kept free of any RTOS dependency (portable,
host-native testable on its own -- stated in this file's own header
comment), so the priority-aware dispatch logic lives in a new
`host/live_priority_demo.c` rather than a new `xrce/src/executor.c`: this
is where `rtos/` and `xrce/` get linked together for a demo, the same role
`rtos/arm/ros2_demo.c` already plays for the QEMU firmware image, without
either library depending on the other.

Architecture: one POLLER task drains the UDP socket with a non-blocking
`recv()` (`MSG_DONTWAIT`) and dispatches each `DATA` sample into the
matching topic's `queue_t` (`rtos/src/sync.h`, unmodified); two CONSUMER
tasks at different `task_spawn()` priorities block on `queue_receive()`
for their own topic and measure real dispatch latency (queue-arrival to
callback-start, via `gettimeofday()`). The low-priority consumer does
deliberately slow, non-yielding CPU work per message so it's only
interrupted at real `SIGALRM` tick boundaries -- the RTOS's actual
preemption behavior, not a scripted delay. The poller has to be
non-blocking: every task here is a green thread (`ucontext_t`) sharing the
one real OS thread the RTOS's preemption runs on, so a blocking `recv()`
would freeze every task, not just the poller, since there's no second real
thread for the others to run on.

**A real bug, found only by testing against a live agent, not from reading
the scheduler's own code.** The poller's first version called
`task_yield()` (not `task_sleep()`) between polls. Being the
highest-priority task, `task_yield()` only ever leaves it READY, never
BLOCKED, so `pick_next_ready()` kept re-selecting it every cycle -- the
resulting multi-hundred-thousand-iterations-per-second spin loop was
enough to starve the real agent process of scheduling time on this host:
`DATA` that an unmodified `live_subscribe_demo.c` receives instantly at
the same moment on the same machine never arrived at all here, confirmed
by logging `errno` on every poll attempt (`EAGAIN`, for as long as a
minute straight) rather than assuming the fix without checking. Switching
the poller to `task_sleep(1)` -- one real tick between polls instead of a
bare yield -- fixed it completely, and is arguably more realistic anyway:
a real embedded poll loop doesn't spin as fast as the CPU allows either.

**Verified live, with real measured latency numbers, under real load from
the standard `ros2 topic pub -r` CLI on two topics simultaneously** (not
simulated publishers): the high-priority consumer's dispatch latency stays
at **2-5 microseconds** for every single message, run after run, while the
low-priority consumer's latency grows from single-digit microseconds up to
**over 100 milliseconds** as it falls behind under the same load -- a
20,000x+ gap, both numbers real and printed by the demo itself, not
asserted:

```
[high] done: mean=3us max=5us
[low]  done: mean=39144us max=124432us
```

This is the deliverable's exact ask, with real evidence: the high-priority
topic's callback latency stays bounded regardless of what the low-priority
one is doing, and the low-priority one visibly absorbs the delay instead.

Status: implemented and verified live against a real, unmodified agent and
`ros2 topic pub`, with real measured dispatch latencies as shown above.

## Phase 8 — Diagnostics (Option A: piggyback on ROS2)

Chose Option A (the phase brief's own recommended default) over a second,
out-of-band protocol: reuses everything built in Phases 1-7d rather than
inventing new transport, and targets `diagnostic_msgs/msg/DiagnosticArray`
-- a real, already-installed standard interface (confirmed via `ros2
interface show diagnostic_msgs/msg/DiagnosticArray`) -- specifically so
`/rtos/diagnostics` is idiomatic and, in principle, `rqt_robot_monitor`-
compatible, per the brief's own suggestion. Stated plainly rather than
overclaimed: `rqt_robot_monitor` itself wasn't independently run against
this topic in this pass, only confirmed wire-compatible via `ros2 topic
echo diagnostic_msgs/msg/DiagnosticArray`.

Every field this phase asks for maps onto `DiagnosticStatus.values[]`
`KeyValue` pairs naturally -- one `DiagnosticStatus` per task, one for
scheduler-wide stats, one per queue, one for middleware stats -- with real
data behind every one of them, not placeholders:
- **Per-task state/priority/stack high-water mark**: state and priority
  were already tracked (`task_t.state`/`.priority`); high-water mark is
  new (`task_stack_high_water_mark()`, `rtos/src/task.c`/`.h`) -- the
  whole stack (beyond the existing bottom-guard canary region) gets
  poisoned with a *different* byte (`TASK_HWM_POISON_BYTE`, distinct from
  the canary's, so the two features are never confusable when eyeballing
  a raw memory dump) at creation, and the accessor scans from the bottom
  up for the first still-poisoned byte -- the deepest point the stack
  pointer has ever actually reached. Verified with a real proportional
  test, not just "nonzero": `rtos/tests/test_diagnostics.c` asserts a
  40-deep recursive task's HWM is measurably larger than a 2-deep one's,
  and that a task that never runs has one bounded well under the full
  stack size.
- **Scheduler stats**: `scheduler_switch_count()` (`scheduler.c`/`.h`,
  new) is a real context-switch counter, deliberately distinct from the
  existing `scheduler_tick_count()` -- a tick doesn't always cause a
  switch, and a switch can happen without a tick (voluntary
  `task_yield()`/blocking on a sync primitive), so conflating them would
  under- or over-count. Incremented at both places a real task-to-task
  `swapcontext()` happens (`scheduler_run()`'s own dispatch loop and
  `preempt_handler()`'s forced one), reset by `scheduler_reset()` like
  every other counter. "Missed deadlines": not tracked, stated honestly
  rather than invented -- this RTOS has no deadline concept at all yet,
  only `task_sleep()` wake times.
- **Queue/mailbox depths**: directly computable from `queue_t`'s existing
  `head`/`tail`/`capacity` (`(tail - head + capacity) % capacity`) --
  nothing new needed. **Drop counts**: not implemented -- `queue_send()`
  blocks rather than drops on a full queue (by design, per `rtos/`'s own
  producer/consumer semantics), so there is no real drop-count data to
  report yet; stated as a known gap rather than fabricated.
- **Middleware-layer stats**: bytes sent, tracked locally in the demo
  (`g_bytes_sent`, incremented at every real `sendto()`) since no existing
  library layer tracked this. Retransmits: genuinely 0 in this demo,
  since it only uses the best-effort stream (7c's reliable stream +
  retransmit-count tracking exists and is real, just not wired into this
  particular demo, which values a working live diagnostics loop over
  needing to *also* stand up a reliable topic just to have a nonzero
  number to show) -- the `DiagnosticStatus` message for this says so
  explicitly (`"best-effort only in this demo -- no retransmits
  possible"`) rather than silently reporting a misleadingly-reassuring 0.

`host/live_diagnostics_demo.c` runs two representative worker tasks (real,
changing task state to actually observe) plus a publisher task (periodic)
and a coordinator task (polls for `/rtos/diagnostics/refresh` requests,
reusing 7b's Requester/Replier machinery completely unmodified).

**Verified live against a real, unmodified agent and `ros2` CLI:**
`ros2 topic echo /rtos/diagnostics diagnostic_msgs/msg/DiagnosticArray`
shows real, correct, changing values --
`stack_high_water_mark_bytes: '584'`/`'536'` for the two worker tasks (not
equal, reflecting their actually different stack usage), `tick_count` and
`context_switch_count` climbing in real time, `bytes_sent` growing with
every publish. `ros2 service call /rtos/diagnostics/refresh
std_srvs/srv/Trigger` gets a real `success=True, message='refreshed'`
response and triggers a real, out-of-cycle republish (`refresh requested
-- republishing` in the server's own log, timed to match).

Status: implemented, unit-tested (`task_stack_high_water_mark()`/
`scheduler_switch_count()` in `rtos/tests/test_diagnostics.c`, CDR
round-trip in `xrce/tests/test_msgs.c`), and verified live against a real,
unmodified agent and `ros2` CLI, both the topic and the service.

## Phase 9 — Live terminal UI

`host/rtos_top.py`: an `htop`-style live view of `/rtos/diagnostics`
(Phase 8). Python + the standard-library `curses` module rather than C:
this WSL image has the ncurses runtime libraries but not
`libncurses-dev`, and non-interactive `sudo` isn't available in this
environment to install it (`sudo -n apt-get install` fails with "a
password is required") -- confirmed rather than assumed, before deciding
to route around it. Python's `curses` module is a real binding to the
same ncurses library the phase brief names, not a from-scratch
reimplementation, and needs no separate package. Uses `rclpy` directly
rather than this project's own hand-rolled `xrce/` client -- Phase 8
already chose "piggyback on ROS2" for `/rtos/diagnostics`, so it's a real,
standard topic at this point, and consuming a real ROS2 topic with the
real ROS2 client library is the idiomatic choice, not a compromise; this
tool has no involvement in the RTOS's own embedded protocol stack the way
the `host/*.c` demos do.

**A real bug in the demo, not the TUI, found while trying to verify this
combination live**: `host/live_priority_demo.c` (Phase 7d) was a one-shot
benchmark -- 15 messages per topic, then exit -- tuned for printing a
clean mean/max summary. On an unloaded loopback link that finishes in
well under a second, which routinely raced a fresh `rclpy` subscriber's
own DDS discovery time: the demo would already be done and gone before
`rtos_top.py` had even finished matching with its publisher, so the
screen just showed "waiting for /rtos/diagnostics..." forever. Confirmed
directly (not guessed) by checking `ps` mid-run and finding the demo
process already exited while the TUI was still discovering. Fixed by
wrapping `high_task`/`low_task`/`poller_task`'s bodies in an outer
`for(;;)`, so the demo now runs continuous rounds instead of one --
printing the same per-round mean/max as before (values already recorded
in Phase 7d's writeup above are unaffected, just now repeating), and
giving a live TUI actual continuous data to watch. `live_priority_demo.c`
also gained its own `/rtos/diagnostics` publisher (same topic/type as
Phase 8's `live_diagnostics_demo.c`, reporting this demo's own
poller/high/low tasks instead) specifically so the tool can watch the
*real* Phase 7d priority-load scenario live, not a separate idle stand-in
for it.

**Verified live end to end**, capturing the real terminal screen via
`script` (curses needs a real pty, which this session's tool harness
doesn't provide directly) while a real agent, `live_priority_demo.c`, and
two real `ros2 topic pub -r 20` publishers all ran concurrently: the
rendered table shows real, correct, live-updating values --
`poller RUNNING 100 23752` / `high BLOCKED 50 2056` / `low READY 10 2056`
(name/state/priority/stack-high-water-mark columns) and
`context_switch_count` climbing across frames, with the `low` task's
own state genuinely flipping between `READY` and `BLOCKED` frame to frame
as it falls in and out of contention -- an actual moving picture of the
scheduling behavior Phase 7d measured numerically, not just a static
snapshot.

Status: implemented and verified live against a real, unmodified agent,
`live_priority_demo.c` under real `ros2 topic pub -r` load, and a real
captured terminal screen showing live updates.

## Phase 10 — Multi-node scaling

Closes the "No multi-node support" gap called out in Phase 6's known-
limitations list above: N separately-built `rtos/arm/ros2_demo.c` firmware
images, each booted under its own QEMU instance, all appearing as N
distinct ROS2 nodes to **one** `MicroXRCEAgent` process -- not N agent
processes, and not a custom host-side relay.

**What actually needed to change, and what didn't.** The real agent
already natively demuxes multiple clients on one process via its
`multiserial` transport mode -- that's existing eProsima agent behavior,
not something this project builds. So the work was entirely on this
project's side of the boundary:
- `rtos/arm/ros2_demo.c`'s `client_key` (last byte), DDS participant name,
  and every topic name (`rt/chatter`, `rt/setpoint`, `rt/pong`) were
  string/byte literals, fixed at one value -- exactly the limitation
  Phase 6 flagged. Parametrized all four behind a `NODE_ID` compile-time
  define (default 0, so `make ros2_demo` with no `NODE_ID` is
  byte-identical to every previous phase). Topic/participant name
  substitution uses `#x`/token-pasting stringification
  (`"rt/chatter_" TOSTRING(NODE_ID)`) rather than runtime `snprintf`,
  since this is still a freestanding, no-libc build.
- `rtos/arm/Makefile` threads `NODE_ID` through as `-DNODE_ID=$(NODE_ID)`
  and namespaces every object/ELF/BIN path by it
  (`build/ros2/node$(NODE_ID)/`, `build/ros2_demo_node$(NODE_ID).elf`) so
  `make ros2_demo NODE_ID=1` and `NODE_ID=2` don't clobber each other's
  build output when run back to back.
- `host/run_multi_node.sh` (new): builds N firmware variants, boots N
  QEMU instances each with its own `-serial pty`, greps each instance's
  stderr for the `/dev/pts/N` path QEMU allocates, and starts **one**
  `MicroXRCEAgent multiserial` process spanning all of them.

**A real agent-CLI gotcha found by testing, not guessed:** the obvious
`-D /dev/pts/3 -D /dev/pts/4` (repeated flag) and `-D /dev/pts/3,/dev/pts/4`
(comma-separated) both fail silently -- the former keeps only one device,
the latter is parsed as one literal (bogus) path, confirmed by watching
the agent log for both and seeing `Waiting for devices: /dev/pts/3,/dev/pts/4`
verbatim as a single string. Reading the agent's own argument parser
(`include/uxr/agent/utils/ArgumentParser.hpp`'s `MultiSerialArgs::devs()`
in the `Micro-XRCE-DDS-Agent` source this project builds against) showed
why: `-D`'s value is fed through `std::istringstream` word-splitting, so
it wants **one shell argument, space-separated**:
`-D "/dev/pts/3 /dev/pts/4"`. (A `-f <file>`, one path per line, works
too -- not used here since the two-instance case doesn't need it.)

**Verified live against a real, unmodified agent, 3 simultaneous QEMU
boards:**
- All 9 expected topics present simultaneously with zero collisions:
  `ros2 topic list` showed `/chatter_{1,2,3}`, `/setpoint_{1,2,3}`,
  `/pong_{1,2,3}`, each backed by a distinct `client_key`
  (`0x52544F54`/`55`/`56` in the agent's own log, i.e. `NODE_ID`'s value
  landing in the key's last byte exactly as intended).
- **Independent data, not just independent names:** `ros2 topic echo
  /chatter_1 --once` and `/chatter_2 --once` returned different counter
  values (`17785` vs `19793`) from the two boards' independently-running
  publish loops.
- **Isolation under load, not just at rest:** `ros2 topic pub /setpoint_1
  std_msgs/msg/Int32 "{data: 111}" --once` produced `data: 111` on
  `/pong_1` (that board's real round-trip echo, same mechanism Phase 6's
  latency benchmark uses) -- and confirmed **absent** on `/pong_2`, i.e.
  node 2 never saw node 1's command. Two failure modes this specifically
  rules out: the agent silently merging same-named entities across
  clients, and this project's own firmware misrouting a datareader across
  the two builds.
- `ros2 node list` returns empty for both -- pre-existing behavior, not a
  regression from this phase: these are raw XRCE-DDS participants (XML
  entity creation over the wire), not real `rclcpp`/`rclpy` nodes, so they
  don't publish the extra node-graph metadata `ros2 node list` looks for.
  Topics/data are real and correct regardless; only that one introspection
  command doesn't apply here, same as before Phase 10.

**Known limitation this phase doesn't touch:** `host/udp_loss_proxy.py`
(Phase 7c's fault-injection tool) is still hardwired 1:1 -- it wasn't
needed for the serial/`multiserial` path this phase uses, so it wasn't
touched. A UDP-based multi-node path, if ever wanted, would need that
proxy generalized to N:1 (source-address-keyed forwarding) separately;
serial's per-QEMU-instance ptys sidestepped the whole question here.

Status: implemented and verified live -- 3 simultaneous QEMU boards, one
real unmodified agent process, independently-flowing data confirmed in
both directions, zero cross-node leakage observed.

## Later phases

Every planned phase (1-12) has now landed. Future work, if any, gets
tracked at the top-level project plan level; this file's convention (a
new dated section per phase) stops here since there are no more phases
queued.
