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

## Phase 5 — Subscriptions (bidirectional communication) -- protocol working over UDP, serial wiring not yet working

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

**Attempted, not yet working: wiring this into `rtos/arm/ros2_demo.c`
over the real serial transport.** `uart.c` gained polled RX
(`uart_getc_nonblocking()`), and the firmware now creates a full
subscriber/datareader for `rt/setpoint` alongside the existing publisher,
feeding received bytes through `xrce_serial_reader_feed()` and printing
`setpoint updated: N` when a matching `DATA` submessage decodes. Tested
against a real agent (via a small Python bidirectional pty relay so both
QEMU's UART traffic and the human-readable prints could be observed at
once, since the agent otherwise holds the pty exclusively) -- entity
creation succeeds every time (`datareader created` in the agent's log),
and the agent's own log confirms it reads the published value from DDS
(`[==>> DDS <<==]`, correct header-less bytes), but no `DATA` submessage
was ever observed reaching the firmware, checked directly in the raw pty
traffic. Two contributing-but-not-fully-explaining factors found along
the way:
- Re-announcing `READ_DATA` too often (a 5-publish interval, ~170ms under
  QEMU's fast busy-loop) keeps restarting the agent's internal delivery
  thread (`DataReader::read()` always calls `stop_reading()` before
  `start_reading()`), which alone could starve real deliveries. Backed off
  to a much longer interval (`REANNOUNCE_EVERY_N_PUBLISHES = 200` in
  `ros2_demo.c`) -- necessary, but not sufficient: delivery still didn't
  arrive with this change alone.
- Sending `READ_DATA` exactly once (the alternative extreme) reintroduces
  the same boot-timing race `CREATE_CLIENT`'s periodic re-announcement
  exists to solve in the first place (the agent takes a few real seconds
  to attach to a freshly-allocated pty; a one-time send at boot is easily
  lost).

Since the identical `READ_DATA`/`DATA` code, unchanged, works correctly
and repeatedly over UDP, the protocol implementation itself is not in
question here -- what's unresolved is specific to this transport/test
combination (real serial, through a manually-bridged pty pair rather than
a direct QEMU-to-agent connection). Not root-caused further in this pass;
a reasonable next step would be tracing the agent's
`SessionManager::get_endpoint()`/`push_output_submessage()` path for the
serial transport specifically, or testing with a real (non-emulated)
serial connection to rule out the pty-bridging setup itself as a factor.

Also not yet done: services (request/reply, a different submessage pair
again), and a `ros2 topic pub`-to-QEMU demo confirmed working end to end.

## Later phases

Not started (services/request-reply, a realistic combined publish+
subscribe demo scenario, multi-node, latency/fault-handling writeup).
Tracked at a high level in the top-level project plan; this file gets a
new dated section per phase as it actually lands, same convention as
`rtos/docs/design.md`.
