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

## Phase 4 — Host-side bridge (real agent) -- partial

`host/live_agent_check.c` and `host/live_publish_demo.c` are manual,
one-off programs (BSD sockets, not portable/embeddable, not part of
`make test`) that run this project's session layer against a real,
unmodified `MicroXRCEAgent` over UDP -- the actual point of choosing
Option A: proof against an agent neither of us wrote, not just against our
own reader.

**Verified working, with the agent's own log as evidence, not just this
project's opinion of itself:**
- CREATE_CLIENT: agent logs `create_client` / `session established` for a
  hand-built client key.
- CREATE participant/topic/publisher/datawriter (XML representation):
  agent logs `participant created`, `topic created`, `publisher created`,
  `datawriter created`, each with matching object ids.
- WRITE_DATA reaches the DDS/RTPS layer: `ros2 topic echo /chatter` (real,
  unmodified ROS2 CLI) shows its RTPS reader actively attempting to
  process each published sample -- the message gets all the way from this
  project's hand-rolled client, through the real agent, into real DDS.

**Known gap, narrowed but not yet resolved:** the RTPS reader rejects the
sample with `Change payload size of '12' bytes is larger than the history
payload size of '11' bytes and cannot be resized`. This reproduces
identically under both `-m dds` and `-m rtps` agent middleware modes, which
rules out one initial theory (dynamic-type creation quirks specific to one
middleware backend).

Checked directly against the agent's own verbose (`-v 6`) log rather than
guessed: `DataWriter.cpp`'s `[** <<DDS>> **]` line shows the agent extracts
*exactly* this project's 8-byte CDR-encoded sample (`00 01 00 00 <int32 LE>`,
`len: 8`) from the WRITE_DATA submessage and hands it to
`middleware.write_data()` -- i.e., `Processor::process_write_data_submessage`
and `DataWriter::write()` (both read from the agent's own source,
`src/cpp/processor/Processor.cpp` / `src/cpp/datawriter/DataWriter.cpp`)
strip the `BaseObjectRequest` header correctly and forward this project's
bytes unmodified. So the client and agent sides of the boundary are now
verified byte-exact; the "12 vs 11" mismatch is happening downstream of
that, inside Fast-DDS's own RTPS reader-side history/type-size accounting
(likely related to the topic being declared with a bare type-name string
rather than real type introspection/a TypeObject, but not confirmed to
that level of certainty the way the client/agent boundary now is).

Next concrete step: either give the agent real type introspection for
`std_msgs/Int32` (an `OBJK_TYPE` entity, or a `REF`-based topic pointing at
a profile with real type info, rather than a bare XML name string), or
trace further into Fast-DDS's dynamic-type size inference directly. Not
done in this pass -- called out explicitly rather than left implicit,
since "the agent accepts our messages" and "the full pipe decodes cleanly
end to end" are different claims and only the first is fully nailed down.

Both files' XML entity-representation strings (the `<dds>...</dds>`
wrapper, element-not-attribute children) are ground-truthed against
`Micro-XRCE-DDS-Client/examples/PublishHelloWorld/main.c` -- an earlier
attempt using an attribute-style shorthand (`<dds_topic name="..."
.../>`) failed the agent's XML parser outright (`Not found root tag`),
which is exactly the kind of interop detail this project exists to get
right rather than guess at.

## Later phases

Not started (subscriptions/READ_DATA, services, bidirectional demo,
multi-node, latency/fault-handling writeup). Tracked at a high level in the
top-level project plan; this file gets a new dated section per phase as it
actually lands, same convention as `rtos/docs/design.md`.
