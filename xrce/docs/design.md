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
`make test`, clean under `-fsanitize=address,undefined`). Not yet wired
into a QEMU firmware image or exercised against a real UART -- that lands
with the Phase 0 QEMU port and the Phase 1 end-to-end deliverable (RTOS
task in QEMU exchanging framed packets with a host peer).

## Later phases

Not started yet. Tracked at a high level in the top-level project plan;
this file gets a new dated section per phase as it actually lands, same
convention as `rtos/docs/design.md`.
