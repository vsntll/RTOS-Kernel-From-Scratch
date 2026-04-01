#ifndef XRCE_SERIAL_TRANSPORT_H
#define XRCE_SERIAL_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Wire-compatible reimplementation of the Micro XRCE-DDS "Serial Transport"
 * framing (the same framing a real micro-ROS board speaks over its debug
 * UART, and what an unmodified `micro-ros-agent --serial` expects). Frame
 * layout, on the wire, before byte-stuffing:
 *
 *   0x7E  src_addr  dst_addr  len_lsb  len_msb  payload[len]  crc_lsb  crc_msb
 *
 * Every byte after the leading 0x7E is individually stuffed: a byte equal to
 * FLAG (0x7E) or ESC (0x7D) becomes the two-byte sequence ESC, byte ^ 0x20.
 * The leading 0x7E is never stuffed -- it's the delimiter a reader scans
 * for, and any unescaped 0x7E seen mid-frame means "a new frame just
 * started here", used to resync after a corrupted/truncated frame.
 *
 * The CRC is CRC-16/ARC (poly 0x8005, reflected in and out, init 0, no
 * xorout) computed over the payload bytes ONLY -- not the leading flag, not
 * src/dst/len. This detail is easy to get wrong from the public docs (which
 * describe the CRC as covering the whole frame including the flag); it was
 * ground-truthed against eProsima's reference client source
 * (src/c/profile/transport/stream_framing/stream_framing_protocol.c) rather
 * than implemented from memory, since strict interop requires it to be
 * byte-exact.
 *
 * Reliability (retransmit on loss) deliberately does NOT live at this
 * layer, even though it's within scope for a from-scratch Phase 1 transport
 * -- the real protocol puts reliability in the XRCE Reliable Stream
 * (heartbeat/acknack, Phase 3+), not in the serial framing. Doing ad hoc
 * retry here would just be incompatible noise the real agent doesn't speak.
 * What this layer does guarantee: corrupt or truncated frames are detected
 * (bad CRC, or never completing) and dropped without wedging the reader, so
 * the next well-formed frame after some lost/garbled bytes is still parsed
 * correctly -- see xrce_serial_reader_feed().
 */

#define XRCE_SERIAL_FLAG 0x7E
#define XRCE_SERIAL_ESC 0x7D
#define XRCE_SERIAL_XOR 0x20

/* Fixed per the spec: address, address, len_lsb, len_msb before stuffing;
 * crc_lsb, crc_msb after the payload, before stuffing. Worst case (every
 * byte needs stuffing) an encoded frame is 1 + 2*(4 + payload_len + 2)
 * bytes; callers sizing a raw output buffer should use
 * XRCE_SERIAL_MAX_ENCODED_LEN(payload_len). */
#define XRCE_SERIAL_HEADER_LEN 4
#define XRCE_SERIAL_CRC_LEN 2
#define XRCE_SERIAL_MAX_ENCODED_LEN(payload_len) \
    (1 + 2 * (XRCE_SERIAL_HEADER_LEN + (payload_len) + XRCE_SERIAL_CRC_LEN))

uint16_t xrce_serial_crc16(const uint8_t *data, size_t len);

/* Encodes one complete frame (flag + stuffed header/payload/crc) into `out`.
 * Returns the number of bytes written, or 0 if `out_cap` is too small to
 * hold the worst-case stuffed frame. No dynamic allocation -- caller owns
 * `out`, sized for a real task's outgoing packet on an embedded target. */
size_t xrce_serial_frame_encode(uint8_t *out, size_t out_cap,
                                 uint8_t local_addr, uint8_t remote_addr,
                                 const uint8_t *payload, size_t payload_len);

typedef enum {
    XRCE_SERIAL_WAIT_FLAG,
    XRCE_SERIAL_READ_SRC,
    XRCE_SERIAL_READ_DST,
    XRCE_SERIAL_READ_LEN_LSB,
    XRCE_SERIAL_READ_LEN_MSB,
    XRCE_SERIAL_READ_PAYLOAD,
    XRCE_SERIAL_READ_CRC_LSB,
    XRCE_SERIAL_READ_CRC_MSB
} xrce_serial_state_t;

typedef struct {
    xrce_serial_state_t state;
    uint8_t local_addr;
    uint8_t src_addr;
    uint16_t len;
    uint16_t pos;
    uint16_t crc;
    uint16_t rx_crc;
    bool have_escape;
    uint8_t *payload_buf;
    size_t payload_cap;
} xrce_serial_reader_t;

/* `payload_buf`/`payload_cap` is caller-owned storage a completed frame's
 * payload gets written into -- sized once at init time, no allocation
 * on the hot path. */
void xrce_serial_reader_init(xrce_serial_reader_t *r, uint8_t local_addr,
                              uint8_t *payload_buf, size_t payload_cap);

typedef enum {
    XRCE_SERIAL_FEED_IN_PROGRESS, /* byte consumed, frame not complete yet */
    XRCE_SERIAL_FEED_FRAME_READY, /* byte completed a frame; payload_buf/out_* valid */
    XRCE_SERIAL_FEED_ERROR        /* bad CRC or payload too big for payload_cap; frame dropped, reader resynced */
} xrce_serial_feed_result_t;

/* Feeds one raw (still-stuffed) byte as read off the wire into the state
 * machine. On XRCE_SERIAL_FEED_FRAME_READY, `*out_src_addr` and `*out_len`
 * are set and r->payload_buf[0..*out_len) holds the decoded payload. */
xrce_serial_feed_result_t xrce_serial_reader_feed(xrce_serial_reader_t *r,
                                                   uint8_t byte,
                                                   uint8_t *out_src_addr,
                                                   uint16_t *out_len);

#endif
