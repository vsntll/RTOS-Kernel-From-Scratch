#include "xrce/serial_transport.h"

/* CRC-16/ARC: poly 0x8005, reflected in/out, init 0, no xorout. Bit-banged
 * rather than table-driven -- this runs once per outgoing packet on an
 * embedded target, not in a hot loop, so the table's ROM cost isn't worth
 * paying for. 0xA001 is 0x8005 with the bits reversed, which is what lets
 * the reflected form shift right instead of left. */
static uint16_t crc16_update(uint16_t crc, uint8_t byte) {
    crc = (uint16_t)(crc ^ byte);
    for (int i = 0; i < 8; i++) {
        if (crc & 1) {
            crc = (uint16_t)((crc >> 1) ^ 0xA001);
        } else {
            crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

uint16_t xrce_serial_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = crc16_update(crc, data[i]);
    }
    return crc;
}

static bool put_stuffed(uint8_t *out, size_t out_cap, size_t *pos, uint8_t byte) {
    if (byte == XRCE_SERIAL_FLAG || byte == XRCE_SERIAL_ESC) {
        if (*pos + 2 > out_cap) {
            return false;
        }
        out[(*pos)++] = XRCE_SERIAL_ESC;
        out[(*pos)++] = (uint8_t)(byte ^ XRCE_SERIAL_XOR);
    } else {
        if (*pos + 1 > out_cap) {
            return false;
        }
        out[(*pos)++] = byte;
    }
    return true;
}

size_t xrce_serial_frame_encode(uint8_t *out, size_t out_cap,
                                 uint8_t local_addr, uint8_t remote_addr,
                                 const uint8_t *payload, size_t payload_len) {
    if (out_cap < 1) {
        return 0;
    }
    size_t pos = 0;
    out[pos++] = XRCE_SERIAL_FLAG; /* delimiter itself is never stuffed */

    if (!put_stuffed(out, out_cap, &pos, local_addr) ||
        !put_stuffed(out, out_cap, &pos, remote_addr) ||
        !put_stuffed(out, out_cap, &pos, (uint8_t)(payload_len & 0xFF)) ||
        !put_stuffed(out, out_cap, &pos, (uint8_t)((payload_len >> 8) & 0xFF))) {
        return 0;
    }

    uint16_t crc = 0;
    for (size_t i = 0; i < payload_len; i++) {
        crc = crc16_update(crc, payload[i]);
        if (!put_stuffed(out, out_cap, &pos, payload[i])) {
            return 0;
        }
    }

    if (!put_stuffed(out, out_cap, &pos, (uint8_t)(crc & 0xFF)) ||
        !put_stuffed(out, out_cap, &pos, (uint8_t)((crc >> 8) & 0xFF))) {
        return 0;
    }

    return pos;
}

void xrce_serial_reader_init(xrce_serial_reader_t *r, uint8_t local_addr,
                              uint8_t *payload_buf, size_t payload_cap) {
    r->state = XRCE_SERIAL_WAIT_FLAG;
    r->local_addr = local_addr;
    r->src_addr = 0;
    r->len = 0;
    r->pos = 0;
    r->crc = 0;
    r->rx_crc = 0;
    r->have_escape = false;
    r->payload_buf = payload_buf;
    r->payload_cap = payload_cap;
}

static void reset_to_wait(xrce_serial_reader_t *r) {
    r->state = XRCE_SERIAL_WAIT_FLAG;
    r->have_escape = false;
}

/* Unstuffs one raw wire byte. Returns false while it's mid-escape (the ESC
 * byte itself was consumed, no decoded octet yet); the caller keeps feeding
 * more raw bytes until this returns true (or the mid-frame-restart case
 * below fires). */
static bool unstuff(xrce_serial_reader_t *r, uint8_t raw, uint8_t *decoded) {
    if (r->have_escape) {
        r->have_escape = false;
        *decoded = (uint8_t)(raw ^ XRCE_SERIAL_XOR);
        return true;
    }
    if (raw == XRCE_SERIAL_ESC) {
        r->have_escape = true;
        return false;
    }
    *decoded = raw;
    return true;
}

xrce_serial_feed_result_t xrce_serial_reader_feed(xrce_serial_reader_t *r,
                                                   uint8_t byte,
                                                   uint8_t *out_src_addr,
                                                   uint16_t *out_len) {
    /* An unescaped FLAG byte always means "a frame starts here", even mid
     * frame -- that's how a reader resyncs after bytes were lost or
     * corrupted instead of getting permanently wedged waiting for a payload
     * that will never arrive. */
    if (!r->have_escape && byte == XRCE_SERIAL_FLAG) {
        r->state = XRCE_SERIAL_READ_SRC;
        r->have_escape = false;
        return XRCE_SERIAL_FEED_IN_PROGRESS;
    }

    if (r->state == XRCE_SERIAL_WAIT_FLAG) {
        return XRCE_SERIAL_FEED_IN_PROGRESS;
    }

    uint8_t decoded;
    if (!unstuff(r, byte, &decoded)) {
        return XRCE_SERIAL_FEED_IN_PROGRESS; /* consumed an ESC, waiting for its pair */
    }

    switch (r->state) {
    case XRCE_SERIAL_WAIT_FLAG:
        return XRCE_SERIAL_FEED_IN_PROGRESS;

    case XRCE_SERIAL_READ_SRC:
        r->src_addr = decoded;
        r->state = XRCE_SERIAL_READ_DST;
        return XRCE_SERIAL_FEED_IN_PROGRESS;

    case XRCE_SERIAL_READ_DST:
        if (decoded != r->local_addr) {
            /* Not addressed to us -- drop and wait for the next flag,
             * same behavior as the reference client's address filter. */
            reset_to_wait(r);
            return XRCE_SERIAL_FEED_ERROR;
        }
        r->state = XRCE_SERIAL_READ_LEN_LSB;
        return XRCE_SERIAL_FEED_IN_PROGRESS;

    case XRCE_SERIAL_READ_LEN_LSB:
        r->len = decoded;
        r->state = XRCE_SERIAL_READ_LEN_MSB;
        return XRCE_SERIAL_FEED_IN_PROGRESS;

    case XRCE_SERIAL_READ_LEN_MSB:
        r->len = (uint16_t)(r->len | ((uint16_t)decoded << 8));
        r->pos = 0;
        r->crc = 0;
        if (r->len > r->payload_cap) {
            reset_to_wait(r);
            return XRCE_SERIAL_FEED_ERROR;
        }
        r->state = (r->len == 0) ? XRCE_SERIAL_READ_CRC_LSB : XRCE_SERIAL_READ_PAYLOAD;
        return XRCE_SERIAL_FEED_IN_PROGRESS;

    case XRCE_SERIAL_READ_PAYLOAD:
        r->payload_buf[r->pos++] = decoded;
        r->crc = crc16_update(r->crc, decoded);
        if (r->pos == r->len) {
            r->state = XRCE_SERIAL_READ_CRC_LSB;
        }
        return XRCE_SERIAL_FEED_IN_PROGRESS;

    case XRCE_SERIAL_READ_CRC_LSB:
        r->rx_crc = decoded;
        r->state = XRCE_SERIAL_READ_CRC_MSB;
        return XRCE_SERIAL_FEED_IN_PROGRESS;

    case XRCE_SERIAL_READ_CRC_MSB:
        r->rx_crc = (uint16_t)(r->rx_crc | ((uint16_t)decoded << 8));
        reset_to_wait(r);
        if (r->rx_crc != r->crc) {
            return XRCE_SERIAL_FEED_ERROR;
        }
        *out_src_addr = r->src_addr;
        *out_len = r->len;
        return XRCE_SERIAL_FEED_FRAME_READY;
    }

    return XRCE_SERIAL_FEED_IN_PROGRESS;
}
