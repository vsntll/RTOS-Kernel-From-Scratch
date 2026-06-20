#include "xrce/cdr.h"

#include <string.h>

/* Explicit little-endian byte packing rather than memcpy + assuming the
 * host is LE: both the WSL/x86_64 host and the Cortex-M4 QEMU target
 * happen to be LE, so it wouldn't currently matter, but CDR_LE is a wire
 * format contract, not a host-endianness accident, and should read as
 * one. */

static bool ensure(size_t cap, size_t pos, size_t n) {
    return n <= cap - pos; /* cap - pos can't underflow: pos <= cap always */
}

static void align_writer(xrce_cdr_writer_t *w, size_t n) {
    size_t rem = w->pos % n;
    if (rem != 0) {
        size_t pad = n - rem;
        for (size_t i = 0; i < pad && w->pos < w->cap; i++) {
            w->buf[w->pos++] = 0;
        }
    }
}

static bool align_reader(xrce_cdr_reader_t *r, size_t n) {
    size_t rem = r->pos % n;
    if (rem == 0) {
        return true;
    }
    size_t pad = n - rem;
    if (!ensure(r->len, r->pos, pad)) {
        return false;
    }
    r->pos += pad;
    return true;
}

void xrce_cdr_writer_init(xrce_cdr_writer_t *w, uint8_t *buf, size_t cap) {
    w->buf = buf;
    w->cap = cap;
    w->pos = 0;
}

bool xrce_cdr_write_header(xrce_cdr_writer_t *w) {
    if (!ensure(w->cap, w->pos, 4)) {
        return false;
    }
    w->buf[w->pos++] = 0x00;
    w->buf[w->pos++] = 0x01; /* representation_id = CDR_LE */
    w->buf[w->pos++] = 0x00;
    w->buf[w->pos++] = 0x00; /* representation_options, unused */
    return true;
}

bool xrce_cdr_write_bool(xrce_cdr_writer_t *w, bool v) {
    return xrce_cdr_write_u8(w, v ? 1 : 0);
}

bool xrce_cdr_write_u8(xrce_cdr_writer_t *w, uint8_t v) {
    if (!ensure(w->cap, w->pos, 1)) {
        return false;
    }
    w->buf[w->pos++] = v;
    return true;
}

bool xrce_cdr_write_i16(xrce_cdr_writer_t *w, int16_t v) {
    uint16_t u = (uint16_t)v;
    align_writer(w, 2);
    if (!ensure(w->cap, w->pos, 2)) {
        return false;
    }
    w->buf[w->pos++] = (uint8_t)(u & 0xFF);
    w->buf[w->pos++] = (uint8_t)((u >> 8) & 0xFF);
    return true;
}

bool xrce_cdr_write_bytes(xrce_cdr_writer_t *w, const uint8_t *data, size_t len) {
    if (!ensure(w->cap, w->pos, len)) {
        return false;
    }
    memcpy(&w->buf[w->pos], data, len);
    w->pos += len;
    return true;
}

bool xrce_cdr_write_i32(xrce_cdr_writer_t *w, int32_t v) {
    return xrce_cdr_write_u32(w, (uint32_t)v);
}

bool xrce_cdr_write_u32(xrce_cdr_writer_t *w, uint32_t v) {
    align_writer(w, 4);
    if (!ensure(w->cap, w->pos, 4)) {
        return false;
    }
    for (int i = 0; i < 4; i++) {
        w->buf[w->pos++] = (uint8_t)(v >> (8 * i));
    }
    return true;
}

bool xrce_cdr_write_f64(xrce_cdr_writer_t *w, double v) {
    uint64_t bits;
    memcpy(&bits, &v, sizeof(bits)); /* reinterpret, not convert */
    align_writer(w, 8);
    if (!ensure(w->cap, w->pos, 8)) {
        return false;
    }
    for (int i = 0; i < 8; i++) {
        w->buf[w->pos++] = (uint8_t)(bits >> (8 * i));
    }
    return true;
}

bool xrce_cdr_write_string(xrce_cdr_writer_t *w, const char *s) {
    size_t slen = strlen(s) + 1; /* CDR strings include the terminator */
    if (slen > UINT32_MAX) {
        return false;
    }
    if (!xrce_cdr_write_u32(w, (uint32_t)slen)) {
        return false;
    }
    if (!ensure(w->cap, w->pos, slen)) {
        return false;
    }
    memcpy(&w->buf[w->pos], s, slen);
    w->pos += slen;
    return true;
}

bool xrce_cdr_write_seq_i32(xrce_cdr_writer_t *w, const int32_t *elems, uint32_t count) {
    if (!xrce_cdr_write_u32(w, count)) {
        return false;
    }
    for (uint32_t i = 0; i < count; i++) {
        if (!xrce_cdr_write_i32(w, elems[i])) {
            return false;
        }
    }
    return true;
}

void xrce_cdr_reader_init(xrce_cdr_reader_t *r, const uint8_t *buf, size_t len) {
    r->buf = buf;
    r->len = len;
    r->pos = 0;
}

bool xrce_cdr_read_header(xrce_cdr_reader_t *r) {
    if (!ensure(r->len, r->pos, 4)) {
        return false;
    }
    bool is_cdr_le = (r->buf[r->pos] == 0x00) && (r->buf[r->pos + 1] == 0x01);
    r->pos += 4;
    return is_cdr_le;
}

bool xrce_cdr_read_bool(xrce_cdr_reader_t *r, bool *out) {
    uint8_t v;
    if (!xrce_cdr_read_u8(r, &v)) {
        return false;
    }
    *out = (v != 0);
    return true;
}

bool xrce_cdr_read_u8(xrce_cdr_reader_t *r, uint8_t *out) {
    if (!ensure(r->len, r->pos, 1)) {
        return false;
    }
    *out = r->buf[r->pos++];
    return true;
}

bool xrce_cdr_read_i16(xrce_cdr_reader_t *r, int16_t *out) {
    if (!align_reader(r, 2) || !ensure(r->len, r->pos, 2)) {
        return false;
    }
    uint16_t v = (uint16_t)(r->buf[r->pos] | ((uint16_t)r->buf[r->pos + 1] << 8));
    r->pos += 2;
    *out = (int16_t)v;
    return true;
}

bool xrce_cdr_read_bytes(xrce_cdr_reader_t *r, uint8_t *out, size_t len) {
    if (!ensure(r->len, r->pos, len)) {
        return false;
    }
    memcpy(out, &r->buf[r->pos], len);
    r->pos += len;
    return true;
}

bool xrce_cdr_read_i32(xrce_cdr_reader_t *r, int32_t *out) {
    uint32_t u;
    if (!xrce_cdr_read_u32(r, &u)) {
        return false;
    }
    *out = (int32_t)u;
    return true;
}

bool xrce_cdr_read_u32(xrce_cdr_reader_t *r, uint32_t *out) {
    if (!align_reader(r, 4) || !ensure(r->len, r->pos, 4)) {
        return false;
    }
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) {
        v |= (uint32_t)r->buf[r->pos + (size_t)i] << (8 * i);
    }
    r->pos += 4;
    *out = v;
    return true;
}

bool xrce_cdr_read_f64(xrce_cdr_reader_t *r, double *out) {
    if (!align_reader(r, 8) || !ensure(r->len, r->pos, 8)) {
        return false;
    }
    uint64_t bits = 0;
    for (int i = 0; i < 8; i++) {
        bits |= (uint64_t)r->buf[r->pos + (size_t)i] << (8 * i);
    }
    r->pos += 8;
    memcpy(out, &bits, sizeof(bits));
    return true;
}

bool xrce_cdr_read_string(xrce_cdr_reader_t *r, char *out, size_t out_cap) {
    uint32_t slen;
    if (!xrce_cdr_read_u32(r, &slen)) {
        return false;
    }
    if (slen == 0 || slen > out_cap || !ensure(r->len, r->pos, slen)) {
        return false;
    }
    memcpy(out, &r->buf[r->pos], slen);
    r->pos += slen;
    if (out[slen - 1] != '\0') {
        return false; /* malformed: CDR strings are always NUL-terminated */
    }
    return true;
}

bool xrce_cdr_read_seq_i32(xrce_cdr_reader_t *r, int32_t *out, size_t out_cap, uint32_t *out_count) {
    uint32_t count;
    if (!xrce_cdr_read_u32(r, &count) || count > out_cap) {
        return false;
    }
    for (uint32_t i = 0; i < count; i++) {
        if (!xrce_cdr_read_i32(r, &out[i])) {
            return false;
        }
    }
    *out_count = count;
    return true;
}
