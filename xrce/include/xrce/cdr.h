#ifndef XRCE_CDR_H
#define XRCE_CDR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Minimal CDR (Common Data Representation) reader/writer -- the wire
 * encoding ROS2 message fields use, both over DDS/RTPS on the ROS2 side
 * and (with the 4-byte "encapsulation header" this implements) over the
 * XRCE WRITE_DATA payload a Micro XRCE-DDS client sends. Little-endian
 * only (CDR_LE, representation id 0x0001) -- what every mainstream ROS2
 * platform (x86_64, aarch64, and Cortex-M under QEMU) actually uses; a
 * real client never needs to emit CDR_BE.
 *
 * Alignment: each primitive is padded so its start offset, measured from
 * the very beginning of the buffer (i.e. including the 4-byte header,
 * which counts as already-consumed alignment), is a multiple of the
 * primitive's own size (1/2/4/8). This matches Fast-CDR's behavior, which
 * is what a real ROS2 node's DDS participant is actually running --
 * getting this wrong produces bytes a real subscriber silently
 * misinterprets rather than rejects, so it's exactly the kind of detail
 * worth stating explicitly rather than leaving implicit. */

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t pos;
} xrce_cdr_writer_t;

typedef struct {
    const uint8_t *buf;
    size_t len;
    size_t pos;
} xrce_cdr_reader_t;

void xrce_cdr_writer_init(xrce_cdr_writer_t *w, uint8_t *buf, size_t cap);
/* Writes the 4-byte CDR_LE encapsulation header ([0x00, 0x01, 0x00, 0x00]).
 * Must be the first thing written -- every alignment calculation after
 * this assumes it's present. */
bool xrce_cdr_write_header(xrce_cdr_writer_t *w);

bool xrce_cdr_write_bool(xrce_cdr_writer_t *w, bool v);
bool xrce_cdr_write_u8(xrce_cdr_writer_t *w, uint8_t v);
bool xrce_cdr_write_i16(xrce_cdr_writer_t *w, int16_t v);
bool xrce_cdr_write_i32(xrce_cdr_writer_t *w, int32_t v);
bool xrce_cdr_write_u32(xrce_cdr_writer_t *w, uint32_t v);
bool xrce_cdr_write_f64(xrce_cdr_writer_t *w, double v);
/* Raw byte copy, no alignment beyond 1 -- matches CDR's array_uint8_t
 * semantics (what the XRCE spec uses for fields like ObjectId/RequestId/
 * ClientKey: fixed-size byte arrays with no further internal structure). */
bool xrce_cdr_write_bytes(xrce_cdr_writer_t *w, const uint8_t *data, size_t len);
/* CDR string: uint32 length (byte count INCLUDING the null terminator),
 * then that many bytes (including the terminator). `s` must be a normal
 * C string. */
bool xrce_cdr_write_string(xrce_cdr_writer_t *w, const char *s);

void xrce_cdr_reader_init(xrce_cdr_reader_t *r, const uint8_t *buf, size_t len);
/* Consumes and validates the 4-byte header; fails (returns false) on
 * anything other than CDR_LE, since that's the only thing this project
 * ever produces or expects to consume. */
bool xrce_cdr_read_header(xrce_cdr_reader_t *r);

bool xrce_cdr_read_bool(xrce_cdr_reader_t *r, bool *out);
bool xrce_cdr_read_u8(xrce_cdr_reader_t *r, uint8_t *out);
bool xrce_cdr_read_i16(xrce_cdr_reader_t *r, int16_t *out);
bool xrce_cdr_read_i32(xrce_cdr_reader_t *r, int32_t *out);
bool xrce_cdr_read_u32(xrce_cdr_reader_t *r, uint32_t *out);
bool xrce_cdr_read_f64(xrce_cdr_reader_t *r, double *out);
bool xrce_cdr_read_bytes(xrce_cdr_reader_t *r, uint8_t *out, size_t len);
/* Writes up to out_cap-1 bytes into `out` and NUL-terminates; fails if the
 * encoded string (including its terminator) doesn't fit `out_cap`, rather
 * than silently truncating a real payload. */
bool xrce_cdr_read_string(xrce_cdr_reader_t *r, char *out, size_t out_cap);

#endif
