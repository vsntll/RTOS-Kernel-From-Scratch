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
 * Alignment: each primitive is padded so its start offset is a multiple of
 * the primitive's own size (1/2/4/8), measured from `align_base` -- 0 for
 * a plain writer/reader, or the position right after the 4-byte header
 * for one that called xrce_cdr_write_header()/xrce_cdr_read_header().
 * This matches Fast-CDR's actual behavior (confirmed against real
 * `rclpy.serialization.serialize_message()` output, not assumed): the
 * encapsulation header does NOT count toward subsequent field alignment,
 * i.e. the first field after it is treated as starting at a fresh offset
 * 0, not at absolute offset 4. An earlier version of this comment (and
 * this file's code) claimed the opposite -- that the header's 4 bytes
 * count as already-consumed alignment -- which is wrong and went
 * undetected because it happened to not matter for every message type
 * this project had live-verified so far (std_msgs/Int32, trivially;
 * sensor_msgs/Imu was CDR-tested but never live-verified against a real
 * subscriber). Phase 14's geometry_msgs/Twist -- header immediately
 * followed by a float64, the simplest case where the two schemes
 * actually disagree -- surfaced it: a real ROS2 subscriber's RTPS reader
 * rejected every sample with "payload size of '56' bytes is larger than
 * the history payload size of '55'" (its own generated typesupport
 * expects 52; see xrce/docs/design.md's Phase 14 section for the byte-
 * for-byte comparison against a captured real `rclpy`-serialized Twist).
 * Fixed here, in the one shared place both Imu and Twist (and any future
 * message type) go through, rather than worked around per call site. */

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t pos;
    size_t align_base;
} xrce_cdr_writer_t;

typedef struct {
    const uint8_t *buf;
    size_t len;
    size_t pos;
    size_t align_base;
} xrce_cdr_reader_t;

void xrce_cdr_writer_init(xrce_cdr_writer_t *w, uint8_t *buf, size_t cap);
/* Writes the 4-byte CDR_LE encapsulation header ([0x00, 0x01, 0x00, 0x00])
 * and resets align_base to right after it, so every alignment calculation
 * after this treats that point as offset 0 -- must be the first thing
 * written if it's called at all. Callers that never call this (e.g.
 * session.c's message/submessage headers, a different framing layer with
 * its own, unrelated header) keep align_base at 0 from _init(), unaffected
 * by this. */
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

/* CDR sequence<int32>: uint32 element count, then that many 4-byte-aligned
 * int32 elements -- the layout every unbounded `int32[]` ROS2 field uses
 * (e.g. action feedback/result arrays). Kept as its own primitive rather
 * than a generic templated sequence writer: C has no generics, and this
 * project only ever needs a handful of concrete element types. */
bool xrce_cdr_write_seq_i32(xrce_cdr_writer_t *w, const int32_t *elems, uint32_t count);

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

/* Reads a sequence<int32> into `out` (capacity `out_cap` elements); fails
 * (rather than truncating) if the encoded count exceeds `out_cap`. */
bool xrce_cdr_read_seq_i32(xrce_cdr_reader_t *r, int32_t *out, size_t out_cap, uint32_t *out_count);

#endif
