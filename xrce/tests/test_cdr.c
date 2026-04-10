/* Phase 2 milestone (ROS2 layer): CDR primitive read/write. Byte-exact
 * checks on a few cases (not just round-trips) because alignment bugs are
 * exactly the kind of thing that round-trips cleanly against your own
 * (buggy) reader while producing bytes a real Fast-CDR-based ROS2
 * subscriber would misinterpret. */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../include/xrce/cdr.h"

typedef void (*test_case_fn)(void);
static int g_tests_run;

static void run_case(const char *name, test_case_fn fn) {
    printf("[case] %s\n", name);
    fn();
    g_tests_run++;
}

static void case_header_round_trip(void) {
    uint8_t buf[4];
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, sizeof(buf));
    assert(xrce_cdr_write_header(&w));
    assert(w.pos == 4);
    assert(buf[0] == 0x00 && buf[1] == 0x01 && buf[2] == 0x00 && buf[3] == 0x00);

    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, sizeof(buf));
    assert(xrce_cdr_read_header(&r));
}

static void case_header_rejects_big_endian(void) {
    uint8_t buf[4] = {0x00, 0x00, 0x00, 0x00}; /* CDR_BE */
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, sizeof(buf));
    assert(!xrce_cdr_read_header(&r));
}

/* header(4) + u8 (no align, pos 4->5) + i32 (align4: pad 3 to pos8, write
 * 4 LE bytes, pos 12) + f64 (align8: pad 4 to pos16, write 8 LE bytes,
 * pos 24) -- exact expected byte layout, not just "it round-trips". */
static void case_alignment_byte_exact(void) {
    uint8_t buf[24];
    memset(buf, 0xCC, sizeof(buf)); /* poison, so padding bytes are visible */
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, sizeof(buf));

    assert(xrce_cdr_write_header(&w));
    assert(xrce_cdr_write_u8(&w, 0xAB));
    assert(xrce_cdr_write_i32(&w, 0x11223344));
    assert(xrce_cdr_write_f64(&w, 1.0)); /* 0x3FF0000000000000 */
    assert(w.pos == 24);

    static const uint8_t expected[24] = {
        0x00, 0x01, 0x00, 0x00,             /* header */
        0xAB,                               /* u8 */
        0x00, 0x00, 0x00,                   /* pad to 4-align */
        0x44, 0x33, 0x22, 0x11,             /* i32 LE */
        0x00, 0x00, 0x00, 0x00,             /* pad pos 12->16 to 8-align */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F, /* f64 LE */
    };
    assert(memcmp(buf, expected, sizeof(expected)) == 0);

    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, sizeof(buf));
    bool hdr_ok = xrce_cdr_read_header(&r);
    uint8_t u8v;
    int32_t i32v;
    double f64v;
    assert(hdr_ok);
    assert(xrce_cdr_read_u8(&r, &u8v) && u8v == 0xAB);
    assert(xrce_cdr_read_i32(&r, &i32v) && i32v == 0x11223344);
    assert(xrce_cdr_read_f64(&r, &f64v) && f64v == 1.0);
}

static void case_string_round_trip(void) {
    uint8_t buf[32];
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, sizeof(buf));
    assert(xrce_cdr_write_header(&w));
    assert(xrce_cdr_write_string(&w, "AB"));
    /* len field (3, includes NUL) right after the 4-byte header, already
     * 4-aligned, then 3 payload bytes. */
    assert(w.pos == 4 + 4 + 3);
    assert(buf[4] == 0x03 && buf[5] == 0 && buf[6] == 0 && buf[7] == 0);
    assert(buf[8] == 'A' && buf[9] == 'B' && buf[10] == '\0');

    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, w.pos);
    char out[32];
    assert(xrce_cdr_read_header(&r));
    assert(xrce_cdr_read_string(&r, out, sizeof(out)));
    assert(strcmp(out, "AB") == 0);
}

static void case_write_fails_when_buffer_too_small(void) {
    uint8_t buf[5]; /* room for the header + 1 byte, not a whole i32 */
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, sizeof(buf));
    assert(xrce_cdr_write_header(&w));
    assert(xrce_cdr_write_u8(&w, 1));
    assert(!xrce_cdr_write_i32(&w, 42));
}

static void case_read_fails_on_truncated_buffer(void) {
    uint8_t buf[24];
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, sizeof(buf));
    assert(xrce_cdr_write_header(&w));
    assert(xrce_cdr_write_f64(&w, 3.5));

    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, w.pos - 1); /* one byte short */
    assert(xrce_cdr_read_header(&r));
    double v;
    assert(!xrce_cdr_read_f64(&r, &v));
}

int main(void) {
    run_case("header round trip", case_header_round_trip);
    run_case("header rejects big-endian", case_header_rejects_big_endian);
    run_case("alignment is byte-exact, not just round-trippable", case_alignment_byte_exact);
    run_case("string round trip", case_string_round_trip);
    run_case("write fails when buffer too small", case_write_fails_when_buffer_too_small);
    run_case("read fails on truncated buffer", case_read_fails_on_truncated_buffer);

    printf("PASS: %d test cases\n", g_tests_run);
    return 0;
}
