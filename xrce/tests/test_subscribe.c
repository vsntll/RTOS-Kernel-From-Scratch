/* Phase 5 milestone (ROS2 layer): subscription half of the protocol --
 * READ_DATA (ask the agent to start delivering a datareader's samples)
 * and parsing the DATA submessages it sends back. Symmetric with Phase 3's
 * WRITE_DATA tests: decode every field of what we build, and hand-build a
 * plausible incoming DATA message to prove parsing works without needing
 * a live agent for this level of test (the live counterpart is
 * host/live_subscribe_demo.c). */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../include/xrce/msgs.h"
#include "../include/xrce/session.h"

typedef void (*test_case_fn)(void);
static int g_tests_run;

static void run_case(const char *name, test_case_fn fn) {
    printf("[case] %s\n", name);
    fn();
    g_tests_run++;
}

static void case_read_data_round_trip(void) {
    uint8_t key[4] = {7, 7, 7, 7};
    xrce_session_t s;
    xrce_session_init(&s, 0x05, key, 512);

    xrce_object_id_t datareader_id = xrce_object_id(0x001, XRCE_OBJK_DATAREADER);
    uint8_t buf[64];
    size_t len = xrce_session_build_read_data(&s, 1, datareader_id, 1, buf, sizeof(buf));
    assert(len > 0);

    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);

    uint8_t session_id, stream_id;
    assert(xrce_cdr_read_u8(&r, &session_id) && session_id == 0x05);
    assert(xrce_cdr_read_u8(&r, &stream_id) && stream_id == 1);
    int16_t seq_raw;
    assert(xrce_cdr_read_i16(&r, &seq_raw));
    uint8_t key_out[4];
    assert(xrce_cdr_read_bytes(&r, key_out, 4) && memcmp(key_out, key, 4) == 0);

    uint8_t sub_id, sub_flags;
    int16_t sub_len_raw;
    assert(xrce_cdr_read_u8(&r, &sub_id) && sub_id == 8 /* SUBMESSAGE_ID_READ_DATA */);
    assert(xrce_cdr_read_u8(&r, &sub_flags) && (sub_flags & 0x01) == 0x01);
    assert(xrce_cdr_read_i16(&r, &sub_len_raw));
    uint16_t sub_len = (uint16_t)sub_len_raw;
    assert(sub_len == len - r.pos);

    uint8_t req_id_raw[2], obj_id_raw[2];
    assert(xrce_cdr_read_bytes(&r, req_id_raw, 2));
    assert(xrce_cdr_read_bytes(&r, obj_id_raw, 2));
    uint16_t decoded_id = (uint16_t)(((uint16_t)obj_id_raw[0] << 4) | (obj_id_raw[1] >> 4));
    uint8_t decoded_kind = obj_id_raw[1] & 0x0F;
    assert(decoded_id == datareader_id.id);
    assert(decoded_kind == datareader_id.kind);

    uint8_t preferred_stream, data_format;
    bool optional_filter, optional_delivery;
    assert(xrce_cdr_read_u8(&r, &preferred_stream) && preferred_stream == 1);
    assert(xrce_cdr_read_u8(&r, &data_format) && data_format == 0x00 /* FORMAT_DATA */);
    assert(xrce_cdr_read_bool(&r, &optional_filter) && !optional_filter);
    /* Always true, with max_samples = unlimited -- omitting delivery_control
     * entirely defaults the agent to a ONE-SHOT single-sample read, which
     * is not what a real subscription wants. See session.c/session.h for
     * how this was found. */
    assert(xrce_cdr_read_bool(&r, &optional_delivery) && optional_delivery);
    int16_t max_samples_raw, max_elapsed_raw, max_bytes_raw, min_pace_raw;
    assert(xrce_cdr_read_i16(&r, &max_samples_raw) && (uint16_t)max_samples_raw == 0xFFFF);
    assert(xrce_cdr_read_i16(&r, &max_elapsed_raw) && max_elapsed_raw == 0);
    assert(xrce_cdr_read_i16(&r, &max_bytes_raw) && max_bytes_raw == 0);
    assert(xrce_cdr_read_i16(&r, &min_pace_raw) && min_pace_raw == 0);
    assert(r.pos == len);
}

/* Hand-builds a plausible incoming message -- header + DATA submessage +
 * BaseObjectRequest + raw (header-less) sample bytes -- the way the real
 * agent would send one, and confirms xrce_session_parse_data() extracts
 * the datareader object id and hands back exactly the sample bytes. */
static void case_parse_data_extracts_sample(void) {
    uint8_t msg[] = {
        0x05, 0x01, 0x00, 0x00, 7, 7, 7, 7,       /* header: session 5, stream 1, seq 0 */
        0x09, 0x01, 0x08, 0x00,                   /* DATA, LE, len=8 */
        0x00, 0x0A,                                /* request_id (ignored) */
        0x00, 0x16,                                /* object_id: raw[0]=0x00,raw[1]=0x16 -> id=(0<<4)|(0x16>>4)=1, kind=0x16&0xF=6 (DATAREADER) */
        0x2A, 0x00, 0x00, 0x00,                    /* raw sample: int32 LE = 42, no CDR header */
    };

    xrce_object_id_t object_id;
    const uint8_t *sample;
    size_t sample_len;
    assert(xrce_session_parse_data(msg, sizeof(msg), &object_id, &sample, &sample_len));
    assert(object_id.id == 1);
    assert(object_id.kind == XRCE_OBJK_DATAREADER);
    assert(sample_len == 4);

    /* Explicit little-endian assembly rather than memcpy+assume-host-endianness
     * -- same reasoning as xrce_cdr.c's own primitives (CDR_LE is a wire
     * contract, not a host-endianness accident). */
    int32_t value = (int32_t)((uint32_t)sample[0] | ((uint32_t)sample[1] << 8) |
                               ((uint32_t)sample[2] << 16) | ((uint32_t)sample[3] << 24));
    assert(value == 42);
}

static void case_parse_data_rejects_wrong_submessage(void) {
    uint8_t msg[] = {
        0x05, 0x01, 0x00, 0x00, 7, 7, 7, 7,
        0x07, 0x01, 0x08, 0x00, /* WRITE_DATA (7), not DATA (9) */
        0x00, 0x0A, 0x00, 0x16,
        0x2A, 0x00, 0x00, 0x00,
    };
    xrce_object_id_t object_id;
    const uint8_t *sample;
    size_t sample_len;
    assert(!xrce_session_parse_data(msg, sizeof(msg), &object_id, &sample, &sample_len));
}

int main(void) {
    run_case("READ_DATA round trip (decode every field)", case_read_data_round_trip);
    run_case("parse_data extracts header-less sample bytes", case_parse_data_extracts_sample);
    run_case("parse_data rejects a non-DATA submessage", case_parse_data_rejects_wrong_submessage);

    printf("PASS: %d test cases\n", g_tests_run);
    return 0;
}
