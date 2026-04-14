/* Phase 3 milestone (ROS2 layer): session/entity-creation/write-data
 * message construction. case_create_client_byte_exact hand-computes the
 * full expected wire bytes for CREATE_CLIENT (the one message small and
 * fixed-size enough to do that for); the rest decode their own output
 * field-by-field, which is what actually exercises the generic
 * alignment-aware writer against variable-length XML strings instead of
 * just re-checking one fixed case. */

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

static void case_create_client_byte_exact(void) {
    uint8_t key[4] = {0x11, 0x22, 0x33, 0x44};
    xrce_session_t s;
    xrce_session_init(&s, 0x01, key, 512);

    uint8_t buf[64];
    size_t len = xrce_session_build_create_client(&s, buf, sizeof(buf));

    static const uint8_t expected[28] = {
        0x00, 0x00, 0x00, 0x00,             /* header: session 0, stream 0, seq 0 */
        0x11, 0x22, 0x33, 0x44,             /* client key */
        0x00, 0x01, 0x10, 0x00,             /* subheader: CREATE_CLIENT, LE flag, len=16 */
        'X', 'R', 'C', 'E',                 /* xrce_cookie */
        0x01, 0x00,                         /* xrce_version 1.0 */
        0x01, 0x0F,                         /* xrce_vendor_id (eProsima's) */
        0x11, 0x22, 0x33, 0x44,             /* client_key (again, inside payload) */
        0x01,                               /* session_id (the *desired* one) */
        0x00,                               /* optional_properties = false */
        0x00, 0x02,                         /* mtu = 512, LE */
    };
    assert(len == sizeof(expected));
    assert(memcmp(buf, expected, sizeof(expected)) == 0);
}

static void case_create_client_reply_parsing(void) {
    /* header (session 0x01 < 0x80, so 8 bytes incl. key) + STATUS_AGENT
     * subheader + a single status byte = OK. */
    uint8_t ok_reply[] = {
        0x01, 0x00, 0x00, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, /* header */
        0x04, 0x01, 0x01, 0x00,                         /* STATUS_AGENT, LE, len=1 */
        0x00,                                           /* ResultStatus.status = OK */
    };
    assert(xrce_session_parse_create_client_reply(ok_reply, sizeof(ok_reply)));

    uint8_t err_reply[] = {
        0x01, 0x00, 0x00, 0x00, 0xAA, 0xBB, 0xCC, 0xDD,
        0x04, 0x01, 0x01, 0x00,
        0x82, /* UXR_STATUS_ERR_ALREADY_EXISTS */
    };
    assert(!xrce_session_parse_create_client_reply(err_reply, sizeof(err_reply)));
}

static void case_create_topic_round_trip(void) {
    uint8_t key[4] = {1, 2, 3, 4};
    xrce_session_t s;
    xrce_session_init(&s, 0x02, key, 512);

    xrce_object_id_t topic_id = xrce_object_id(0x123, XRCE_OBJK_TOPIC);
    xrce_object_id_t participant_id = xrce_object_id(0x001, XRCE_OBJK_PARTICIPANT);
    const char *xml =
        "<dds_topic name=\"rt/chatter\" dataType=\"std_msgs::msg::dds_::Int32_\"/>";

    uint8_t buf[256];
    size_t len = xrce_session_build_create_xml(&s, 1, topic_id, participant_id, xml, buf, sizeof(buf));
    assert(len > 0);

    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);

    uint8_t session_id, stream_id;
    uint16_t seq_num;
    uint8_t key_out[4];
    assert(xrce_cdr_read_u8(&r, &session_id) && session_id == 0x02);
    assert(xrce_cdr_read_u8(&r, &stream_id) && stream_id == 1);
    int16_t seq_raw;
    assert(xrce_cdr_read_i16(&r, &seq_raw));
    seq_num = (uint16_t)seq_raw;
    assert(seq_num == 0); /* first message on this stream */
    assert(xrce_cdr_read_bytes(&r, key_out, 4) && memcmp(key_out, key, 4) == 0);

    uint8_t sub_id, sub_flags;
    int16_t sub_len_raw;
    assert(xrce_cdr_read_u8(&r, &sub_id) && sub_id == 1 /* SUBMESSAGE_ID_CREATE */);
    assert(xrce_cdr_read_u8(&r, &sub_flags) && (sub_flags & 0x01) == 0x01 /* LE */);
    assert(xrce_cdr_read_i16(&r, &sub_len_raw));
    uint16_t sub_len = (uint16_t)sub_len_raw;
    assert(sub_len == len - r.pos);

    uint8_t req_id_raw[2], obj_id_raw[2];
    assert(xrce_cdr_read_bytes(&r, req_id_raw, 2));
    assert(xrce_cdr_read_bytes(&r, obj_id_raw, 2));
    /* ObjectId wire packing: raw[0] = id>>4, raw[1] = (id<<4)|kind. */
    uint16_t decoded_topic_id = (uint16_t)(((uint16_t)obj_id_raw[0] << 4) | (obj_id_raw[1] >> 4));
    uint8_t decoded_kind = obj_id_raw[1] & 0x0F;
    assert(decoded_topic_id == topic_id.id);
    assert(decoded_kind == topic_id.kind);

    uint8_t kind_field, format_field;
    assert(xrce_cdr_read_u8(&r, &kind_field) && kind_field == XRCE_OBJK_TOPIC);
    assert(xrce_cdr_read_u8(&r, &format_field) && format_field == 0x02 /* AS_XML_STRING */);
    char xml_out[128];
    assert(xrce_cdr_read_string(&r, xml_out, sizeof(xml_out)));
    assert(strcmp(xml_out, xml) == 0);

    uint8_t parent_raw[2];
    assert(xrce_cdr_read_bytes(&r, parent_raw, 2));
    uint16_t decoded_parent_id = (uint16_t)(((uint16_t)parent_raw[0] << 4) | (parent_raw[1] >> 4));
    uint8_t decoded_parent_kind = parent_raw[1] & 0x0F;
    assert(decoded_parent_id == participant_id.id);
    assert(decoded_parent_kind == participant_id.kind);

    assert(r.pos == len); /* nothing left over, nothing short */
}

/* Odd-length XML forces the domain_id field's 2-byte alignment to
 * actually insert a padding byte -- proves the generic aligned writer
 * handles this dynamically rather than something that only happens to
 * work for round xml lengths. */
static void case_create_participant_with_odd_length_xml(void) {
    uint8_t key[4] = {9, 9, 9, 9};
    xrce_session_t s;
    xrce_session_init(&s, 0x03, key, 512);

    xrce_object_id_t participant_id = xrce_object_id(0x001, XRCE_OBJK_PARTICIPANT);
    /* 18 chars -> CDR string length field (17 chars would also do it, but
     * being explicit about *why* this string was chosen matters more than
     * the string's content) is 19 (includes the NUL), landing domain_id
     * on an odd byte offset unless the writer pads it -- exercises the
     * dynamic alignment path rather than a case that happens to need zero
     * padding bytes. */
    const char *xml = "<dds_participant/>";

    uint8_t buf[256];
    size_t len = xrce_session_build_create_participant(&s, 1, participant_id, 42, xml, buf, sizeof(buf));
    assert(len > 0);

    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    r.pos = 8; /* skip message header (session_id 0x03 < 0x80, so 8 bytes) */

    uint8_t sub_id, sub_flags;
    int16_t sub_len_raw;
    assert(xrce_cdr_read_u8(&r, &sub_id) && sub_id == 1);
    assert(xrce_cdr_read_u8(&r, &sub_flags));
    assert(xrce_cdr_read_i16(&r, &sub_len_raw));

    uint8_t req_id_raw[2], obj_id_raw[2], kind_field, format_field;
    assert(xrce_cdr_read_bytes(&r, req_id_raw, 2));
    assert(xrce_cdr_read_bytes(&r, obj_id_raw, 2));
    assert(xrce_cdr_read_u8(&r, &kind_field) && kind_field == XRCE_OBJK_PARTICIPANT);
    assert(xrce_cdr_read_u8(&r, &format_field));
    char xml_out[64];
    assert(xrce_cdr_read_string(&r, xml_out, sizeof(xml_out)));
    assert(strcmp(xml_out, xml) == 0);

    int16_t domain_id;
    assert(xrce_cdr_read_i16(&r, &domain_id)); /* would fail/misread without correct padding */
    assert(domain_id == 42);
    assert(r.pos == len);
}

static void case_write_data_wraps_sample_verbatim(void) {
    uint8_t key[4] = {5, 5, 5, 5};
    xrce_session_t s;
    xrce_session_init(&s, 0x04, key, 512);

    std_msgs_Int32 msg = {.data = 777};
    uint8_t sample[16];
    size_t sample_len;
    assert(std_msgs_Int32_encode(&msg, sample, sizeof(sample), &sample_len));

    xrce_object_id_t datawriter_id = xrce_object_id(0x55, XRCE_OBJK_DATAWRITER);
    uint8_t buf[128];
    size_t len = xrce_session_build_write_data(&s, 1, datawriter_id, sample, sample_len, buf, sizeof(buf));
    assert(len > 0);
    assert(len == 8 /* header */ + 4 /* subheader */ + 4 /* base object request */ + sample_len);

    /* The trailing bytes must be the sample, byte-for-byte, and still
     * decode correctly through the ordinary message-layer decoder. */
    const uint8_t *trailing = buf + (len - sample_len);
    assert(memcmp(trailing, sample, sample_len) == 0);

    std_msgs_Int32 decoded;
    assert(std_msgs_Int32_decode(trailing, sample_len, &decoded));
    assert(decoded.data == 777);
}

static void case_create_reply_parsing(void) {
    uint8_t ok_reply[] = {
        0x02, 0x00, 0x00, 0x00, 0, 0, 0, 0,       /* header */
        0x05, 0x01, 0x06, 0x00,                   /* STATUS, LE, len=6 */
        0x00, 0x09, 0x01, 0x20,                   /* related_request (request_id+object_id, ignored) */
        0x00, 0x00,                               /* ResultStatus: OK, impl_status 0 */
    };
    assert(xrce_session_parse_create_reply(ok_reply, sizeof(ok_reply)));
}

int main(void) {
    run_case("CREATE_CLIENT is byte-exact", case_create_client_byte_exact);
    run_case("CREATE_CLIENT reply parsing (OK and error)", case_create_client_reply_parsing);
    run_case("CREATE topic round trip (decode every field)", case_create_topic_round_trip);
    run_case("CREATE participant, odd-length XML exercises alignment padding",
              case_create_participant_with_odd_length_xml);
    run_case("WRITE_DATA wraps an already-CDR-serialized sample verbatim",
              case_write_data_wraps_sample_verbatim);
    run_case("CREATE reply parsing", case_create_reply_parsing);

    printf("PASS: %d test cases\n", g_tests_run);
    return 0;
}
