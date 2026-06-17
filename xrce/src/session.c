#include "xrce/session.h"

#include <string.h>

#include "xrce/cdr.h"

#define XRCE_SUBMSG_CREATE_CLIENT 0
#define XRCE_SUBMSG_CREATE 1
#define XRCE_SUBMSG_DELETE 3
#define XRCE_SUBMSG_STATUS_AGENT 4
#define XRCE_SUBMSG_STATUS 5
#define XRCE_SUBMSG_WRITE_DATA 7
#define XRCE_SUBMSG_READ_DATA 8
#define XRCE_SUBMSG_DATA 9
#define XRCE_FORMAT_DATA 0x00

#define XRCE_FLAG_ENDIANNESS_LE 0x01
#define XRCE_REPRESENTATION_AS_XML_STRING 0x02

#define XRCE_CREATE_CLIENT_PAYLOAD_SIZE 16

xrce_object_id_t xrce_object_id(uint16_t id, uint8_t kind) {
    xrce_object_id_t oid = {.id = id, .kind = kind};
    return oid;
}

static bool write_u16(xrce_cdr_writer_t *w, uint16_t v) {
    /* Bit-pattern reinterpretation, not a value conversion -- LE byte
     * layout is identical whether xrce_cdr_write_i16 treats it as signed
     * or not, and adding a dedicated u16 primitive to cdr.h just to avoid
     * this cast isn't worth the duplication. */
    return xrce_cdr_write_i16(w, (int16_t)v);
}

static bool read_u16(xrce_cdr_reader_t *r, uint16_t *out) {
    int16_t v;
    if (!xrce_cdr_read_i16(r, &v)) {
        return false;
    }
    *out = (uint16_t)v;
    return true;
}

/* ObjectId wire packing is NOT a CDR primitive -- it's the XRCE spec's own
 * bit-packing (12-bit id, 4-bit kind) into 2 raw bytes, ground-truthed
 * against object_id.c's uxr_object_id_to_raw(). */
static bool write_object_id(xrce_cdr_writer_t *w, xrce_object_id_t oid) {
    uint8_t raw[2];
    raw[0] = (uint8_t)(oid.id >> 4);
    raw[1] = (uint8_t)((uint8_t)(oid.id << 4) | (oid.kind & 0x0F));
    return xrce_cdr_write_bytes(w, raw, sizeof(raw));
}

static bool read_object_id(xrce_cdr_reader_t *r, xrce_object_id_t *oid) {
    uint8_t raw[2];
    if (!xrce_cdr_read_bytes(r, raw, sizeof(raw))) {
        return false;
    }
    oid->id = (uint16_t)(((uint16_t)raw[0] << 4) | (raw[1] >> 4));
    oid->kind = raw[1] & 0x0F;
    return true;
}

/* RequestId is big-endian (unlike everything else here, which is LE) --
 * matches uxr_init_base_object_request() writing request_id.data[0] as
 * the high byte. */
static bool write_base_object_request(xrce_cdr_writer_t *w, uint16_t request_id,
                                       xrce_object_id_t object_id) {
    uint8_t req[2] = {(uint8_t)(request_id >> 8), (uint8_t)request_id};
    return xrce_cdr_write_bytes(w, req, sizeof(req)) && write_object_id(w, object_id);
}

/* Message header: session_id(1) + stream_id(1) + seq_num(2, LE) +
 * [client_key(4) iff session_id < 0x80]. Every session_id this project
 * ever uses is < 0x80 (see xrce_session_init), so the key is always
 * present -- meaning this header is always exactly 8 bytes, always a
 * multiple of 4, which is why nothing here needs an explicit align-to-4
 * before the submessage header that follows (see submessage_header_at
 * below). */
static bool write_message_header(xrce_cdr_writer_t *w, uint8_t session_id, uint8_t stream_id,
                                  uint16_t seq_num, const uint8_t key[4]) {
    return xrce_cdr_write_u8(w, session_id) && xrce_cdr_write_u8(w, stream_id) &&
           write_u16(w, seq_num) && xrce_cdr_write_bytes(w, key, 4);
}

/* Writes a submessage header with a zero length placeholder and returns
 * its buffer offset so the caller can backpatch the real length once the
 * payload's been written -- simpler and less error-prone than
 * precomputing payload sizes by hand (which is what the reference client
 * does, purely as a buffer-sizing optimization, not because it's required
 * for correctness). */
static bool submessage_header_at(xrce_cdr_writer_t *w, uint8_t id, uint8_t flags,
                                  size_t *header_pos) {
    *header_pos = w->pos;
    return xrce_cdr_write_u8(w, id) && xrce_cdr_write_u8(w, flags) && write_u16(w, 0);
}

static void backpatch_length(uint8_t *buf, size_t header_pos, size_t payload_len) {
    buf[header_pos + 2] = (uint8_t)(payload_len & 0xFF);
    buf[header_pos + 3] = (uint8_t)((payload_len >> 8) & 0xFF);
}

void xrce_session_init(xrce_session_t *s, uint8_t session_id, const uint8_t client_key[4],
                        uint16_t mtu) {
    s->session_id = session_id;
    memcpy(s->client_key, client_key, 4);
    s->mtu = mtu;
    s->next_request_id = 9; /* matches RESERVED_REQUESTS_ID in the reference client */
    s->out_seq_num = 0;
}

size_t xrce_session_build_create_client(const xrce_session_t *s, uint8_t *buf, size_t cap) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);

    /* The handshake message header is special-cased: session_id is always
     * 0x00 here (not s->session_id), matching
     * uxr_stamp_create_session_header()'s `info->id & SESSION_ID_WITHOUT_CLIENT_KEY`
     * -- the target session doesn't exist yet, so this always goes out on
     * the well-known "unestablished" session/stream/seq (0/0/0). The
     * *desired* session id is carried inside the payload instead. */
    if (!write_message_header(&w, 0x00, 0x00, 0x00, s->client_key)) {
        return 0;
    }

    size_t header_pos;
    if (!submessage_header_at(&w, XRCE_SUBMSG_CREATE_CLIENT, XRCE_FLAG_ENDIANNESS_LE, &header_pos)) {
        return 0;
    }

    size_t payload_start = w.pos;
    static const uint8_t cookie[4] = {'X', 'R', 'C', 'E'};
    static const uint8_t version[2] = {0x01, 0x00};
    static const uint8_t vendor_id[2] = {0x01, 0x0F};
    if (!xrce_cdr_write_bytes(&w, cookie, sizeof(cookie)) ||
        !xrce_cdr_write_bytes(&w, version, sizeof(version)) ||
        !xrce_cdr_write_bytes(&w, vendor_id, sizeof(vendor_id)) ||
        !xrce_cdr_write_bytes(&w, s->client_key, 4) ||
        !xrce_cdr_write_u8(&w, s->session_id) ||
        !xrce_cdr_write_bool(&w, false) /* optional_properties */ ||
        !write_u16(&w, s->mtu)) {
        return 0;
    }

    backpatch_length(buf, header_pos, w.pos - payload_start);
    return w.pos;
}

static bool skip_message_header(xrce_cdr_reader_t *r) {
    if (r->len < 2) {
        return false;
    }
    uint8_t session_id = r->buf[0];
    size_t header_len = (session_id < 0x80) ? 8 : 4;
    if (r->len < header_len) {
        return false;
    }
    r->pos = header_len;
    return true;
}

bool xrce_session_parse_create_client_reply(const uint8_t *buf, size_t len) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    if (!skip_message_header(&r)) {
        return false;
    }
    uint8_t id, flags;
    uint16_t sub_len;
    if (!xrce_cdr_read_u8(&r, &id) || !xrce_cdr_read_u8(&r, &flags) || !read_u16(&r, &sub_len)) {
        return false;
    }
    (void)flags;
    (void)sub_len;
    if (id != XRCE_SUBMSG_STATUS_AGENT) {
        return false;
    }
    uint8_t status;
    if (!xrce_cdr_read_u8(&r, &status)) {
        return false;
    }
    return status == XRCE_STATUS_OK || status == XRCE_STATUS_OK_MATCHED;
}

static size_t build_create(xrce_session_t *s, uint8_t stream_id_raw, xrce_object_id_t object_id,
                            const char *xml, bool is_participant, int16_t domain_id,
                            xrce_object_id_t parent_id, uint8_t *buf, size_t cap) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);

    if (!write_message_header(&w, s->session_id, stream_id_raw, s->out_seq_num, s->client_key)) {
        return 0;
    }
    s->out_seq_num++;

    size_t header_pos;
    if (!submessage_header_at(&w, XRCE_SUBMSG_CREATE, XRCE_FLAG_ENDIANNESS_LE, &header_pos)) {
        return 0;
    }

    size_t payload_start = w.pos;
    uint16_t request_id = s->next_request_id++;
    if (!write_base_object_request(&w, request_id, object_id) ||
        !xrce_cdr_write_u8(&w, object_id.kind) ||
        !xrce_cdr_write_u8(&w, XRCE_REPRESENTATION_AS_XML_STRING) ||
        !xrce_cdr_write_string(&w, xml)) {
        return 0;
    }
    bool tail_ok = is_participant ? xrce_cdr_write_i16(&w, domain_id) : write_object_id(&w, parent_id);
    if (!tail_ok) {
        return 0;
    }

    backpatch_length(buf, header_pos, w.pos - payload_start);
    return w.pos;
}

size_t xrce_session_build_create_participant(xrce_session_t *s, uint8_t stream_id_raw,
                                              xrce_object_id_t object_id, int16_t domain_id,
                                              const char *xml, uint8_t *buf, size_t cap) {
    xrce_object_id_t unused_parent = {0};
    return build_create(s, stream_id_raw, object_id, xml, true, domain_id, unused_parent, buf, cap);
}

size_t xrce_session_build_create_xml(xrce_session_t *s, uint8_t stream_id_raw,
                                      xrce_object_id_t object_id, xrce_object_id_t parent_id,
                                      const char *xml, uint8_t *buf, size_t cap) {
    return build_create(s, stream_id_raw, object_id, xml, false, 0, parent_id, buf, cap);
}

/* Shared by CREATE and DELETE replies -- both are just a STATUS submessage
 * (related_request + a status byte), the only difference being which request
 * they're replying to, which this project doesn't correlate by request_id
 * (see the header comment on xrce_session_parse_create_reply). */
static bool parse_status_reply(const uint8_t *buf, size_t len) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    if (!skip_message_header(&r)) {
        return false;
    }
    uint8_t id, flags;
    uint16_t sub_len;
    if (!xrce_cdr_read_u8(&r, &id) || !xrce_cdr_read_u8(&r, &flags) || !read_u16(&r, &sub_len)) {
        return false;
    }
    (void)flags;
    (void)sub_len;
    if (id != XRCE_SUBMSG_STATUS) {
        return false;
    }
    uint8_t related_request[4];
    uint8_t status;
    if (!xrce_cdr_read_bytes(&r, related_request, sizeof(related_request)) ||
        !xrce_cdr_read_u8(&r, &status)) {
        return false;
    }
    (void)related_request;
    return status == XRCE_STATUS_OK || status == XRCE_STATUS_OK_MATCHED;
}

bool xrce_session_parse_create_reply(const uint8_t *buf, size_t len) {
    return parse_status_reply(buf, len);
}

size_t xrce_session_build_write_data(xrce_session_t *s, uint8_t stream_id_raw,
                                      xrce_object_id_t datawriter_id, const uint8_t *sample,
                                      size_t sample_len, uint8_t *buf, size_t cap) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);

    if (!write_message_header(&w, s->session_id, stream_id_raw, s->out_seq_num, s->client_key)) {
        return 0;
    }
    s->out_seq_num++;

    size_t header_pos;
    if (!submessage_header_at(&w, XRCE_SUBMSG_WRITE_DATA, XRCE_FLAG_ENDIANNESS_LE, &header_pos)) {
        return 0;
    }

    size_t payload_start = w.pos;
    uint16_t request_id = s->next_request_id++;
    if (!write_base_object_request(&w, request_id, datawriter_id) ||
        !xrce_cdr_write_bytes(&w, sample, sample_len)) {
        return 0;
    }

    backpatch_length(buf, header_pos, w.pos - payload_start);
    return w.pos;
}

size_t xrce_session_build_read_data(xrce_session_t *s, uint8_t stream_id_raw,
                                     xrce_object_id_t datareader_id,
                                     uint8_t preferred_stream_id_raw, uint8_t *buf, size_t cap) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);

    if (!write_message_header(&w, s->session_id, stream_id_raw, s->out_seq_num, s->client_key)) {
        return 0;
    }
    s->out_seq_num++;

    size_t header_pos;
    if (!submessage_header_at(&w, XRCE_SUBMSG_READ_DATA, XRCE_FLAG_ENDIANNESS_LE, &header_pos)) {
        return 0;
    }

    size_t payload_start = w.pos;
    uint16_t request_id = s->next_request_id++;
    /* optional_delivery_control MUST be true with max_samples set to
     * "unlimited" (0xFFFF, matching UXR_MAX_SAMPLES_UNLIMITED in the
     * reference client) for a real, continuously-running subscription.
     * Omitting delivery_control (what an earlier version of this function
     * did) isn't just "no limit specified" -- the agent's own
     * DataReader::read() defaults it to max_samples=1, a ONE-SHOT read.
     * Confirmed by testing against a real agent: the first published
     * value after CREATE arrived, but nothing after -- rediscovered by
     * reading DataReader.cpp's `else { ... max_samples(1); }` branch,
     * and confirmed against the reference client's own
     * examples/SubscribeHelloWorld/main.c, which always sets this
     * explicitly for exactly this reason. */
    if (!write_base_object_request(&w, request_id, datareader_id) ||
        !xrce_cdr_write_u8(&w, preferred_stream_id_raw) ||
        !xrce_cdr_write_u8(&w, XRCE_FORMAT_DATA) ||
        !xrce_cdr_write_bool(&w, false) /* optional_content_filter_expression */ ||
        !xrce_cdr_write_bool(&w, true) /* optional_delivery_control */ ||
        !write_u16(&w, 0xFFFF) /* max_samples: unlimited */ ||
        !write_u16(&w, 0) /* max_elapsed_time: unlimited */ ||
        !write_u16(&w, 0) /* max_bytes_per_seconds: unlimited */ ||
        !write_u16(&w, 0) /* min_pace_period: none */) {
        return 0;
    }

    backpatch_length(buf, header_pos, w.pos - payload_start);
    return w.pos;
}

bool xrce_session_parse_data(const uint8_t *buf, size_t len, xrce_object_id_t *out_object_id,
                              const uint8_t **out_sample, size_t *out_sample_len) {
    xrce_cdr_reader_t r;
    xrce_cdr_reader_init(&r, buf, len);
    if (!skip_message_header(&r)) {
        return false;
    }
    uint8_t id, flags;
    uint16_t sub_len;
    if (!xrce_cdr_read_u8(&r, &id) || !xrce_cdr_read_u8(&r, &flags) || !read_u16(&r, &sub_len)) {
        return false;
    }
    (void)flags;
    (void)sub_len;
    if (id != XRCE_SUBMSG_DATA) {
        return false;
    }

    uint8_t request_id_raw[2];
    if (!xrce_cdr_read_bytes(&r, request_id_raw, sizeof(request_id_raw)) ||
        !read_object_id(&r, out_object_id)) {
        return false;
    }
    (void)request_id_raw;

    *out_sample = &buf[r.pos];
    *out_sample_len = r.len - r.pos;
    return true;
}

size_t xrce_session_build_delete(xrce_session_t *s, uint8_t stream_id_raw,
                                  xrce_object_id_t object_id, uint8_t *buf, size_t cap) {
    xrce_cdr_writer_t w;
    xrce_cdr_writer_init(&w, buf, cap);

    if (!write_message_header(&w, s->session_id, stream_id_raw, s->out_seq_num, s->client_key)) {
        return 0;
    }
    s->out_seq_num++;

    size_t header_pos;
    if (!submessage_header_at(&w, XRCE_SUBMSG_DELETE, XRCE_FLAG_ENDIANNESS_LE, &header_pos)) {
        return 0;
    }

    size_t payload_start = w.pos;
    uint16_t request_id = s->next_request_id++;
    if (!write_base_object_request(&w, request_id, object_id)) {
        return 0;
    }

    backpatch_length(buf, header_pos, w.pos - payload_start);
    return w.pos;
}

bool xrce_session_parse_delete_reply(const uint8_t *buf, size_t len) {
    return parse_status_reply(buf, len);
}
