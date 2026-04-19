#ifndef XRCE_SESSION_H
#define XRCE_SESSION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Minimal Micro XRCE-DDS session/client layer: enough to establish a
 * session with a real, unmodified micro-ros-agent, create a
 * participant/topic/publisher/datawriter via XML representation, and
 * publish samples with WRITE_DATA. Wire-format details (message header
 * layout, submessage IDs, entity representation field order) are
 * ground-truthed against eProsima's reference client source
 * (src/c/core/session/, src/c/core/serialization/xrce_types.c) -- see
 * xrce/docs/design.md for the specific files and the reasoning behind
 * choices that aren't dictated by the spec (XML over REF/BIN
 * representation, single best-effort output stream).
 *
 * Deliberately out of scope for this pass: reliable streams
 * (heartbeat/acknack retransmission) and services (request/reply) -- both
 * real XRCE features, just later phases. Every message here goes out on
 * one best-effort output stream, so delivery isn't guaranteed -- matching
 * the serial transport layer's own "reliability is a higher layer's job"
 * stance (xrce/include/xrce/serial_transport.h).
 *
 * Phase 5 adds the subscription half: READ_DATA (request the agent start
 * delivering a datareader's samples) and parsing the DATA submessages it
 * sends back. Symmetric with WRITE_DATA in an important, easy-to-miss way:
 * just as this project strips its own CDR header before WRITE_DATA (the
 * agent's generic topic type adds its own -- see the Phase 4 section of
 * xrce/docs/design.md), the sample bytes inside a received DATA
 * submessage are ALSO header-less raw field data, not a full CDR blob --
 * ground-truthed against the reference client's read_access.c
 * (read_format_data() hands the callback the buffer right after
 * BaseObjectRequest, with no further unwrapping) and TopicPubSubType's
 * deserialize() (`buffer->assign(payload->data + 4, ...)` -- the agent
 * strips its own header before this project's client ever sees the bytes). */

#define XRCE_OBJK_PARTICIPANT 0x01
#define XRCE_OBJK_TOPIC 0x02
#define XRCE_OBJK_PUBLISHER 0x03
#define XRCE_OBJK_SUBSCRIBER 0x04
#define XRCE_OBJK_DATAWRITER 0x05
#define XRCE_OBJK_DATAREADER 0x06

#define XRCE_STATUS_OK 0x00
#define XRCE_STATUS_OK_MATCHED 0x01

typedef struct {
    uint16_t id;
    uint8_t kind;
} xrce_object_id_t;

xrce_object_id_t xrce_object_id(uint16_t id, uint8_t kind);

typedef struct {
    uint8_t session_id;   /* client-chosen, < 0x80 so the key is always sent */
    uint8_t client_key[4];
    uint16_t mtu;
    uint16_t next_request_id;
    uint16_t out_seq_num; /* single best-effort output stream's counter */
} xrce_session_t;

void xrce_session_init(xrce_session_t *s, uint8_t session_id,
                        const uint8_t client_key[4], uint16_t mtu);

/* Builds the CREATE_CLIENT handshake message (its own special-cased
 * header: session_id 0x00, stream 0, seq_num 0 -- see design notes) into
 * `buf`. Returns bytes written, 0 on overflow. */
size_t xrce_session_build_create_client(const xrce_session_t *s, uint8_t *buf, size_t cap);

/* True if `buf` (a full received message) is a STATUS_AGENT reply
 * reporting success for the CREATE_CLIENT handshake. */
bool xrce_session_parse_create_client_reply(const uint8_t *buf, size_t len);

/* Builds a CREATE message for a PARTICIPANT (the only entity kind whose
 * representation ends in a domain_id rather than a parent ObjectId).
 * `stream_id_raw` is the raw output stream byte (1 = best-effort index 0).
 * Returns bytes written, 0 on overflow. */
size_t xrce_session_build_create_participant(xrce_session_t *s, uint8_t stream_id_raw,
                                              xrce_object_id_t object_id, int16_t domain_id,
                                              const char *xml, uint8_t *buf, size_t cap);

/* Builds a CREATE message for TOPIC/PUBLISHER/DATAWRITER (any entity kind
 * whose representation ends in a parent ObjectId) via XML representation. */
size_t xrce_session_build_create_xml(xrce_session_t *s, uint8_t stream_id_raw,
                                      xrce_object_id_t object_id, xrce_object_id_t parent_id,
                                      const char *xml, uint8_t *buf, size_t cap);

/* True if `buf` is a STATUS reply reporting success for the most recent
 * CREATE request (does not correlate by request_id -- fine for this
 * project's one-request-in-flight-at-a-time usage). */
bool xrce_session_parse_create_reply(const uint8_t *buf, size_t len);

/* Builds a WRITE_DATA message wrapping an already-CDR-serialized sample
 * (e.g. from xrce/include/xrce/msgs.h) verbatim -- WRITE_DATA's payload is
 * the raw sample bytes with no further framing on top. */
size_t xrce_session_build_write_data(xrce_session_t *s, uint8_t stream_id_raw,
                                      xrce_object_id_t datawriter_id,
                                      const uint8_t *sample, size_t sample_len,
                                      uint8_t *buf, size_t cap);

/* Requests the agent start delivering `datareader_id`'s samples as
 * FORMAT_DATA (raw, no SampleInfo wrapper) on `preferred_stream_id_raw`
 * (the raw stream byte the agent should use when sending DATA back to
 * us -- this project always uses the same best-effort stream for both
 * directions, so callers pass the same raw id as `stream_id_raw`). No
 * content filter. Always requests UNLIMITED delivery (max_samples =
 * 0xFFFF) -- NOT the same as "no delivery control specified": the agent
 * defaults an omitted delivery_control to a ONE-SHOT single sample, which
 * looks like it works (the first published value arrives) right up until
 * the second one silently never does. See session.c for how this was
 * actually found. */
size_t xrce_session_build_read_data(xrce_session_t *s, uint8_t stream_id_raw,
                                     xrce_object_id_t datareader_id,
                                     uint8_t preferred_stream_id_raw, uint8_t *buf, size_t cap);

/* Parses a received (deframed) message as a DATA submessage. On success,
 * `*out_object_id` is the datareader the sample came from and
 * `*out_sample`/`*out_sample_len` point at the raw field bytes within
 * `buf` (valid only as long as `buf` is) -- header-less, see the header
 * comment above for why. */
bool xrce_session_parse_data(const uint8_t *buf, size_t len, xrce_object_id_t *out_object_id,
                              const uint8_t **out_sample, size_t *out_sample_len);

#endif
