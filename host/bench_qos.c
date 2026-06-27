/* Phase 7c deliverable: publish the same sequence of samples on a
 * best-effort topic and a reliable topic, both through a lossy link
 * (host/udp_loss_proxy.py sits between this program and the real agent),
 * and show the reliable one actually recovers from induced loss while the
 * best-effort one just drops data -- with the real retry/latency cost
 * printed, not asserted.
 *
 * Entity setup (CREATE_CLIENT/CREATE) talks DIRECTLY to the real agent,
 * not through the lossy proxy -- found the hard way: the agent recognizes
 * a resent CREATE as a duplicate of one it already processed and simply
 * doesn't reply to it again, so a blind client-side resend can never
 * recover a *reply* that got dropped (only the request side benefits from
 * resending). That's a real, useful thing to know, not just a workaround:
 * it's exactly why XRCE has real sequence-number-aware reliable streams
 * instead of "just resend it" -- which is precisely the mechanism this
 * demo exists to show working. Only the *data* path (WRITE_DATA/HEARTBEAT/
 * ACKNACK) -- the thing actually under test -- goes through the lossy
 * proxy, matching a real reliable-vs-best-effort QoS comparison rather
 * than accidentally testing "can setup survive loss" instead.
 *
 * Two independent xrce_session_t's (separate client keys/sockets) --
 * xrce_session_t tracks a single shared out_seq_num across whatever
 * streams it uses, so mixing a best-effort and a reliable stream on ONE
 * session would make the reliable stream's sequence numbers
 * non-contiguous from its own perspective (gaps caused by the other
 * stream's sends, not real loss). Two sessions sidesteps that cleanly
 * instead of teaching session.c per-stream counters just for this demo.
 *
 * Usage:
 *   MicroXRCEAgent udp4 -p 8888 &
 *   python3 host/udp_loss_proxy.py 8898 127.0.0.1 8888 35 &   # best-effort path, 35% loss
 *   python3 host/udp_loss_proxy.py 8899 127.0.0.1 8888 35 &   # reliable path, 35% loss
 *   gcc -Ixrce/include host/bench_qos.c xrce/build/libxrce.a -o /tmp/bench_qos
 *   /tmp/bench_qos 127.0.0.1 8898 127.0.0.1 8899 20
 *   # elsewhere (start these BEFORE running the bench, and pass the type
 *   # explicitly -- without it, `ros2 topic echo` resolves the type via
 *   # graph discovery once at startup and gives up for good if the topic
 *   # doesn't exist yet at that exact moment, which it won't on a fresh
 *   # agent):
 *   timeout 15 ros2 topic echo /qos_best_effort std_msgs/msg/Int32 | grep -c data:
 *   timeout 15 ros2 topic echo /qos_reliable std_msgs/msg/Int32 | grep -c data:
 */
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "xrce/msgs.h"
#include "xrce/reliable_stream.h"
#include "xrce/session.h"

#define BEST_EFFORT_STREAM_RAW 1

static uint64_t now_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
}

static int make_socket(const char *ip, uint16_t port, struct sockaddr_in *out_addr) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(out_addr, 0, sizeof(*out_addr));
    out_addr->sin_family = AF_INET;
    out_addr->sin_port = htons(port);
    inet_pton(AF_INET, ip, &out_addr->sin_addr);
    struct timeval tv = {.tv_sec = 0, .tv_usec = 100 * 1000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return fd;
}

/* Setup traffic (CREATE_CLIENT/CREATE) goes through the SAME lossy proxy
 * as the data path -- not a direct connection to the agent -- and that's
 * required, not just simpler: eProsima's agent correlates an established
 * session with the UDP source *address* subsequent messages arrive from.
 * An earlier version of this demo sent setup directly to the agent (its
 * own address/port) and only routed data through the proxy; the proxy
 * relays through one fixed local socket, so from the agent's point of
 * view, WRITE_DATA/HEARTBEAT then arrived from a completely different
 * source address than the one the session was established from --
 * confirmed by adding temporary per-packet logging to
 * host/udp_loss_proxy.py, which showed it forwarding every non-dropped
 * packet correctly, while the agent's own log never recorded processing
 * any of them at all (no `[** <<DDS>> **]`, nothing) even though CREATE
 * had worked moments earlier over the direct connection. Routing
 * everything through one proxy keeps the source address the agent sees
 * consistent for the whole session, which fixed it.
 *
 * A second real bug, worth recording separately: resending the exact same
 * bytes doesn't help either, because it isn't just a network-level
 * duplicate -- it carries the same message-header sequence number as the
 * original, and the agent's best-effort input stream tracks the last
 * sequence number it has already processed per stream and silently drops
 * anything at or before that watermark as a replay, without generating a
 * fresh reply (this only came to light once the address-consistency bug
 * above was fixed and CREATE participant *still* got zero replies across
 * 30 identical resends -- statistically impossible from real per-packet
 * loss alone, and confirmed by seeing exactly one `participant created`
 * in the agent's log no matter how many times the identical bytes were
 * resent). CREATE_CLIENT is unaffected because it always targets the
 * special unestablished-session header (session/stream/seq forced to 0),
 * which isn't subject to this replay tracking at all. Fixed by rebuilding
 * the message fresh (via a `build` callback) on every retry, so each
 * attempt gets a new sequence number and looks like new traffic to the
 * agent.
 *
 * A third real bug, downstream of the second fix: once retries got a
 * reply every time, CREATE calls still failed -- with a genuine, non-OK
 * STATUS, not a timeout. The reply's own status byte turned out to be
 * `0x82` (UXR_STATUS_ERR_ALREADY_EXISTS, per test_session.c's own comment
 * on the same value), not OK_MATCHED as this project's earlier design.md
 * notes ("duplicate CREATEs for already-existing entities... log already
 * exists and are otherwise harmless") assumed -- confirmed via temporary
 * per-attempt status logging, catching it on a publisher CREATE
 * specifically (empty-XML representation, so there's nothing for the
 * agent to compare for an exact-match check, apparently tipping it toward
 * ALREADY_EXISTS instead of OK_MATCHED). Either way, for a *retry* --
 * where the goal is just "does this entity exist now" -- ALREADY_EXISTS
 * means success just as much as OK/OK_MATCHED does; see
 * create_reply_ok_or_exists() below. */
#define XRCE_STATUS_ERR_ALREADY_EXISTS 0x82

static bool create_reply_ok_or_exists(const uint8_t *buf, size_t len) {
    if (xrce_session_parse_create_reply(buf, len)) {
        return true;
    }
    /* Same shape as xrce_session_parse_create_reply(), just also accepting
     * ALREADY_EXISTS -- status byte is the last one in an 18-byte STATUS
     * reply (8-byte header + 4-byte subheader + 4-byte related_request +
     * 1-byte status). */
    return len >= 18 && buf[8] == 5 /* XRCE_SUBMSG_STATUS */ && buf[16] == XRCE_STATUS_ERR_ALREADY_EXISTS;
}

static bool send_and_wait_ok(int fd, struct sockaddr_in *addr, const char *step_name,
                              size_t (*build)(void *ctx, uint8_t *buf, size_t cap), void *build_ctx,
                              bool (*parse_reply)(const uint8_t *, size_t)) {
    for (int attempt = 0; attempt < 30; attempt++) {
        uint8_t msg[512];
        size_t len = build(build_ctx, msg, sizeof(msg));
        if (sendto(fd, msg, len, 0, (struct sockaddr *)addr, sizeof(*addr)) != (ssize_t)len) {
            perror("sendto");
            return false;
        }
        uint8_t in[256];
        ssize_t n = recv(fd, in, sizeof(in), 0);
        if (n <= 0) {
            continue; /* dropped, or nothing arrived within the timeout -- resend */
        }
        if (!parse_reply(in, (size_t)n)) {
            continue; /* stale reply from an earlier attempt/step, or a real rejection --
                       * either way, resending is the right recovery here */
        }
        printf("%-24s -> OK (attempt %d)\n", step_name, attempt + 1);
        fflush(stdout);
        return true;
    }
    fprintf(stderr, "FAIL [%s]: no matching reply after retries\n", step_name);
    return false;
}

static size_t build_create_client_cb(void *ctx, uint8_t *buf, size_t cap) {
    return xrce_session_build_create_client((xrce_session_t *)ctx, buf, cap);
}

/* Shared rebuild context/callback for every CREATE-family message below --
 * participant uses domain_id, everything else uses parent_id, matching
 * xrce_session_build_create_participant()/build_create_xml()'s own split. */
typedef struct {
    xrce_session_t *s;
    uint8_t stream_id_raw;
    xrce_object_id_t object_id;
    xrce_object_id_t parent_id;
    int16_t domain_id;
    const char *xml;
    bool is_participant;
} create_ctx_t;

static size_t build_create_cb(void *vctx, uint8_t *buf, size_t cap) {
    create_ctx_t *ctx = vctx;
    if (ctx->is_participant) {
        return xrce_session_build_create_participant(ctx->s, ctx->stream_id_raw, ctx->object_id,
                                                       ctx->domain_id, ctx->xml, buf, cap);
    }
    return xrce_session_build_create_xml(ctx->s, ctx->stream_id_raw, ctx->object_id, ctx->parent_id,
                                          ctx->xml, buf, cap);
}

/* Entity creation always goes out on the best-effort stream, even for the
 * "reliable" session below -- a reliable *stream* expects continuous,
 * gap-free sequence numbers from its very first use (confirmed
 * empirically: sending CREATE traffic on stream_id_raw=128 before any
 * WRITE_DATA made the agent reject the very next CREATE with a real
 * FAILED status, not a dropped-packet timeout -- moving entity creation to
 * the best-effort stream and reserving the reliable stream purely for the
 * sample data being measured fixed it). Only the datawriter itself is
 * created on `stream_id_raw` so its declared stream_id matches what
 * WRITE_DATA/HEARTBEAT will actually use. */
static bool setup_session(xrce_session_t *s, int fd, struct sockaddr_in *addr, uint8_t key_seed,
                           const char *topic_name, xrce_object_id_t *out_datawriter_id) {
    uint8_t key[4] = {key_seed, key_seed, key_seed, key_seed};
    xrce_session_init(s, 0x01, key, 512);

    if (!send_and_wait_ok(fd, addr, "CREATE_CLIENT", build_create_client_cb, s,
                           xrce_session_parse_create_client_reply)) {
        return false;
    }

    xrce_object_id_t participant_id = xrce_object_id(0x001, XRCE_OBJK_PARTICIPANT);
    create_ctx_t participant_ctx = {.s = s,
                                     .stream_id_raw = BEST_EFFORT_STREAM_RAW,
                                     .object_id = participant_id,
                                     .domain_id = 0,
                                     .xml = "<dds><participant><rtps><name>qos_demo</name>"
                                            "</rtps></participant></dds>",
                                     .is_participant = true};
    if (!send_and_wait_ok(fd, addr, "CREATE participant", build_create_cb, &participant_ctx,
                           create_reply_ok_or_exists)) {
        return false;
    }

    xrce_object_id_t topic_id = xrce_object_id(0x001, XRCE_OBJK_TOPIC);
    char topic_xml[256];
    snprintf(topic_xml, sizeof(topic_xml),
             "<dds><topic><name>rt/%s</name>"
             "<dataType>std_msgs::msg::dds_::Int32_</dataType></topic></dds>",
             topic_name);
    create_ctx_t topic_ctx = {.s = s,
                               .stream_id_raw = BEST_EFFORT_STREAM_RAW,
                               .object_id = topic_id,
                               .parent_id = participant_id,
                               .xml = topic_xml,
                               .is_participant = false};
    if (!send_and_wait_ok(fd, addr, "CREATE topic", build_create_cb, &topic_ctx,
                           create_reply_ok_or_exists)) {
        return false;
    }

    xrce_object_id_t publisher_id = xrce_object_id(0x001, XRCE_OBJK_PUBLISHER);
    create_ctx_t publisher_ctx = {.s = s,
                                   .stream_id_raw = BEST_EFFORT_STREAM_RAW,
                                   .object_id = publisher_id,
                                   .parent_id = participant_id,
                                   .xml = "",
                                   .is_participant = false};
    if (!send_and_wait_ok(fd, addr, "CREATE publisher", build_create_cb, &publisher_ctx,
                           create_reply_ok_or_exists)) {
        return false;
    }

    *out_datawriter_id = xrce_object_id(0x001, XRCE_OBJK_DATAWRITER);
    char dw_xml[384];
    snprintf(dw_xml, sizeof(dw_xml),
             "<dds><data_writer><topic><kind>NO_KEY</kind><name>rt/%s</name>"
             "<dataType>std_msgs::msg::dds_::Int32_</dataType></topic></data_writer></dds>",
             topic_name);
    create_ctx_t writer_ctx = {.s = s,
                                .stream_id_raw = BEST_EFFORT_STREAM_RAW,
                                .object_id = *out_datawriter_id,
                                .parent_id = publisher_id,
                                .xml = dw_xml,
                                .is_participant = false};
    return send_and_wait_ok(fd, addr, "CREATE datawriter", build_create_cb, &writer_ctx,
                             create_reply_ok_or_exists);
}

static size_t build_sample(int32_t value, uint8_t *out) {
    std_msgs_Int32 msg = {.data = value};
    uint8_t sample[16];
    size_t sample_len;
    std_msgs_Int32_encode(&msg, sample, sizeof(sample), &sample_len);
    memcpy(out, sample + 4, sample_len - 4); /* strip our CDR header, see design.md's Phase 4 note */
    return sample_len - 4;
}

int main(int argc, char **argv) {
    if (argc != 6) {
        fprintf(stderr, "usage: %s <best_effort_proxy_ip> <port> <reliable_proxy_ip> <port> <count>\n",
                argv[0]);
        return 2;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    const char *be_ip = argv[1];
    uint16_t be_port = (uint16_t)atoi(argv[2]);
    const char *rel_ip = argv[3];
    uint16_t rel_port = (uint16_t)atoi(argv[4]);
    int count = atoi(argv[5]);

    struct sockaddr_in be_addr, rel_addr;
    int be_fd = make_socket(be_ip, be_port, &be_addr);
    int rel_fd = make_socket(rel_ip, rel_port, &rel_addr);

    xrce_session_t be_session, rel_session;
    xrce_object_id_t be_writer, rel_writer;
    printf("--- best-effort session ---\n");
    if (!setup_session(&be_session, be_fd, &be_addr, 0xB0, "qos_best_effort", &be_writer)) {
        return 1;
    }
    printf("--- reliable session ---\n");
    uint8_t reliable_stream_raw = xrce_reliable_stream_raw_id(0);
    if (!setup_session(&rel_session, rel_fd, &rel_addr, 0xB1, "qos_reliable", &rel_writer)) {
        return 1;
    }

    /* DDS discovery/matching for just-created entities isn't instant --
     * same note as host/bench_latency.c's burst-mode caveat and
     * host/live_delete_demo.c's design.md writeup. Without this, a burst
     * of publishes right after CREATE confounds discovery latency with
     * actual loss and looks like 100% loss regardless of the proxy. */
    printf("\nsettling (letting DDS discovery finish) before publishing...\n");
    sleep(5);

    printf("publishing %d samples on each topic (best-effort: fire and forget; "
           "reliable: tracked + retried) through a lossy proxy...\n\n",
           count);

    /* Best-effort: no tracking, no retry -- exactly what a plain
     * WRITE_DATA on a best-effort stream already does. */
    uint64_t be_start = now_ms();
    for (int i = 0; i < count; i++) {
        uint8_t sample[16];
        size_t sample_len = build_sample(i, sample);
        uint8_t buf[128];
        size_t len = xrce_session_build_write_data(&be_session, BEST_EFFORT_STREAM_RAW, be_writer, sample,
                                                     sample_len, buf, sizeof(buf));
        sendto(be_fd, buf, len, 0, (struct sockaddr *)&be_addr, sizeof(be_addr));
    }
    uint64_t be_elapsed = now_ms() - be_start;
    printf("best-effort: sent %d samples in %llums, no retries (none possible)\n", count,
           (unsigned long long)be_elapsed);

    /* Reliable: track every sample, then drain with periodic HEARTBEATs
     * soliciting ACKNACKs and retransmitting whatever the agent reports
     * missing -- bounded rounds so a demo run can't hang forever if
     * something is badly wrong.
     *
     * A real bug, found only by logging the agent's actual ACKNACKs: the
     * agent's reliable *input* stream tracks its own expected sequence
     * number per stream, starting fresh at 0 the first time it ever sees
     * traffic on that stream -- independent of what number this session's
     * shared out_seq_num counter happens to be at. Setup already advanced
     * out_seq_num well past 0 sending CREATE on the best-effort stream, so
     * the first reliable WRITE_DATA arrived carrying (say) seq 15 on a
     * stream the agent had never seen before; its ACKNACK kept reporting
     * first_unacked=0, asking for 15 sequence numbers that this client
     * never actually sent *on that stream* and never could retransmit.
     * xrce_session_t has one seq_num counter shared across every stream a
     * session uses (documented in session.h as this project's "single
     * best-effort output stream" simplification) -- fine as long as only
     * one stream is ever used, which stops being true the moment a second,
     * reliable stream joins in. Since the reliable stream is used for the
     * first time right here and stream 1 (best-effort) is never touched
     * again in this session, resetting the counter to 0 at this exact
     * point is the correct fix, not a hack: from here on it *is* stream
     * 128's own counter. */
    rel_session.out_seq_num = 0;

    xrce_reliable_out_t ro;
    xrce_reliable_out_init(&ro);
    uint64_t rel_start = now_ms();
    uint16_t rel_start_seq = rel_session.out_seq_num;
    ro.last_acknown = (uint16_t)(rel_start_seq - 1); /* so the first HEARTBEAT's first_unacked
                                                        * is the real first seq sent on this
                                                        * stream (0), not stale from setup. */
    for (int i = 0; i < count; i++) {
        uint8_t sample[16];
        size_t sample_len = build_sample(i, sample);
        uint16_t seq = rel_session.out_seq_num;
        uint8_t buf[128];
        size_t len = xrce_session_build_write_data(&rel_session, reliable_stream_raw, rel_writer, sample,
                                                     sample_len, buf, sizeof(buf));
        xrce_reliable_out_track(&ro, seq, buf, len);
        sendto(rel_fd, buf, len, 0, (struct sockaddr *)&rel_addr, sizeof(rel_addr));
    }

    int rounds = 0;
    const int max_rounds = 12;
    for (; rounds < max_rounds; rounds++) {
        uint8_t hb[32];
        size_t hb_len = xrce_session_build_heartbeat(&rel_session, reliable_stream_raw, reliable_stream_raw,
                                                       (uint16_t)(ro.last_acknown + 1), ro.last_sent, hb,
                                                       sizeof(hb));
        sendto(rel_fd, hb, hb_len, 0, (struct sockaddr *)&rel_addr, sizeof(rel_addr));

        uint8_t in[256];
        ssize_t n;
        bool got_acknack = false;
        uint64_t round_start = now_ms();
        while (now_ms() - round_start < 150) {
            n = recv(rel_fd, in, sizeof(in), 0);
            if (n <= 0) {
                continue;
            }
            uint16_t first_unacked, bitmap;
            if (xrce_session_parse_acknack(in, (size_t)n, &first_unacked, &bitmap)) {
                xrce_reliable_out_on_acknack(&ro, first_unacked, bitmap);
                got_acknack = true;
            }
        }
        if (!got_acknack) {
            continue;
        }

        const uint8_t *msg;
        size_t msg_len;
        while (xrce_reliable_out_next_retransmit(&ro, &msg, &msg_len)) {
            sendto(rel_fd, msg, msg_len, 0, (struct sockaddr *)&rel_addr, sizeof(rel_addr));
        }
        if (ro.last_acknown == (uint16_t)(rel_start_seq + count - 1)) { /* every sent seq now acked */
            break;
        }
    }
    uint64_t rel_elapsed = now_ms() - rel_start;
    printf("reliable:    sent %d samples in %llums across %d heartbeat round(s), "
           "%u retransmitted byte-for-byte, last_acknown=%u (fully acked once it reaches %u)\n",
           count, (unsigned long long)rel_elapsed, rounds + 1, ro.retransmit_count, ro.last_acknown,
           (uint16_t)(rel_start_seq + count - 1));

    printf("\nnow check what actually arrived, e.g.:\n"
           "  timeout 6 ros2 topic echo /qos_best_effort | grep -c data:\n"
           "  timeout 6 ros2 topic echo /qos_reliable | grep -c data:\n");
    return 0;
}
