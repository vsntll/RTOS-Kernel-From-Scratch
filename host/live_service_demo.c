/* Manual, one-off demo (same reasoning as live_publish_demo.c): a real
 * XRCE Replier answering a real, unmodified ROS2 CLI's `ros2 service call`
 * -- request/reply correlation over the actual protocol wire bytes, not a
 * custom bridge. Serves std_srvs/srv/Trigger under the name "self_test"
 * (matches the phase brief's own "trigger a self-test" example).
 *
 * REQUESTER/REPLIER are CREATE'd exactly like any other XML entity
 * (xrce_session_build_create_xml() -- see session.h's header comment) and
 * WRITE_DATA/READ_DATA/DATA carry requests/replies unchanged, except a
 * Replier's request DATA and reply WRITE_DATA are each prefixed with a
 * 24-byte SampleIdentity this project only ever stores-and-replays, never
 * interprets (XRCE_SAMPLE_IDENTITY_SIZE, session.h).
 *
 * Usage (agent already running: `MicroXRCEAgent udp4 -p 8888`):
 *   gcc -Ixrce/include host/live_service_demo.c xrce/build/libxrce.a -o /tmp/live_service_demo
 *   /tmp/live_service_demo 127.0.0.1 8888
 *   # elsewhere: ros2 service call /self_test std_srvs/srv/Trigger
 */
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "xrce/msgs.h"
#include "xrce/session.h"

#define BEST_EFFORT_STREAM_0 1

static int g_fd;
static struct sockaddr_in g_addr;

static bool send_and_wait_ok(const char *step_name, const uint8_t *msg, size_t len,
                              bool (*parse_reply)(const uint8_t *, size_t)) {
    if (sendto(g_fd, msg, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr)) != (ssize_t)len) {
        perror("sendto");
        return false;
    }
    uint8_t in[256];
    ssize_t n = recv(g_fd, in, sizeof(in), 0);
    if (n <= 0) {
        fprintf(stderr, "FAIL [%s]: no reply\n", step_name);
        return false;
    }
    bool ok = parse_reply(in, (size_t)n);
    printf("%-24s -> %s (%zd byte reply)\n", step_name, ok ? "OK" : "FAILED", n);
    return ok;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s <agent_ip> <agent_port>\n", argv[0]);
        return 2;
    }

    g_fd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&g_addr, 0, sizeof(g_addr));
    g_addr.sin_family = AF_INET;
    g_addr.sin_port = htons((uint16_t)atoi(argv[2]));
    inet_pton(AF_INET, argv[1], &g_addr.sin_addr);
    struct timeval tv = {.tv_sec = 3, .tv_usec = 0};
    setsockopt(g_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t key[4] = {0x5E, 0x27, 0xE5, 0x70};
    xrce_session_t s;
    xrce_session_init(&s, 0x01, key, 512);

    uint8_t buf[512];
    size_t len;

    len = xrce_session_build_create_client(&s, buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE_CLIENT", buf, len, xrce_session_parse_create_client_reply)) {
        return 1;
    }

    xrce_object_id_t participant_id = xrce_object_id(0x001, XRCE_OBJK_PARTICIPANT);
    len = xrce_session_build_create_participant(
        &s, BEST_EFFORT_STREAM_0, participant_id, 0,
        "<dds><participant><rtps><name>rtos_service_demo</name></rtps></participant></dds>", buf,
        sizeof(buf));
    if (!send_and_wait_ok("CREATE participant", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    /* profile_name/service_name/request_type/reply_type attribute shape is
     * ground-truthed against the reference client's own
     * examples/RequestAdder/main.c -- unlike topic/datawriter XML (element
     * children), requester/replier XML uses attributes directly on the
     * <requester>/<replier> tag. Type name mangling ("pkg::srv::dds_::
     * Type_Request_"/"..._Response_") follows the same convention already
     * verified for topics ("pkg::msg::dds_::Type_"). request_topic_name/
     * reply_topic_name are set explicitly to ROS2's own DDS-RPC naming
     * convention (design.ros2.org/articles/topic_and_service_names.html:
     * request topic = "rq" + fully-qualified service name + "Request",
     * reply topic = "rr" + ... + "Reply") -- FastDDS's own default
     * derivation from a bare `service_name` does NOT produce this, so a
     * real `ros2 service list` never discovered the entity without these
     * set explicitly (confirmed by testing: bare service_name alone
     * created a replier the agent accepted, but it was invisible to ROS2's
     * service graph). */
    xrce_object_id_t replier_id = xrce_object_id(0x001, XRCE_OBJK_REPLIER);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, replier_id, participant_id,
                                         "<dds><replier profile_name=\"self_test_replier\" "
                                         "service_name=\"self_test\" "
                                         "request_type=\"std_srvs::srv::dds_::Trigger_Request_\" "
                                         "reply_type=\"std_srvs::srv::dds_::Trigger_Response_\">"
                                         "<request_topic_name>rq/self_testRequest</request_topic_name>"
                                         "<reply_topic_name>rr/self_testReply</reply_topic_name>"
                                         "</replier></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE replier", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    len = xrce_session_build_read_data(&s, BEST_EFFORT_STREAM_0, replier_id, BEST_EFFORT_STREAM_0,
                                        buf, sizeof(buf));
    if (sendto(g_fd, buf, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr)) != (ssize_t)len) {
        perror("sendto READ_DATA");
        return 1;
    }
    printf("READ_DATA (requests)     -> sent\n");

    printf("\nServing std_srvs/Trigger on /self_test -- run "
           "`ros2 service call /self_test std_srvs/srv/Trigger` now. Ctrl-C to stop.\n\n");

    int reply_count = 0;
    for (;;) {
        uint8_t in[256];
        ssize_t n = recv(g_fd, in, sizeof(in), 0);
        if (n <= 0) {
            continue; /* recv timeout (tv above) -- just poll again */
        }

        xrce_object_id_t from_id;
        const uint8_t *sample;
        size_t sample_len;
        if (!xrce_session_parse_data(in, (size_t)n, &from_id, &sample, &sample_len)) {
            continue;
        }
        if (sample_len < XRCE_SAMPLE_IDENTITY_SIZE) {
            fprintf(stderr, "request sample too short for a SampleIdentity prefix\n");
            continue;
        }
        const uint8_t *sample_identity = sample;
        /* Received samples are header-less, the mirror image of WRITE_DATA
         * (see host/live_subscribe_demo.c and xrce/docs/design.md's Phase 5
         * section) -- std_srvs_Trigger_Request_decode() expects a CDR
         * header (for round-tripping this project's own encode() output)
         * and would wrongly reject a real wire request. Trigger's request
         * has no fields anyway, so there's nothing to decode: the entire
         * request payload is just the empty CDR encoding of nothing.
         * Ignored here rather than mis-decoded. */

        reply_count++;
        printf("request #%d received -- replying\n", reply_count);

        std_srvs_Trigger_Response resp = {.success = true};
        snprintf(resp.message, sizeof(resp.message), "self-test #%d ok", reply_count);
        uint8_t resp_sample[512];
        size_t resp_sample_len;
        if (!std_srvs_Trigger_Response_encode(&resp, resp_sample, sizeof(resp_sample), &resp_sample_len)) {
            fprintf(stderr, "failed to encode Trigger response\n");
            continue;
        }

        /* Prefix with the SampleIdentity captured from the request (raw,
         * opaque, replayed verbatim -- see session.h), and strip our own
         * 4-byte CDR header the same way WRITE_DATA does for plain topics
         * (the agent's generic dynamic type adds its own). */
        uint8_t wire[XRCE_SAMPLE_IDENTITY_SIZE + 512];
        memcpy(wire, sample_identity, XRCE_SAMPLE_IDENTITY_SIZE);
        memcpy(wire + XRCE_SAMPLE_IDENTITY_SIZE, resp_sample + 4, resp_sample_len - 4);
        size_t wire_len = XRCE_SAMPLE_IDENTITY_SIZE + (resp_sample_len - 4);

        len = xrce_session_build_write_data(&s, BEST_EFFORT_STREAM_0, replier_id, wire, wire_len, buf,
                                             sizeof(buf));
        if (sendto(g_fd, buf, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr)) != (ssize_t)len) {
            perror("sendto reply");
        } else {
            printf("reply #%d sent\n", reply_count);
        }
    }
}
