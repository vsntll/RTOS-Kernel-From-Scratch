/* Manual, one-off demo (same reasoning as live_publish_demo.c): proves
 * xrce_session_build_delete() against a real, unmodified MicroXRCEAgent --
 * not just that the agent accepts a DELETE, but that the entity it names
 * actually stops existing (`ros2 topic list` no longer shows it), which is
 * the real 7a deliverable.
 *
 * Creates participant/topic/publisher/datawriter for rt/delete_test,
 * publishes one sample, waits (so an external `ros2 topic list` can
 * confirm it's there), then deletes datawriter+publisher+topic (child
 * before parent -- not required by the protocol, just tidy) and waits
 * again (so the same external check can confirm it's gone).
 *
 * Usage (agent already running: `MicroXRCEAgent udp4 -p 8888`):
 *   gcc -Ixrce/include host/live_delete_demo.c xrce/build/libxrce.a -o /tmp/live_delete_demo
 *   /tmp/live_delete_demo 127.0.0.1 8888
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

    uint8_t key[4] = {0xDE, 0x1E, 0x7E, 0x57};
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
        "<dds><participant><rtps><name>rtos_delete_demo</name></rtps></participant></dds>", buf,
        sizeof(buf));
    if (!send_and_wait_ok("CREATE participant", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t topic_id = xrce_object_id(0x001, XRCE_OBJK_TOPIC);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, topic_id, participant_id,
                                         "<dds><topic><name>rt/delete_test</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType></topic></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE topic", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t publisher_id = xrce_object_id(0x001, XRCE_OBJK_PUBLISHER);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, publisher_id, participant_id, "",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE publisher", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t datawriter_id = xrce_object_id(0x001, XRCE_OBJK_DATAWRITER);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, datawriter_id, publisher_id,
                                         "<dds><data_writer><topic><kind>NO_KEY</kind>"
                                         "<name>rt/delete_test</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType>"
                                         "</topic></data_writer></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE datawriter", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    std_msgs_Int32 msg = {.data = 1};
    uint8_t sample[16];
    size_t sample_len;
    std_msgs_Int32_encode(&msg, sample, sizeof(sample), &sample_len);
    len = xrce_session_build_write_data(&s, BEST_EFFORT_STREAM_0, datawriter_id, sample + 4,
                                         sample_len - 4, buf, sizeof(buf));
    sendto(g_fd, buf, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr));
    printf("published one sample on rt/delete_test -- entities exist now\n");
    fflush(stdout);
    /* DDS discovery/matching for just-created entities isn't instant (same
     * note as host/bench_latency.c's burst-mode caveat) -- give `ros2 topic
     * list` real time to see it before deleting, rather than a delay tuned
     * just long enough to make CREATE's own STATUS replies come back. */
    sleep(10);

    len = xrce_session_build_delete(&s, BEST_EFFORT_STREAM_0, datawriter_id, buf, sizeof(buf));
    if (!send_and_wait_ok("DELETE datawriter", buf, len, xrce_session_parse_delete_reply)) {
        return 1;
    }
    len = xrce_session_build_delete(&s, BEST_EFFORT_STREAM_0, publisher_id, buf, sizeof(buf));
    if (!send_and_wait_ok("DELETE publisher", buf, len, xrce_session_parse_delete_reply)) {
        return 1;
    }
    len = xrce_session_build_delete(&s, BEST_EFFORT_STREAM_0, topic_id, buf, sizeof(buf));
    if (!send_and_wait_ok("DELETE topic", buf, len, xrce_session_parse_delete_reply)) {
        return 1;
    }

    printf("deleted datawriter/publisher/topic -- entities should be gone now\n");
    fflush(stdout);
    sleep(6);
    return 0;
}
