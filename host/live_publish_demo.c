/* Manual, one-off demo (not part of `make test`, same reasoning as
 * live_agent_check.c): runs the full entity-creation pipeline against a
 * real, unmodified MicroXRCEAgent -- participant, topic, publisher,
 * datawriter -- then publishes a std_msgs/Int32 once a second. Point
 * `ros2 topic echo /chatter` at the same agent while this runs; seeing
 * live values there is the actual Phase 4 deliverable.
 *
 * Usage (agent already running: `MicroXRCEAgent udp4 -p 8888`):
 *   gcc -Ixrce/include host/live_publish_demo.c xrce/build/libxrce.a -o /tmp/live_publish_demo
 *   /tmp/live_publish_demo 127.0.0.1 8888
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

    uint8_t key[4] = {0xCA, 0xFE, 0xF0, 0x0D};
    xrce_session_t s;
    xrce_session_init(&s, 0x01, key, 512);

    uint8_t buf[512];
    size_t len;

    len = xrce_session_build_create_client(&s, buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE_CLIENT", buf, len, xrce_session_parse_create_client_reply)) {
        return 1;
    }

    /* XML shapes here (the <dds> root wrapper, element-not-attribute
     * children) are ground-truthed against eProsima's own
     * examples/PublishHelloWorld/main.c, not guessed -- the attribute-style
     * shorthand this started with (`<dds_topic name="..." .../>`) fails
     * the agent's XML parser ("Not found root tag"). The "rt/" topic-name
     * prefix and "pkg::msg::dds_::Type_" dataType mangling are what
     * real micro-ROS/rmw_microxrcedds generate to make a topic visible to
     * `ros2 topic list`/`ros2 topic echo` -- without them, the agent still
     * creates the DDS entities, they just won't look like a ROS2 topic. */
    xrce_object_id_t participant_id = xrce_object_id(0x001, XRCE_OBJK_PARTICIPANT);
    len = xrce_session_build_create_participant(
        &s, BEST_EFFORT_STREAM_0, participant_id, 0,
        "<dds><participant><rtps><name>rtos_demo</name></rtps></participant></dds>", buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE participant", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t topic_id = xrce_object_id(0x001, XRCE_OBJK_TOPIC);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, topic_id, participant_id,
                                         "<dds><topic><name>rt/chatter</name>"
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
                                         "<name>rt/chatter</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType>"
                                         "</topic></data_writer></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE datawriter", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    printf("\nPublishing std_msgs/Int32 on /chatter every second -- "
           "run `ros2 topic echo /chatter` now. Ctrl-C to stop.\n\n");

    for (int32_t i = 0;; i++) {
        std_msgs_Int32 msg = {.data = i};
        uint8_t sample[16];
        size_t sample_len;
        if (!std_msgs_Int32_encode(&msg, sample, sizeof(sample), &sample_len)) {
            fprintf(stderr, "FAIL: encode\n");
            return 1;
        }
        len = xrce_session_build_write_data(&s, BEST_EFFORT_STREAM_0, datawriter_id, sample,
                                             sample_len, buf, sizeof(buf));
        if (sendto(g_fd, buf, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr)) != (ssize_t)len) {
            perror("sendto");
            return 1;
        }
        printf("published data=%d\n", i);
        sleep(1);
    }
}
