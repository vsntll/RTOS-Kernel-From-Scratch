/* Manual, one-off program (same reasoning as live_publish_demo.c) proving
 * the other half of Phase 5 against a real, unmodified MicroXRCEAgent:
 * this project's client creates a subscription, and a real `ros2 topic
 * pub` on the host drives it -- the "host sends a command, the RTOS task
 * receives and acts on it" scenario from the original project brief,
 * minus the RTOS part (this is the host-side proof; wiring it into
 * rtos/arm/ros2_demo.c over UART would need RX support uart.c doesn't
 * have yet -- see xrce/docs/design.md).
 *
 * Usage (agent already running: `MicroXRCEAgent udp4 -p 8888`):
 *   gcc -Ixrce/include host/live_subscribe_demo.c xrce/build/libxrce.a -o /tmp/sub_demo
 *   /tmp/sub_demo 127.0.0.1 8888
 *   # separately: ros2 topic pub /cmd std_msgs/msg/Int32 "{data: 99}" --once
 */
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

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
    setvbuf(stdout, NULL, _IONBF, 0); /* unbuffered: stdout is usually redirected to a
                                        * log file when this runs detached, and fully
                                        * buffered stdio would otherwise hide output
                                        * until the (never-reached, infinite-loop) exit */
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

    uint8_t key[4] = {0x53, 0x55, 0x42, 0x00}; /* "SUB\0" */
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
        "<dds><participant><rtps><name>rtos_sub_demo</name></rtps></participant></dds>", buf,
        sizeof(buf));
    if (!send_and_wait_ok("CREATE participant", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t topic_id = xrce_object_id(0x001, XRCE_OBJK_TOPIC);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, topic_id, participant_id,
                                         "<dds><topic><name>rt/cmd</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType></topic></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE topic", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t subscriber_id = xrce_object_id(0x001, XRCE_OBJK_SUBSCRIBER);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, subscriber_id, participant_id,
                                         "", buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE subscriber", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t datareader_id = xrce_object_id(0x001, XRCE_OBJK_DATAREADER);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, datareader_id, subscriber_id,
                                         "<dds><data_reader><topic><kind>NO_KEY</kind>"
                                         "<name>rt/cmd</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType>"
                                         "</topic></data_reader></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE datareader", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    len = xrce_session_build_read_data(&s, BEST_EFFORT_STREAM_0, datareader_id,
                                        BEST_EFFORT_STREAM_0, buf, sizeof(buf));
    if (sendto(g_fd, buf, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr)) != (ssize_t)len) {
        perror("sendto READ_DATA");
        return 1;
    }
    printf("READ_DATA sent -- waiting for /cmd samples "
           "(run: ros2 topic pub /cmd std_msgs/msg/Int32 \"{data: 99}\" --once)\n\n");

    for (;;) {
        uint8_t in[256];
        ssize_t n = recv(g_fd, in, sizeof(in), 0);
        if (n <= 0) {
            continue; /* timeout: keep waiting */
        }
        xrce_object_id_t from_id;
        const uint8_t *sample;
        size_t sample_len;
        if (!xrce_session_parse_data(in, (size_t)n, &from_id, &sample, &sample_len)) {
            continue; /* STATUS replies etc. for the entities above; ignore */
        }
        if (sample_len < 4) {
            continue;
        }
        int32_t value = (int32_t)((uint32_t)sample[0] | ((uint32_t)sample[1] << 8) |
                                   ((uint32_t)sample[2] << 16) | ((uint32_t)sample[3] << 24));
        printf("received /cmd data=%d (from object 0x%03x kind 0x%x)\n", value, from_id.id,
               from_id.kind);
    }
}
