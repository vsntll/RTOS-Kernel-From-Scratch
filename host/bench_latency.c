/* Phase 6: measures real host<->RTOS round-trip latency through the
 * ros2_demo.c firmware (running in QEMU, over the actual serial
 * transport into a real agent -- or any other client speaking rt/setpoint
 * -> rt/pong, including a second copy of this project's own client).
 *
 * For each of COUNT iterations: publish an incrementing value on
 * rt/setpoint, then block (with a timeout) until a matching value is
 * read back on rt/pong, timing the gap with a monotonic clock. This is a
 * real, external, wall-clock measurement -- not something computed from
 * QEMU's own (deliberately uncalibrated, see rtos/arm/main.c's
 * SYSTICK_RELOAD comment) sense of time, so it captures the full path:
 * this process -> agent -> DDS -> agent -> serial -> QEMU's UART ->
 * firmware's poll loop -> echo -> back through the same chain.
 *
 * Usage (agent already running, ros2_demo.c already booted and attached):
 *   gcc -Ixrce/include host/bench_latency.c xrce/build/libxrce.a -o /tmp/bench
 *   /tmp/bench 127.0.0.1 8888 [count]
 */
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "xrce/msgs.h"
#include "xrce/session.h"

#define BEST_EFFORT_STREAM_0 1
#define DEFAULT_COUNT 20
#define PER_SAMPLE_TIMEOUT_MS 3000

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

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

static int cmp_double(const void *a, const void *b) {
    double da = *(const double *)a, db = *(const double *)b;
    return (da > db) - (da < db);
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <agent_ip> <agent_port> [count | burst <n>]\n", argv[0]);
        return 2;
    }
    bool burst_mode = (argc >= 4) && (strcmp(argv[3], "burst") == 0);
    int count = burst_mode ? ((argc >= 5) ? atoi(argv[4]) : DEFAULT_COUNT)
                            : ((argc >= 4) ? atoi(argv[3]) : DEFAULT_COUNT);

    g_fd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&g_addr, 0, sizeof(g_addr));
    g_addr.sin_family = AF_INET;
    g_addr.sin_port = htons((uint16_t)atoi(argv[2]));
    inet_pton(AF_INET, argv[1], &g_addr.sin_addr);
    struct timeval tv = {.tv_sec = 3, .tv_usec = 0};
    setsockopt(g_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    /* Seeded by pid rather than a fixed constant: a fixed client_key
     * collides with this same tool's own previous run against a
     * still-running (long-lived) agent -- the agent still has the old
     * session's participant/topic/etc registered under that key, so a
     * second run's CREATE calls come back ALREADY_EXISTS instead of OK.
     * Found by actually hitting it while re-running this tool repeatedly
     * against one agent process. */
    uint8_t key[4] = {0x42, 0x45, (uint8_t)(getpid() >> 8), (uint8_t)getpid()};
    xrce_session_t s;
    xrce_session_init(&s, 0x02, key, 512);

    uint8_t buf[512];
    size_t len;

    len = xrce_session_build_create_client(&s, buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE_CLIENT", buf, len, xrce_session_parse_create_client_reply)) {
        return 1;
    }

    xrce_object_id_t participant_id = xrce_object_id(0x001, XRCE_OBJK_PARTICIPANT);
    len = xrce_session_build_create_participant(
        &s, BEST_EFFORT_STREAM_0, participant_id, 0,
        "<dds><participant><rtps><name>rtos_bench</name></rtps></participant></dds>", buf,
        sizeof(buf));
    if (!send_and_wait_ok("CREATE participant", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    /* Publish side: rt/setpoint (the "ping"). */
    xrce_object_id_t ping_topic_id = xrce_object_id(0x001, XRCE_OBJK_TOPIC);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, ping_topic_id, participant_id,
                                         "<dds><topic><name>rt/setpoint</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType></topic></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE topic (ping)", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t publisher_id = xrce_object_id(0x001, XRCE_OBJK_PUBLISHER);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, publisher_id, participant_id, "",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE publisher", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t ping_datawriter_id = xrce_object_id(0x001, XRCE_OBJK_DATAWRITER);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, ping_datawriter_id, publisher_id,
                                         "<dds><data_writer><topic><kind>NO_KEY</kind>"
                                         "<name>rt/setpoint</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType>"
                                         "</topic></data_writer></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE datawriter (ping)", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    /* Subscribe side: rt/pong (the echo). */
    xrce_object_id_t pong_topic_id = xrce_object_id(0x002, XRCE_OBJK_TOPIC);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, pong_topic_id, participant_id,
                                         "<dds><topic><name>rt/pong</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType></topic></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE topic (pong)", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t subscriber_id = xrce_object_id(0x001, XRCE_OBJK_SUBSCRIBER);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, subscriber_id, participant_id,
                                         "", buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE subscriber", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t pong_datareader_id = xrce_object_id(0x001, XRCE_OBJK_DATAREADER);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, pong_datareader_id,
                                         subscriber_id,
                                         "<dds><data_reader><topic><kind>NO_KEY</kind>"
                                         "<name>rt/pong</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType>"
                                         "</topic></data_reader></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE datareader (pong)", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    len = xrce_session_build_read_data(&s, BEST_EFFORT_STREAM_0, pong_datareader_id,
                                        BEST_EFFORT_STREAM_0, buf, sizeof(buf));
    if (sendto(g_fd, buf, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr)) != (ssize_t)len) {
        perror("sendto READ_DATA");
        return 1;
    }
    printf("READ_DATA sent -- pinging rt/setpoint, waiting for rt/pong echoes\n\n");

    if (burst_mode) {
        /* DDS discovery/matching for these just-created entities takes a
         * real, non-zero amount of wall-clock time -- the sequential mode
         * gets this for free (each ping retries against a per-message
         * timeout), but firing a burst immediately after setup would
         * measure discovery latency, not RX throughput. Settle first. */
        usleep(500 * 1000);
        /* Throughput/loss characterization: fire `count` pings back-to-back
         * with no per-ping wait (unlike the default one-at-a-time round
         * trip mode above), then spend a fixed window collecting whatever
         * pongs arrive and report how many of the `count` sent actually
         * made it back. This is meaningful specifically because
         * ros2_demo.c's UART RX is polled, not interrupt-driven (see its
         * own header comment) -- there is no hardware/software FIFO
         * backing it beyond what pace_and_poll_rx() drains between other
         * work, so loss under a burst is a real, expected characteristic
         * of this implementation, not a bug to fix. Best-effort delivery
         * (Phase 1/3's own design choice, see xrce/docs/design.md) means
         * some loss is expected by protocol design too, independent of
         * this specific RX limitation. */
        double t_start = now_ms();
        for (int i = 1; i <= count; i++) {
            std_msgs_Int32 msg = {.data = i};
            uint8_t sample[16];
            size_t sample_len;
            std_msgs_Int32_encode(&msg, sample, sizeof(sample), &sample_len);
            len = xrce_session_build_write_data(&s, BEST_EFFORT_STREAM_0, ping_datawriter_id,
                                                 sample + 4, sample_len - 4, buf, sizeof(buf));
            sendto(g_fd, buf, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr));
        }
        double t_sent = now_ms();
        printf("sent %d pings back-to-back in %.2f ms\n", count, t_sent - t_start);

        bool seen[4096] = {0};
        int distinct_received = 0;
        double collect_deadline = now_ms() + (double)PER_SAMPLE_TIMEOUT_MS;
        while (now_ms() < collect_deadline) {
            uint8_t in[256];
            ssize_t n = recv(g_fd, in, sizeof(in), 0);
            if (n <= 0) {
                continue;
            }
            xrce_object_id_t from_id;
            const uint8_t *echoed;
            size_t echoed_len;
            if (!xrce_session_parse_data(in, (size_t)n, &from_id, &echoed, &echoed_len) ||
                echoed_len < 4) {
                continue;
            }
            int32_t value = (int32_t)((uint32_t)echoed[0] | ((uint32_t)echoed[1] << 8) |
                                       ((uint32_t)echoed[2] << 16) | ((uint32_t)echoed[3] << 24));
            if (value >= 1 && value <= count && !seen[value]) {
                seen[value] = true;
                distinct_received++;
            }
            if (distinct_received == count) {
                break; /* got everything; no need to wait out the full timeout */
            }
        }
        double t_done = now_ms();
        printf("received %d/%d distinct echoes within %.2f ms (%.1f%% loss)\n", distinct_received,
               count, t_done - t_sent, 100.0 * (double)(count - distinct_received) / count);
        return 0;
    }

    double latencies_ms[4096];
    int received = 0;
    int timed_out = 0;
    int to_run = count < (int)(sizeof(latencies_ms) / sizeof(latencies_ms[0])) ? count
                                                                                : (int)(sizeof(latencies_ms) / sizeof(latencies_ms[0]));

    for (int i = 1; i <= to_run; i++) {
        std_msgs_Int32 msg = {.data = i};
        uint8_t sample[16];
        size_t sample_len;
        std_msgs_Int32_encode(&msg, sample, sizeof(sample), &sample_len);

        double t0 = now_ms();
        len = xrce_session_build_write_data(&s, BEST_EFFORT_STREAM_0, ping_datawriter_id,
                                             sample + 4, sample_len - 4, buf, sizeof(buf));
        if (sendto(g_fd, buf, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr)) !=
            (ssize_t)len) {
            perror("sendto ping");
            continue;
        }

        bool got_it = false;
        double deadline = t0 + PER_SAMPLE_TIMEOUT_MS;
        while (now_ms() < deadline) {
            uint8_t in[256];
            ssize_t n = recv(g_fd, in, sizeof(in), 0);
            if (n <= 0) {
                continue;
            }
            xrce_object_id_t from_id;
            const uint8_t *echoed;
            size_t echoed_len;
            if (!xrce_session_parse_data(in, (size_t)n, &from_id, &echoed, &echoed_len) ||
                echoed_len < 4) {
                continue;
            }
            int32_t value = (int32_t)((uint32_t)echoed[0] | ((uint32_t)echoed[1] << 8) |
                                       ((uint32_t)echoed[2] << 16) | ((uint32_t)echoed[3] << 24));
            if (value != i) {
                continue; /* stale echo from a previous ping; keep waiting for this one */
            }
            double t1 = now_ms();
            latencies_ms[received++] = t1 - t0;
            got_it = true;
            break;
        }
        if (!got_it) {
            timed_out++;
            printf("ping %3d: TIMED OUT (no echo within %dms)\n", i, PER_SAMPLE_TIMEOUT_MS);
        } else {
            printf("ping %3d: %.2f ms\n", i, latencies_ms[received - 1]);
        }
    }

    printf("\n%d/%d round trips completed, %d timed out\n", received, to_run, timed_out);
    if (received > 0) {
        qsort(latencies_ms, (size_t)received, sizeof(double), cmp_double);
        double sum = 0;
        for (int i = 0; i < received; i++) {
            sum += latencies_ms[i];
        }
        double mean = sum / received;
        double p50 = latencies_ms[received / 2];
        double p95 = latencies_ms[(received * 95) / 100 < received ? (received * 95) / 100
                                                                    : received - 1];
        printf("min=%.2fms  p50=%.2fms  mean=%.2fms  p95=%.2fms  max=%.2fms\n", latencies_ms[0],
               p50, mean, p95, latencies_ms[received - 1]);
    }
    return 0;
}
