/* Phase 8 deliverable: a live inspector for the RTOS, running over the
 * same transport as the ROS2 traffic (Option A from the phase brief --
 * piggyback on ROS2 itself, reusing everything built in Phases 1-7d,
 * rather than a second out-of-band protocol). Publishes real per-task
 * state/priority/stack-high-water-mark, real scheduler stats (tick and
 * context-switch counts), and real queue depths as a standard
 * `diagnostic_msgs/DiagnosticArray` on `/rtos/diagnostics`, idiomatic
 * enough in principle for `rqt_robot_monitor` (not independently
 * confirmed against that specific tool here -- stated plainly rather than
 * overclaimed). A `std_srvs/Trigger`-typed `/rtos/diagnostics/refresh`
 * service (reusing 7b's Requester/Replier machinery unchanged) forces an
 * immediate publish on demand instead of waiting for the next periodic
 * tick.
 *
 * Two representative worker tasks (different priorities, one doing
 * periodic blocking work, one doing occasional busy work) exist purely so
 * there's real, changing task state to observe -- this file's own point
 * is the diagnostics plumbing, not the workers themselves.
 *
 * Usage (agent already running: `MicroXRCEAgent udp4 -p 8888`):
 *   gcc -Ixrce/include -Irtos/src host/live_diagnostics_demo.c \
 *       rtos/build/librtos.a xrce/build/libxrce.a -o /tmp/live_diagnostics_demo
 *   /tmp/live_diagnostics_demo 127.0.0.1 8888
 *   # elsewhere:
 *   ros2 topic echo /rtos/diagnostics diagnostic_msgs/msg/DiagnosticArray
 *   ros2 service call /rtos/diagnostics/refresh std_srvs/srv/Trigger
 */
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "kernel.h"
#include "sync.h"

#include "xrce/msgs.h"
#include "xrce/session.h"

#define BEST_EFFORT_STREAM_0 1
#define QUEUE_CAPACITY 4

static int g_fd;
static struct sockaddr_in g_addr;
static xrce_session_t g_s;
static uint64_t g_bytes_sent;

static bool send_and_wait_ok(const char *step_name, const uint8_t *msg, size_t len,
                              bool (*parse_reply)(const uint8_t *, size_t)) {
    if (sendto(g_fd, msg, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr)) != (ssize_t)len) {
        perror("sendto");
        return false;
    }
    g_bytes_sent += (uint64_t)len;
    uint8_t in[256];
    ssize_t n = recv(g_fd, in, sizeof(in), 0);
    if (n <= 0) {
        fprintf(stderr, "FAIL [%s]: no reply\n", step_name);
        return false;
    }
    bool ok = parse_reply(in, (size_t)n);
    printf("%-24s -> %s\n", step_name, ok ? "OK" : "FAILED");
    return ok;
}

static void send_tracked(const uint8_t *buf, size_t len) {
    if (sendto(g_fd, buf, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr)) == (ssize_t)len) {
        g_bytes_sent += (uint64_t)len;
    }
}

/* --- Representative worker tasks: real, changing task state to report --- */

static void *g_work_queue_storage[QUEUE_CAPACITY];
static queue_t g_work_queue;

static void producer_worker(void *arg) {
    (void)arg;
    for (int i = 0;; i++) {
        queue_send(&g_work_queue, (void *)(intptr_t)i);
        task_sleep(200);
    }
}

static void consumer_worker(void *arg) {
    (void)arg;
    for (;;) {
        queue_receive(&g_work_queue);
        task_sleep(350);
    }
}

static const char *task_state_name(task_state_t s) {
    switch (s) {
        case TASK_READY:
            return "READY";
        case TASK_RUNNING:
            return "RUNNING";
        case TASK_BLOCKED:
            return "BLOCKED";
        case TASK_TERMINATED:
            return "TERMINATED";
        default:
            return "UNKNOWN";
    }
}

static void add_task_status(diagnostic_msgs_DiagnosticArray *arr, const task_t *t) {
    diagnostic_msgs_DiagnosticStatus *st = &arr->status[arr->status_count++];
    memset(st, 0, sizeof(*st));
    st->level = DIAGNOSTIC_STATUS_OK;
    snprintf(st->name, sizeof(st->name), "task/%s", t->name);
    snprintf(st->message, sizeof(st->message), "%s", task_state_name(t->state));
    snprintf(st->hardware_id, sizeof(st->hardware_id), "rtos");

    int i = 0;
    snprintf(st->values[i].key, sizeof(st->values[i].key), "priority");
    snprintf(st->values[i].value, sizeof(st->values[i].value), "%d", t->priority);
    i++;
    snprintf(st->values[i].key, sizeof(st->values[i].key), "stack_high_water_mark_bytes");
    snprintf(st->values[i].value, sizeof(st->values[i].value), "%zu", task_stack_high_water_mark(t));
    i++;
    snprintf(st->values[i].key, sizeof(st->values[i].key), "ticks_run");
    snprintf(st->values[i].value, sizeof(st->values[i].value), "%llu",
             (unsigned long long)t->ticks_run);
    i++;
    snprintf(st->values[i].key, sizeof(st->values[i].key), "ticks_ready");
    snprintf(st->values[i].value, sizeof(st->values[i].value), "%llu",
             (unsigned long long)t->ticks_ready);
    i++;
    st->values_count = (uint32_t)i;
}

static void add_scheduler_status(diagnostic_msgs_DiagnosticArray *arr) {
    diagnostic_msgs_DiagnosticStatus *st = &arr->status[arr->status_count++];
    memset(st, 0, sizeof(*st));
    st->level = DIAGNOSTIC_STATUS_OK;
    snprintf(st->name, sizeof(st->name), "scheduler");
    snprintf(st->message, sizeof(st->message), "running");
    snprintf(st->hardware_id, sizeof(st->hardware_id), "rtos");

    snprintf(st->values[0].key, sizeof(st->values[0].key), "tick_count");
    snprintf(st->values[0].value, sizeof(st->values[0].value), "%llu",
             (unsigned long long)scheduler_tick_count());
    snprintf(st->values[1].key, sizeof(st->values[1].key), "context_switch_count");
    snprintf(st->values[1].value, sizeof(st->values[1].value), "%llu",
             (unsigned long long)scheduler_switch_count());
    st->values_count = 2;
}

static void add_queue_status(diagnostic_msgs_DiagnosticArray *arr, const char *name, const queue_t *q) {
    diagnostic_msgs_DiagnosticStatus *st = &arr->status[arr->status_count++];
    memset(st, 0, sizeof(*st));
    int depth = (q->tail - q->head + q->capacity) % q->capacity;
    st->level = (depth == q->capacity - 1) ? DIAGNOSTIC_STATUS_WARN : DIAGNOSTIC_STATUS_OK;
    snprintf(st->name, sizeof(st->name), "queue/%s", name);
    snprintf(st->message, sizeof(st->message), "depth %d/%d", depth, q->capacity);
    snprintf(st->hardware_id, sizeof(st->hardware_id), "rtos");
    snprintf(st->values[0].key, sizeof(st->values[0].key), "depth");
    snprintf(st->values[0].value, sizeof(st->values[0].value), "%d", depth);
    snprintf(st->values[1].key, sizeof(st->values[1].key), "capacity");
    snprintf(st->values[1].value, sizeof(st->values[1].value), "%d", q->capacity);
    st->values_count = 2;
}

static void add_middleware_status(diagnostic_msgs_DiagnosticArray *arr) {
    diagnostic_msgs_DiagnosticStatus *st = &arr->status[arr->status_count++];
    memset(st, 0, sizeof(*st));
    st->level = DIAGNOSTIC_STATUS_OK;
    snprintf(st->name, sizeof(st->name), "middleware");
    snprintf(st->message, sizeof(st->message), "best-effort only in this demo -- no retransmits possible");
    snprintf(st->hardware_id, sizeof(st->hardware_id), "rtos");
    snprintf(st->values[0].key, sizeof(st->values[0].key), "bytes_sent");
    snprintf(st->values[0].value, sizeof(st->values[0].value), "%llu", (unsigned long long)g_bytes_sent);
    st->values_count = 1;
}

static xrce_object_id_t g_diag_writer;
static xrce_object_id_t g_refresh_replier;
static task_t *g_producer_task;
static task_t *g_consumer_task;

static void publish_diagnostics(void) {
    diagnostic_msgs_DiagnosticArray arr = {0};
    add_task_status(&arr, g_producer_task);
    add_task_status(&arr, g_consumer_task);
    add_scheduler_status(&arr);
    add_queue_status(&arr, "work", &g_work_queue);
    add_middleware_status(&arr);

    uint8_t sample[2048];
    size_t sample_len;
    if (!diagnostic_msgs_DiagnosticArray_encode(&arr, sample, sizeof(sample), &sample_len)) {
        fprintf(stderr, "diagnostics encode failed\n");
        return;
    }
    uint8_t buf[2048];
    size_t len = xrce_session_build_write_data(&g_s, BEST_EFFORT_STREAM_0, g_diag_writer, sample + 4,
                                                sample_len - 4, buf, sizeof(buf));
    send_tracked(buf, len);
}

/* Handles a std_srvs/Trigger refresh request the same way
 * host/live_service_demo.c does -- see its header comment for the
 * SampleIdentity/header-less-sample details, not repeated here. */
static void handle_refresh_requests(void) {
    uint8_t in[256];
    ssize_t n = recv(g_fd, in, sizeof(in), MSG_DONTWAIT);
    if (n <= 0) {
        return;
    }
    xrce_object_id_t from_id;
    const uint8_t *sample;
    size_t sample_len;
    if (!xrce_session_parse_data(in, (size_t)n, &from_id, &sample, &sample_len) ||
        from_id.id != g_refresh_replier.id || from_id.kind != XRCE_OBJK_REPLIER ||
        sample_len < XRCE_SAMPLE_IDENTITY_SIZE) {
        return;
    }
    const uint8_t *sample_identity = sample;

    std_srvs_Trigger_Response resp = {.success = true};
    snprintf(resp.message, sizeof(resp.message), "refreshed");
    uint8_t resp_sample[128];
    size_t resp_sample_len;
    std_srvs_Trigger_Response_encode(&resp, resp_sample, sizeof(resp_sample), &resp_sample_len);

    uint8_t wire[XRCE_SAMPLE_IDENTITY_SIZE + 128];
    memcpy(wire, sample_identity, XRCE_SAMPLE_IDENTITY_SIZE);
    memcpy(wire + XRCE_SAMPLE_IDENTITY_SIZE, resp_sample + 4, resp_sample_len - 4);
    size_t wire_len = XRCE_SAMPLE_IDENTITY_SIZE + (resp_sample_len - 4);

    uint8_t buf[256];
    size_t len =
        xrce_session_build_write_data(&g_s, BEST_EFFORT_STREAM_0, g_refresh_replier, wire, wire_len, buf,
                                       sizeof(buf));
    send_tracked(buf, len);
    printf("refresh requested -- republishing\n");
    publish_diagnostics();
}

static void coordinator_task(void *arg) {
    (void)arg;
    for (;;) {
        handle_refresh_requests();
        task_sleep(50);
    }
}

static void publisher_task(void *arg) {
    (void)arg;
    for (;;) {
        publish_diagnostics();
        task_sleep(1000);
    }
}

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s <agent_ip> <agent_port>\n", argv[0]);
        return 2;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);

    g_fd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&g_addr, 0, sizeof(g_addr));
    g_addr.sin_family = AF_INET;
    g_addr.sin_port = htons((uint16_t)atoi(argv[2]));
    inet_pton(AF_INET, argv[1], &g_addr.sin_addr);
    struct timeval tv = {.tv_sec = 3, .tv_usec = 0};
    setsockopt(g_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t key[4] = {0x08, 0x08, 0x08, 0x08};
    xrce_session_init(&g_s, 0x01, key, 512);

    uint8_t buf[512];
    size_t len;

    len = xrce_session_build_create_client(&g_s, buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE_CLIENT", buf, len, xrce_session_parse_create_client_reply)) {
        return 1;
    }

    xrce_object_id_t participant_id = xrce_object_id(0x001, XRCE_OBJK_PARTICIPANT);
    len = xrce_session_build_create_participant(
        &g_s, BEST_EFFORT_STREAM_0, participant_id, 0,
        "<dds><participant><rtps><name>rtos_diagnostics</name></rtps></participant></dds>", buf,
        sizeof(buf));
    if (!send_and_wait_ok("CREATE participant", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t topic_id = xrce_object_id(0x001, XRCE_OBJK_TOPIC);
    len = xrce_session_build_create_xml(&g_s, BEST_EFFORT_STREAM_0, topic_id, participant_id,
                                         "<dds><topic><name>rt/rtos/diagnostics</name>"
                                         "<dataType>diagnostic_msgs::msg::dds_::DiagnosticArray_</dataType>"
                                         "</topic></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE topic", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t publisher_id = xrce_object_id(0x001, XRCE_OBJK_PUBLISHER);
    len = xrce_session_build_create_xml(&g_s, BEST_EFFORT_STREAM_0, publisher_id, participant_id, "", buf,
                                         sizeof(buf));
    if (!send_and_wait_ok("CREATE publisher", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    g_diag_writer = xrce_object_id(0x001, XRCE_OBJK_DATAWRITER);
    len = xrce_session_build_create_xml(
        &g_s, BEST_EFFORT_STREAM_0, g_diag_writer, publisher_id,
        "<dds><data_writer><topic><kind>NO_KEY</kind><name>rt/rtos/diagnostics</name>"
        "<dataType>diagnostic_msgs::msg::dds_::DiagnosticArray_</dataType>"
        "</topic></data_writer></dds>",
        buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE diagnostics datawriter", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    g_refresh_replier = xrce_object_id(0x001, XRCE_OBJK_REPLIER);
    len = xrce_session_build_create_xml(
        &g_s, BEST_EFFORT_STREAM_0, g_refresh_replier, participant_id,
        "<dds><replier profile_name=\"diag_refresh\" service_name=\"rtos/diagnostics/refresh\" "
        "request_type=\"std_srvs::srv::dds_::Trigger_Request_\" "
        "reply_type=\"std_srvs::srv::dds_::Trigger_Response_\">"
        "<request_topic_name>rq/rtos/diagnostics/refreshRequest</request_topic_name>"
        "<reply_topic_name>rr/rtos/diagnostics/refreshReply</reply_topic_name>"
        "</replier></dds>",
        buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE refresh replier", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }
    len = xrce_session_build_read_data(&g_s, BEST_EFFORT_STREAM_0, g_refresh_replier,
                                        BEST_EFFORT_STREAM_0, buf, sizeof(buf));
    send_tracked(buf, len);

    queue_init(&g_work_queue, g_work_queue_storage, QUEUE_CAPACITY);
    g_producer_task = task_spawn("producer", producer_worker, NULL, 20);
    g_consumer_task = task_spawn("consumer", consumer_worker, NULL, 10);
    task_spawn("coordinator", coordinator_task, NULL, 30);
    task_spawn("publisher", publisher_task, NULL, 5);

    printf("\nPublishing /rtos/diagnostics every second. Run, elsewhere:\n"
           "  ros2 topic echo /rtos/diagnostics diagnostic_msgs/msg/DiagnosticArray\n"
           "  ros2 service call /rtos/diagnostics/refresh std_srvs/srv/Trigger\n\n");

    scheduler_enable_preemption(5, 1);
    scheduler_run();
    scheduler_disable_preemption();
    return 0;
}
