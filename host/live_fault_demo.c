/* Phase 12 (fault injection test suite): "crash a task mid-publish, prove
 * the system degrades gracefully" -- the one fault category from the
 * project brief that's about the RTOS itself, not the network link, so it
 * lives here rather than in a host-side proxy script. Reuses Phase 8's
 * exact diagnostics harness (host/live_diagnostics_demo.c) unchanged --
 * same task-status/scheduler-status/queue-status builders, same
 * `/rtos/diagnostics` topic and refresh service -- with one added task,
 * `faulty_worker`, that runs several genuine publish-adjacent iterations
 * indistinguishable from a healthy task and then deliberately stomps its
 * own stack canary mid-iteration, simulating a real stack-overflow bug in
 * application code rather than a contrived test hook.
 *
 * What "degrades gracefully" means here, stated precisely rather than
 * oversold: this project's stack canary is a fail-*safe* design
 * (rtos/src/task.c's task_check_canary_or_abort(), checked on every
 * context switch) -- on corruption, the *entire process* halts with a
 * clear diagnostic, on purpose. That is not "other tasks keep running
 * while one crashes"; it is "corrupted state is caught and the system
 * stops cleanly before it can act on it, instead of silently continuing
 * with undefined behavior." Both are legitimate answers to "prove your
 * system degrades gracefully" (this project deliberately chose fail-safe
 * over fail-silent), but only one is what this demo actually shows, and
 * this comment says so plainly rather than let the more impressive-
 * sounding claim stand unchallenged.
 *
 * The graceful part that *is* demonstrated: the last `/rtos/diagnostics`
 * publish before the crash carries completely coherent data (every
 * task's real stats, correctly encoded) -- confirmed by
 * `ros2 topic echo /rtos/diagnostics --once` right up until the abort,
 * and confirmed absent (no more publishes, no corrupted/partial message
 * ever reaches the topic) afterward. See xrce/docs/design.md's Phase 12
 * section for the live transcript this was verified against.
 *
 * Usage (agent already running: `MicroXRCEAgent udp4 -p 8888`):
 *   gcc -Ixrce/include -Irtos/src host/live_fault_demo.c \
 *       rtos/build/librtos.a xrce/build/libxrce.a -o /tmp/live_fault_demo
 *   /tmp/live_fault_demo 127.0.0.1 8888
 *   # elsewhere, watch it happen live:
 *   ros2 topic echo /rtos/diagnostics diagnostic_msgs/msg/DiagnosticArray
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

/* Iterations the faulty task runs looking completely healthy before it
 * corrupts itself -- long enough that a human/CI watching `ros2 topic
 * echo` sees several genuine good readings from it first, not an
 * instant crash indistinguishable from a startup failure. */
#define FAULTY_ITERATIONS_BEFORE_CRASH 5

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

/* --- Worker tasks: one healthy, one that crashes on a timer --- */

static void *g_work_queue_storage[QUEUE_CAPACITY];
static queue_t g_work_queue;

static void healthy_worker(void *arg) {
    (void)arg;
    for (int i = 0;; i++) {
        queue_send(&g_work_queue, (void *)(intptr_t)i);
        task_sleep(200);
    }
}

static task_t *g_faulty_task; /* set right after task_spawn() below, so faulty_worker can reach its own task_t */

/* Looks exactly like a normal worker for its first
 * FAULTY_ITERATIONS_BEFORE_CRASH iterations (this is the point --
 * healthy behavior is what a real bug looks like right up until it
 * isn't), then deliberately overwrites its own canary bytes mid-iteration,
 * simulating a real stack-overflow bug in application code. The
 * corruption itself is silent (no crash *here*) -- the actual detection
 * and abort happens in scheduler.c's task_check_canary_or_abort(), the
 * next time this task yields control back (task_sleep() below), exactly
 * the same detection path a genuine unintentional overflow would hit. */
static void faulty_worker(void *arg) {
    (void)arg;
    for (int i = 0;; i++) {
        queue_receive(&g_work_queue);
        if (i == FAULTY_ITERATIONS_BEFORE_CRASH) {
            printf("[faulty_worker] iteration %d: deliberately corrupting my own stack canary "
                   "now (simulated overflow bug) -- next task_sleep() will trip the abort\n",
                   i);
            for (size_t j = 0; j < TASK_CANARY_SIZE; j++) {
                g_faulty_task->stack[j] = 0x00;
            }
        }
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
static task_t *g_healthy_task;

static void publish_diagnostics(void) {
    diagnostic_msgs_DiagnosticArray arr = {0};
    add_task_status(&arr, g_healthy_task);
    add_task_status(&arr, g_faulty_task);
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

    uint8_t key[4] = {0xFA, 0x01, 0x7A, 0x01}; /* distinct from Phase 8's demo key */
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
        "<dds><participant><rtps><name>rtos_fault_demo</name></rtps></participant></dds>", buf,
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
        "<dds><replier profile_name=\"fault_diag_refresh\" service_name=\"rtos/diagnostics/refresh\" "
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
    g_healthy_task = task_spawn("healthy", healthy_worker, NULL, 20);
    g_faulty_task = task_spawn("faulty", faulty_worker, NULL, 10);
    task_spawn("coordinator", coordinator_task, NULL, 30);
    task_spawn("publisher", publisher_task, NULL, 5);

    printf("\nPublishing /rtos/diagnostics every second. 'faulty' task will corrupt its own\n"
           "stack canary after %d iterations and the whole process will abort cleanly\n"
           "(FATAL message, not a segfault) -- watch, elsewhere:\n"
           "  ros2 topic echo /rtos/diagnostics diagnostic_msgs/msg/DiagnosticArray\n\n",
           FAULTY_ITERATIONS_BEFORE_CRASH);

    scheduler_enable_preemption(5, 1);
    scheduler_run();
    scheduler_disable_preemption();
    return 0;
}
