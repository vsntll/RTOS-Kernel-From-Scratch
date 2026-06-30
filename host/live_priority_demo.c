/* Phase 7d deliverable: callback dispatch driven by the RTOS's own
 * priority scheduler, not a naive round-robin spin loop -- two real ROS2
 * subscriptions at different task priorities, under real load from a
 * host-side `ros2 topic pub -r`, showing the high-priority one's dispatch
 * latency stays bounded while the low-priority one visibly absorbs delay.
 *
 * xrce/ has no RTOS dependency by design (portable, host-native testable
 * on its own -- see xrce/docs/design.md) so the priority-aware dispatch
 * logic lives here, in host/, rather than as a new xrce/src/executor.c:
 * this file is where rtos/ and xrce/ get linked together for a demo,
 * exactly like rtos/arm/ros2_demo.c does for the QEMU firmware image,
 * without either library depending on the other.
 *
 * Architecture: one POLLER task drains the UDP socket with a NON-blocking
 * recv() and dispatches each DATA sample into the matching topic's
 * queue_t. This has to be non-blocking: every "task" here is a green
 * thread (ucontext_t) sharing the one real OS thread the RTOS's own
 * SIGALRM-driven preemption runs on (see rtos/src/scheduler.c) -- a
 * blocking recv() would freeze every other task, not just this one, since
 * there's no second real thread for them to run on. Two CONSUMER tasks
 * (high/low priority) block on queue_receive() for their own topic and
 * measure real dispatch latency (queue-arrival to callback-start). The
 * low-priority consumer does deliberately slow, non-yielding CPU work
 * per message so it only gets interrupted at real tick boundaries --
 * exactly the RTOS's real preemption behavior, not a scripted delay.
 *
 * A real bug, found only by testing against a live agent: the poller's
 * first version called task_yield() (not task_sleep()) between polls.
 * Since it's the highest-priority task and task_yield() only ever leaves
 * it READY (never BLOCKED), pick_next_ready() (scheduler.c) kept
 * re-selecting it every single cycle -- it's the only task that can ever
 * legitimately block it is the queue filling up (sem_wait() inside
 * queue_send()), so the high/low consumers should still eventually get a
 * turn once a queue fills, and did in later testing, but the resulting
 * multi-hundred-thousand-iterations-per-second spin loop was enough to
 * starve the real agent process of scheduling time on this host -- real
 * DATA that a working, unmodified `live_subscribe_demo.c` received
 * instantly at the exact same moment never arrived here at all (confirmed
 * via `errno=EAGAIN` on every poll, for as long as a minute). Switching
 * the poller to `task_sleep(1)` -- one real tick between polls instead of
 * a bare yield -- fixed it completely and is arguably more realistic
 * anyway: a real embedded poll loop doesn't spin as fast as the CPU
 * allows either.
 *
 * Usage (agent already running: `MicroXRCEAgent udp4 -p 8888`):
 *   gcc -Ixrce/include -Irtos/src host/live_priority_demo.c \
 *       rtos/build/librtos.a xrce/build/libxrce.a -o /tmp/live_priority_demo
 *   /tmp/live_priority_demo 127.0.0.1 8888
 *   # elsewhere, drive real load on both topics:
 *   ros2 topic pub -r 20 /priority_high std_msgs/msg/Int32 "{data: 1}" &
 *   ros2 topic pub -r 20 /priority_low std_msgs/msg/Int32 "{data: 1}" &
 */
#include <arpa/inet.h>
#include <fcntl.h>
#include <stdint.h>
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
#define MESSAGES_PER_TOPIC 15
#define QUEUE_CAPACITY 8

/* Tight preemption on purpose -- makes the priority-vs-load behavior this
 * demo exists to show visible within a run lasting a couple of real
 * seconds instead of needing a much longer one. */
#define TICK_MS 5
#define TICKS_PER_SLICE 1

static uint64_t now_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000 + (uint64_t)tv.tv_usec;
}

typedef struct {
    int32_t value;
    uint64_t enqueued_us;
} dispatch_item_t;

/* Fixed pools instead of malloc -- same "no dynamic allocation" pattern
 * xrce/msgs.h's fixed-size buffers already use elsewhere in this project. */
static dispatch_item_t g_high_pool[QUEUE_CAPACITY];
static dispatch_item_t g_low_pool[QUEUE_CAPACITY];
static int g_high_pool_next;
static int g_low_pool_next;

static void *g_high_queue_storage[QUEUE_CAPACITY];
static void *g_low_queue_storage[QUEUE_CAPACITY];
static queue_t g_high_queue;
static queue_t g_low_queue;

static int g_fd;
static struct sockaddr_in g_addr;
static xrce_object_id_t g_high_reader;
static xrce_object_id_t g_low_reader;

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
    printf("%-24s -> %s\n", step_name, ok ? "OK" : "FAILED");
    return ok;
}

/* Non-blocking -- see this file's header comment for why blocking would
 * freeze every task, not just this one. Parses at most one DATA
 * submessage per call and dispatches it into the matching topic's queue;
 * called from a tight loop in poller_task. */
static void poll_once(void) {
    uint8_t in[256];
    ssize_t n = recv(g_fd, in, sizeof(in), MSG_DONTWAIT);
    if (n <= 0) {
        return;
    }

    xrce_object_id_t from_id;
    const uint8_t *sample;
    size_t sample_len;
    if (!xrce_session_parse_data(in, (size_t)n, &from_id, &sample, &sample_len) || sample_len < 4) {
        return;
    }
    int32_t value = (int32_t)((uint32_t)sample[0] | ((uint32_t)sample[1] << 8) |
                               ((uint32_t)sample[2] << 16) | ((uint32_t)sample[3] << 24));

    if (from_id.id == g_high_reader.id && from_id.kind == XRCE_OBJK_DATAREADER) {
        dispatch_item_t *item = &g_high_pool[g_high_pool_next];
        g_high_pool_next = (g_high_pool_next + 1) % QUEUE_CAPACITY;
        item->value = value;
        item->enqueued_us = now_us();
        queue_send(&g_high_queue, item);
    } else if (from_id.id == g_low_reader.id && from_id.kind == XRCE_OBJK_DATAREADER) {
        dispatch_item_t *item = &g_low_pool[g_low_pool_next];
        g_low_pool_next = (g_low_pool_next + 1) % QUEUE_CAPACITY;
        item->value = value;
        item->enqueued_us = now_us();
        queue_send(&g_low_queue, item);
    }
}

/* scheduler_run() only returns once every task has reached
 * TASK_TERMINATED (scheduler.c's any_task_active()) -- an unconditionally
 * infinite poller would keep it running forever even after both consumers
 * finish their MESSAGES_PER_TOPIC quota, so it checks this flag instead. */
static volatile int g_high_done;
static volatile int g_low_done;

static void poller_task(void *arg) {
    (void)arg;
    while (!g_high_done || !g_low_done) {
        poll_once();
        task_sleep(1);
    }
}

/* Deliberately slow, non-yielding CPU work -- gets interrupted only at
 * real SIGALRM tick boundaries (scheduler.c's preempt_handler), which is
 * the actual thing being demonstrated: this task absorbs delay by being
 * preempted, not by choosing to step aside. */
static volatile uint64_t g_busy_sink;
static void simulate_slow_work(void) {
    for (uint64_t i = 0; i < 20000000; i++) {
        g_busy_sink += i;
    }
}

static void high_task(void *arg) {
    (void)arg;
    uint64_t total_latency_us = 0;
    uint64_t max_latency_us = 0;
    for (int i = 0; i < MESSAGES_PER_TOPIC; i++) {
        dispatch_item_t *item = queue_receive(&g_high_queue);
        uint64_t latency_us = now_us() - item->enqueued_us;
        total_latency_us += latency_us;
        if (latency_us > max_latency_us) {
            max_latency_us = latency_us;
        }
        printf("[high] value=%d dispatch_latency=%lluus\n", item->value,
               (unsigned long long)latency_us);
    }
    printf("[high] done: mean=%lluus max=%lluus\n",
           (unsigned long long)(total_latency_us / MESSAGES_PER_TOPIC),
           (unsigned long long)max_latency_us);
    g_high_done = 1;
}

static void low_task(void *arg) {
    (void)arg;
    uint64_t total_latency_us = 0;
    uint64_t max_latency_us = 0;
    for (int i = 0; i < MESSAGES_PER_TOPIC; i++) {
        dispatch_item_t *item = queue_receive(&g_low_queue);
        uint64_t latency_us = now_us() - item->enqueued_us;
        total_latency_us += latency_us;
        if (latency_us > max_latency_us) {
            max_latency_us = latency_us;
        }
        printf("[low]  value=%d dispatch_latency=%lluus (about to do slow work)\n", item->value,
               (unsigned long long)latency_us);
        simulate_slow_work();
    }
    printf("[low]  done: mean=%lluus max=%lluus\n",
           (unsigned long long)(total_latency_us / MESSAGES_PER_TOPIC),
           (unsigned long long)max_latency_us);
    g_low_done = 1;
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

    uint8_t key[4] = {0x7D, 0x7D, 0x7D, 0x7D};
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
        "<dds><participant><rtps><name>priority_demo</name></rtps></participant></dds>", buf,
        sizeof(buf));
    if (!send_and_wait_ok("CREATE participant", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t subscriber_id = xrce_object_id(0x001, XRCE_OBJK_SUBSCRIBER);
    len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, subscriber_id, participant_id, "", buf,
                                         sizeof(buf));
    if (!send_and_wait_ok("CREATE subscriber", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    const char *topic_names[2] = {"priority_high", "priority_low"};
    xrce_object_id_t topic_ids[2] = {xrce_object_id(0x001, XRCE_OBJK_TOPIC),
                                      xrce_object_id(0x002, XRCE_OBJK_TOPIC)};
    xrce_object_id_t reader_ids[2] = {xrce_object_id(0x001, XRCE_OBJK_DATAREADER),
                                       xrce_object_id(0x002, XRCE_OBJK_DATAREADER)};

    for (int i = 0; i < 2; i++) {
        char topic_xml[256];
        snprintf(topic_xml, sizeof(topic_xml),
                 "<dds><topic><name>rt/%s</name>"
                 "<dataType>std_msgs::msg::dds_::Int32_</dataType></topic></dds>",
                 topic_names[i]);
        len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, topic_ids[i], participant_id,
                                             topic_xml, buf, sizeof(buf));
        char step[32];
        snprintf(step, sizeof(step), "CREATE topic %d", i);
        if (!send_and_wait_ok(step, buf, len, xrce_session_parse_create_reply)) {
            return 1;
        }

        char dr_xml[384];
        snprintf(dr_xml, sizeof(dr_xml),
                 "<dds><data_reader><topic><kind>NO_KEY</kind><name>rt/%s</name>"
                 "<dataType>std_msgs::msg::dds_::Int32_</dataType></topic></data_reader></dds>",
                 topic_names[i]);
        len = xrce_session_build_create_xml(&s, BEST_EFFORT_STREAM_0, reader_ids[i], subscriber_id, dr_xml,
                                             buf, sizeof(buf));
        snprintf(step, sizeof(step), "CREATE datareader %d", i);
        if (!send_and_wait_ok(step, buf, len, xrce_session_parse_create_reply)) {
            return 1;
        }

        len = xrce_session_build_read_data(&s, BEST_EFFORT_STREAM_0, reader_ids[i], BEST_EFFORT_STREAM_0,
                                            buf, sizeof(buf));
        sendto(g_fd, buf, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr));
    }
    g_high_reader = reader_ids[0];
    g_low_reader = reader_ids[1];

    printf("\nSubscribed to rt/priority_high and rt/priority_low. Run, elsewhere:\n"
           "  ros2 topic pub -r 20 /priority_high std_msgs/msg/Int32 \"{data: 1}\"\n"
           "  ros2 topic pub -r 20 /priority_low std_msgs/msg/Int32 \"{data: 1}\"\n"
           "Waiting for %d messages on each topic...\n\n",
           MESSAGES_PER_TOPIC);

    queue_init(&g_high_queue, g_high_queue_storage, QUEUE_CAPACITY);
    queue_init(&g_low_queue, g_low_queue_storage, QUEUE_CAPACITY);

    /* Priorities: poller > high consumer > low consumer. The poller has
     * to run often enough to keep draining the socket regardless of which
     * consumer is busy; the high/low split is the thing under test. */
    task_spawn("poller", poller_task, NULL, 100);
    task_spawn("high", high_task, NULL, 50);
    task_spawn("low", low_task, NULL, 10);

    scheduler_enable_preemption(TICK_MS, TICKS_PER_SLICE);
    scheduler_run();
    scheduler_disable_preemption();
    return 0;
}
