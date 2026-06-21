/* Manual, one-off demo (same reasoning as live_service_demo.c): a real
 * example_interfaces/action/Fibonacci action server answering a real,
 * unmodified `ros2 action send_goal` -- goal acceptance, incremental
 * feedback, cancellation, and a final result, all over the actual XRCE
 * wire protocol.
 *
 * A ROS2 action is, at the wire level, just three services + two topics
 * under a "<action>/_action/..." naming convention (design.ros2.org's
 * actions article) -- nothing new at the XRCE layer beyond what 7b's
 * services already built: three Repliers (send_goal, cancel_goal,
 * get_result) and two plain DataWriters (feedback, status). Service/topic
 * names and the request/reply header split
 * (rq<name>Request/rr<name>Reply/rt<name>) are ground-truthed the same way
 * 7b's Trigger replier was; message field layouts are copied from the real
 * generated ROS2 headers (see xrce/include/xrce/msgs.h's header comment).
 *
 * Single-threaded poll loop -- no RTOS task involvement yet (that's Phase
 * 7d's job): one iteration checks for any incoming request/cancel on a
 * short timeout, then advances the running goal's Fibonacci sequence by
 * one step if enough wall-clock time has passed.
 *
 * Usage (agent already running: `MicroXRCEAgent udp4 -p 8888`):
 *   gcc -Ixrce/include host/live_action_demo.c xrce/build/libxrce.a -o /tmp/live_action_demo
 *   /tmp/live_action_demo 127.0.0.1 8888
 *   # elsewhere: ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}" --feedback
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
#define STEP_INTERVAL_MS 500

/* action_msgs/msg/GoalStatus status values (ros2 interface show
 * action_msgs/msg/GoalStatus). */
#define GOAL_STATUS_ACCEPTED 1
#define GOAL_STATUS_EXECUTING 2
#define GOAL_STATUS_CANCELING 3
#define GOAL_STATUS_SUCCEEDED 4
#define GOAL_STATUS_CANCELED 5

static int g_fd;
static struct sockaddr_in g_addr;
static xrce_session_t g_s;

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

static void send_write_data(xrce_object_id_t writer_id, const uint8_t *payload_with_header,
                             size_t payload_len) {
    uint8_t buf[512];
    /* Strip our own 4-byte CDR header -- same convention as every other
     * WRITE_DATA in this project (the agent's generic dynamic type adds
     * its own; see xrce/docs/design.md's Phase 4 section). */
    size_t len = xrce_session_build_write_data(&g_s, BEST_EFFORT_STREAM_0, writer_id,
                                                payload_with_header + 4, payload_len - 4, buf, sizeof(buf));
    sendto(g_fd, buf, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr));
}

static void send_reply(xrce_object_id_t replier_id, const uint8_t *sample_identity,
                        const uint8_t *payload_with_header, size_t payload_len) {
    uint8_t wire[XRCE_SAMPLE_IDENTITY_SIZE + 512];
    memcpy(wire, sample_identity, XRCE_SAMPLE_IDENTITY_SIZE);
    memcpy(wire + XRCE_SAMPLE_IDENTITY_SIZE, payload_with_header + 4, payload_len - 4);
    size_t wire_len = XRCE_SAMPLE_IDENTITY_SIZE + (payload_len - 4);
    uint8_t buf[XRCE_SAMPLE_IDENTITY_SIZE + 512];
    size_t len =
        xrce_session_build_write_data(&g_s, BEST_EFFORT_STREAM_0, replier_id, wire, wire_len, buf, sizeof(buf));
    sendto(g_fd, buf, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr));
}

static uint64_t now_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
}

/* Goal state -- one goal at a time, matching this demo's single-threaded
 * poll loop (a real action server handles concurrent goals; not needed to
 * demonstrate the protocol working). */
static bool g_goal_active;
static bool g_goal_done;
static bool g_cancel_requested;
static uint8_t g_goal_id[XRCE_MSGS_UUID_SIZE];
static int32_t g_order;
static example_interfaces_Fibonacci_Feedback g_feedback;
static int8_t g_status;
static uint64_t g_last_step_ms;

/* A real ActionClient sends GetResult once, right after the goal is
 * accepted, and then just waits -- the server is expected to hold the
 * request and reply only once the goal reaches a terminal state, not have
 * the client poll/retry (confirmed empirically: `ros2 action send_goal`
 * never sent a second GetResult request when the first went unanswered). */
static bool g_result_pending;
static uint8_t g_result_sample_identity[XRCE_SAMPLE_IDENTITY_SIZE];

static void publish_status(xrce_object_id_t status_writer_id) {
    if (!g_goal_active) {
        return;
    }
    action_msgs_GoalStatusArray arr = {0};
    arr.status_list_count = 1;
    memcpy(arr.status_list[0].goal_info.goal_id, g_goal_id, XRCE_MSGS_UUID_SIZE);
    arr.status_list[0].status = g_status;
    uint8_t buf[256];
    size_t len;
    if (action_msgs_GoalStatusArray_encode(&arr, buf, sizeof(buf), &len)) {
        send_write_data(status_writer_id, buf, len);
    }
}

static void handle_send_goal(xrce_object_id_t replier_id, const uint8_t *sample_identity,
                              const uint8_t *request_bytes, size_t request_len,
                              xrce_object_id_t status_writer_id) {
    example_interfaces_Fibonacci_SendGoal_Request req;
    /* Received samples are header-less -- see host/live_service_demo.c's
     * comment for why *_decode() (which expects a header) can't be used
     * directly on wire data; parse the two fields by hand instead. */
    if (request_len < XRCE_MSGS_UUID_SIZE + 4) {
        return;
    }
    memcpy(req.goal_id, request_bytes, XRCE_MSGS_UUID_SIZE);
    req.order = (int32_t)((uint32_t)request_bytes[16] | ((uint32_t)request_bytes[17] << 8) |
                           ((uint32_t)request_bytes[18] << 16) | ((uint32_t)request_bytes[19] << 24));

    printf("send_goal request: order=%d\n", req.order);

    bool can_accept = !g_goal_active || g_goal_done;
    example_interfaces_Fibonacci_SendGoal_Response resp = {.accepted = can_accept};
    uint8_t resp_buf[64];
    size_t resp_len;
    example_interfaces_Fibonacci_SendGoal_Response_encode(&resp, resp_buf, sizeof(resp_buf), &resp_len);
    send_reply(replier_id, sample_identity, resp_buf, resp_len);

    if (!can_accept) {
        printf("send_goal rejected: a goal is already running\n");
        return;
    }

    g_goal_active = true;
    g_goal_done = false;
    g_cancel_requested = false;
    memcpy(g_goal_id, req.goal_id, XRCE_MSGS_UUID_SIZE);
    g_order = req.order;
    g_feedback.sequence_count = 0;
    g_status = GOAL_STATUS_ACCEPTED;
    g_last_step_ms = now_ms();
    publish_status(status_writer_id);
    printf("goal accepted, running Fibonacci(%d)\n", g_order);
}

static void handle_cancel_goal(xrce_object_id_t replier_id, const uint8_t *sample_identity,
                                const uint8_t *request_bytes, size_t request_len,
                                xrce_object_id_t status_writer_id) {
    (void)request_bytes;
    if (request_len < XRCE_MSGS_UUID_SIZE) {
        return;
    }
    /* This demo only ever runs one goal at a time, so any cancel request
     * (specific goal_id or the "cancel everything" all-zero id) cancels
     * whatever's active -- a real multi-goal server would filter by id. */
    action_msgs_CancelGoal_Response resp = {0};
    if (g_goal_active && !g_goal_done) {
        resp.return_code = 0; /* ERROR_NONE */
        resp.goals_canceling_count = 1;
        memcpy(resp.goals_canceling[0].goal_id, g_goal_id, XRCE_MSGS_UUID_SIZE);
        g_cancel_requested = true;
        g_status = GOAL_STATUS_CANCELING;
        publish_status(status_writer_id);
        printf("cancel request accepted\n");
    } else {
        resp.return_code = 2; /* ERROR_UNKNOWN_GOAL_ID */
        printf("cancel request rejected: no active goal\n");
    }
    uint8_t resp_buf[256];
    size_t resp_len;
    action_msgs_CancelGoal_Response_encode(&resp, resp_buf, sizeof(resp_buf), &resp_len);
    send_reply(replier_id, sample_identity, resp_buf, resp_len);
}

static void reply_result(xrce_object_id_t replier_id, const uint8_t *sample_identity) {
    example_interfaces_Fibonacci_GetResult_Response resp = {.status = g_status};
    resp.result = g_feedback; /* same {sequence, sequence_count} shape */
    uint8_t resp_buf[256];
    size_t resp_len;
    example_interfaces_Fibonacci_GetResult_Response_encode(&resp, resp_buf, sizeof(resp_buf), &resp_len);
    send_reply(replier_id, sample_identity, resp_buf, resp_len);
    printf("get_result replied: status=%d, %u values\n", resp.status, resp.result.sequence_count);
}

static void handle_get_result(xrce_object_id_t replier_id, const uint8_t *sample_identity,
                               const uint8_t *request_bytes, size_t request_len) {
    (void)request_bytes;
    if (request_len < XRCE_MSGS_UUID_SIZE) {
        return;
    }
    if (!g_goal_done) {
        /* Hold the request -- a real ActionClient sends GetResult once,
         * right after goal acceptance, and expects the server to reply
         * only once the goal reaches a terminal state (confirmed
         * empirically: it never retries). */
        g_result_pending = true;
        memcpy(g_result_sample_identity, sample_identity, XRCE_SAMPLE_IDENTITY_SIZE);
        printf("get_result request received, goal not finished yet -- holding until it is\n");
        return;
    }
    reply_result(replier_id, sample_identity);
}

static void maybe_reply_pending_result(xrce_object_id_t get_result_replier_id) {
    if (g_result_pending && g_goal_done) {
        reply_result(get_result_replier_id, g_result_sample_identity);
        g_result_pending = false;
    }
}

static void step_goal(xrce_object_id_t feedback_writer_id, xrce_object_id_t status_writer_id,
                       xrce_object_id_t get_result_replier_id) {
    if (!g_goal_active || g_goal_done) {
        return;
    }
    if (now_ms() - g_last_step_ms < STEP_INTERVAL_MS) {
        return;
    }
    g_last_step_ms = now_ms();

    if (g_cancel_requested) {
        g_status = GOAL_STATUS_CANCELED;
        g_goal_done = true;
        publish_status(status_writer_id);
        printf("goal canceled after %u values\n", g_feedback.sequence_count);
        maybe_reply_pending_result(get_result_replier_id);
        return;
    }

    int32_t next = (g_feedback.sequence_count < 2) ? (int32_t)g_feedback.sequence_count
                                                     : g_feedback.sequence[g_feedback.sequence_count - 1] +
                                                           g_feedback.sequence[g_feedback.sequence_count - 2];
    g_feedback.sequence[g_feedback.sequence_count++] = next;

    example_interfaces_Fibonacci_FeedbackMessage fb = {0};
    memcpy(fb.goal_id, g_goal_id, XRCE_MSGS_UUID_SIZE);
    fb.feedback = g_feedback;
    uint8_t buf[256];
    size_t len;
    if (example_interfaces_Fibonacci_FeedbackMessage_encode(&fb, buf, sizeof(buf), &len)) {
        send_write_data(feedback_writer_id, buf, len);
    }
    printf("feedback: %u values, last=%d\n", g_feedback.sequence_count, next);

    g_status = GOAL_STATUS_EXECUTING;
    publish_status(status_writer_id);

    if (g_feedback.sequence_count >= (uint32_t)g_order || g_feedback.sequence_count >= XRCE_MSGS_SEQ_I32_MAX) {
        g_status = GOAL_STATUS_SUCCEEDED;
        g_goal_done = true;
        publish_status(status_writer_id);
        printf("goal succeeded: %u values\n", g_feedback.sequence_count);
        maybe_reply_pending_result(get_result_replier_id);
    }
}

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s <agent_ip> <agent_port>\n", argv[0]);
        return 2;
    }
    setvbuf(stdout, NULL, _IOLBF, 0); /* line-buffer even when piped to a log file */

    g_fd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&g_addr, 0, sizeof(g_addr));
    g_addr.sin_family = AF_INET;
    g_addr.sin_port = htons((uint16_t)atoi(argv[2]));
    inet_pton(AF_INET, argv[1], &g_addr.sin_addr);
    struct timeval tv = {.tv_sec = 0, .tv_usec = 200 * 1000};
    setsockopt(g_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t key[4] = {0xAC, 0x71, 0x0A, 0x01};
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
        "<dds><participant><rtps><name>rtos_action_demo</name></rtps></participant></dds>", buf,
        sizeof(buf));
    if (!send_and_wait_ok("CREATE participant", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    /* Three Repliers -- send_goal/cancel_goal/get_result -- each named per
     * ROS2's own "<action>/_action/<verb>" convention
     * (design.ros2.org/articles/actions.html), mangled into rq/rr DDS
     * topic names exactly like 7b's plain Trigger replier. */
    xrce_object_id_t send_goal_replier = xrce_object_id(0x001, XRCE_OBJK_REPLIER);
    len = xrce_session_build_create_xml(
        &g_s, BEST_EFFORT_STREAM_0, send_goal_replier, participant_id,
        "<dds><replier profile_name=\"fib_send_goal\" service_name=\"fibonacci/_action/send_goal\" "
        "request_type=\"example_interfaces::action::dds_::Fibonacci_SendGoal_Request_\" "
        "reply_type=\"example_interfaces::action::dds_::Fibonacci_SendGoal_Response_\">"
        "<request_topic_name>rq/fibonacci/_action/send_goalRequest</request_topic_name>"
        "<reply_topic_name>rr/fibonacci/_action/send_goalReply</reply_topic_name>"
        "</replier></dds>",
        buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE send_goal replier", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t cancel_goal_replier = xrce_object_id(0x002, XRCE_OBJK_REPLIER);
    len = xrce_session_build_create_xml(
        &g_s, BEST_EFFORT_STREAM_0, cancel_goal_replier, participant_id,
        "<dds><replier profile_name=\"fib_cancel_goal\" service_name=\"fibonacci/_action/cancel_goal\" "
        "request_type=\"action_msgs::srv::dds_::CancelGoal_Request_\" "
        "reply_type=\"action_msgs::srv::dds_::CancelGoal_Response_\">"
        "<request_topic_name>rq/fibonacci/_action/cancel_goalRequest</request_topic_name>"
        "<reply_topic_name>rr/fibonacci/_action/cancel_goalReply</reply_topic_name>"
        "</replier></dds>",
        buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE cancel_goal replier", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t get_result_replier = xrce_object_id(0x003, XRCE_OBJK_REPLIER);
    len = xrce_session_build_create_xml(
        &g_s, BEST_EFFORT_STREAM_0, get_result_replier, participant_id,
        "<dds><replier profile_name=\"fib_get_result\" service_name=\"fibonacci/_action/get_result\" "
        "request_type=\"example_interfaces::action::dds_::Fibonacci_GetResult_Request_\" "
        "reply_type=\"example_interfaces::action::dds_::Fibonacci_GetResult_Response_\">"
        "<request_topic_name>rq/fibonacci/_action/get_resultRequest</request_topic_name>"
        "<reply_topic_name>rr/fibonacci/_action/get_resultReply</reply_topic_name>"
        "</replier></dds>",
        buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE get_result replier", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    /* Plain topics -- feedback and status -- exactly like Phase 4's
     * publisher, just with action-shaped types and "_action/..." names. */
    xrce_object_id_t publisher_id = xrce_object_id(0x001, XRCE_OBJK_PUBLISHER);
    len = xrce_session_build_create_xml(&g_s, BEST_EFFORT_STREAM_0, publisher_id, participant_id, "", buf,
                                         sizeof(buf));
    if (!send_and_wait_ok("CREATE publisher", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t feedback_topic = xrce_object_id(0x001, XRCE_OBJK_TOPIC);
    len = xrce_session_build_create_xml(&g_s, BEST_EFFORT_STREAM_0, feedback_topic, participant_id,
                                         "<dds><topic><name>rt/fibonacci/_action/feedback</name>"
                                         "<dataType>example_interfaces::action::dds_::"
                                         "Fibonacci_FeedbackMessage_</dataType></topic></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE feedback topic", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }
    xrce_object_id_t feedback_writer = xrce_object_id(0x001, XRCE_OBJK_DATAWRITER);
    len = xrce_session_build_create_xml(&g_s, BEST_EFFORT_STREAM_0, feedback_writer, publisher_id,
                                         "<dds><data_writer><topic><kind>NO_KEY</kind>"
                                         "<name>rt/fibonacci/_action/feedback</name>"
                                         "<dataType>example_interfaces::action::dds_::"
                                         "Fibonacci_FeedbackMessage_</dataType></topic></data_writer></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE feedback datawriter", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    xrce_object_id_t status_topic = xrce_object_id(0x002, XRCE_OBJK_TOPIC);
    len = xrce_session_build_create_xml(&g_s, BEST_EFFORT_STREAM_0, status_topic, participant_id,
                                         "<dds><topic><name>rt/fibonacci/_action/status</name>"
                                         "<dataType>action_msgs::msg::dds_::GoalStatusArray_</dataType>"
                                         "</topic></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE status topic", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }
    xrce_object_id_t status_writer = xrce_object_id(0x002, XRCE_OBJK_DATAWRITER);
    len = xrce_session_build_create_xml(&g_s, BEST_EFFORT_STREAM_0, status_writer, publisher_id,
                                         "<dds><data_writer><topic><kind>NO_KEY</kind>"
                                         "<name>rt/fibonacci/_action/status</name>"
                                         "<dataType>action_msgs::msg::dds_::GoalStatusArray_</dataType>"
                                         "</topic></data_writer></dds>",
                                         buf, sizeof(buf));
    if (!send_and_wait_ok("CREATE status datawriter", buf, len, xrce_session_parse_create_reply)) {
        return 1;
    }

    /* READ_DATA on all three repliers so incoming requests actually get
     * delivered back to us as DATA submessages (see session.h/READ_DATA's
     * header comment -- omitting delivery_control silently one-shots). */
    xrce_object_id_t repliers[3] = {send_goal_replier, cancel_goal_replier, get_result_replier};
    for (int i = 0; i < 3; i++) {
        len = xrce_session_build_read_data(&g_s, BEST_EFFORT_STREAM_0, repliers[i], BEST_EFFORT_STREAM_0,
                                            buf, sizeof(buf));
        sendto(g_fd, buf, len, 0, (struct sockaddr *)&g_addr, sizeof(g_addr));
    }

    printf("\nServing example_interfaces/action/Fibonacci on /fibonacci -- run\n"
           "`ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "
           "\"{order: 5}\" --feedback` now. Ctrl-C to stop.\n\n");

    for (;;) {
        uint8_t in[512];
        ssize_t n = recv(g_fd, in, sizeof(in), 0);
        if (n > 0) {
            xrce_object_id_t from_id;
            const uint8_t *sample;
            size_t sample_len;
            if (xrce_session_parse_data(in, (size_t)n, &from_id, &sample, &sample_len) &&
                sample_len >= XRCE_SAMPLE_IDENTITY_SIZE) {
                const uint8_t *sample_identity = sample;
                const uint8_t *req_bytes = sample + XRCE_SAMPLE_IDENTITY_SIZE;
                size_t req_len = sample_len - XRCE_SAMPLE_IDENTITY_SIZE;

                if (from_id.id == send_goal_replier.id && from_id.kind == XRCE_OBJK_REPLIER) {
                    handle_send_goal(send_goal_replier, sample_identity, req_bytes, req_len, status_writer);
                } else if (from_id.id == cancel_goal_replier.id && from_id.kind == XRCE_OBJK_REPLIER) {
                    handle_cancel_goal(cancel_goal_replier, sample_identity, req_bytes, req_len,
                                        status_writer);
                } else if (from_id.id == get_result_replier.id && from_id.kind == XRCE_OBJK_REPLIER) {
                    handle_get_result(get_result_replier, sample_identity, req_bytes, req_len);
                }
            }
        }
        step_goal(feedback_writer, status_writer, get_result_replier);
    }
}
