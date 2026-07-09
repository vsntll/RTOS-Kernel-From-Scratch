/* Phase 14: this RTOS firmware is the control software for a simulated
 * vehicle with real physics, not a data-flows-correctly demo. Publishes
 * geometry_msgs/Twist on /model/vehicle_blue/cmd_vel -- the exact topic
 * name and model gz-sim's own bundled `diff_drive.sdf` world documents in
 * its own header comment as the way to drive it -- bridged into Gazebo's
 * own transport by a real, unmodified `ros_gz_bridge parameter_bridge`,
 * same "real, unmodified" standard this project holds the XRCE-DDS agent
 * to. See xrce/docs/design.md's Phase 14 section for the live verification
 * (the vehicle's own odometry, not a screenshot, confirms it actually
 * moved by the commanded amount).
 *
 * Reuses ros2_demo.c's session/framing/pacing machinery (this file is
 * deliberately not a from-scratch reimplementation of any of that) but is
 * publish-only -- no subscription, no echo -- since driving a simulated
 * vehicle from a fixed motion pattern doesn't need a command channel back
 * in, unlike rt/setpoint's "host sends a command" scenario. NODE_ID/
 * SECURE_LINK (Phases 10/11) apply here unchanged: this is still the same
 * xrce/ session layer over the same serial_transport.
 *
 * Motion pattern: drive straight, then turn, repeating -- not a single
 * constant velocity -- specifically so a live odometry check can confirm
 * the vehicle's heading actually changed (angular.z was obeyed) as well as
 * its position (linear.x was obeyed), rather than only ever exercising
 * one axis. */

#include <stdint.h>

#include "uart.h"
#include "xrce/msgs.h"
#include "xrce/secure_transport.h"
#include "xrce/serial_transport.h"
#include "xrce/session.h"

#ifndef NODE_ID
#define NODE_ID 0
#endif
#ifndef SECURE_LINK
#define SECURE_LINK 0
#endif

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define NODE_ID_STR TOSTRING(NODE_ID)
#if NODE_ID == 0
#define NODE_SUFFIX ""
#else
#define NODE_SUFFIX "_" NODE_ID_STR
#endif

#define SERIAL_LOCAL_ADDR 0x00
#define SERIAL_REMOTE_ADDR 0x00
#define BEST_EFFORT_STREAM_0 1

/* How many publishes to hold each phase of the drive-then-turn pattern --
 * arbitrary but long enough to produce a visible/measurable pose change
 * in the odometry check, short enough that a live verification run
 * doesn't need to sit around for minutes. */
#define PUBLISHES_PER_PHASE 40
/* 5000 (ros2_demo.c's own constant) measured at ~4800 Hz here even with
 * the MMIO-touching pace() below -- this file's per-iteration cost is
 * apparently lighter than ros2_demo.c's own loop, so the same constant
 * doesn't carry over. Bumped by 40x and re-measured live (see pace()'s
 * comment) rather than assumed. */
#define PACE_BUSY_LOOPS 200000

#if SECURE_LINK
static const uint8_t g_secure_key[16] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE,
                                          0x13, 0x37, 0xC0, 0xDE, 0xF0, 0x0D, 0x5E, 0xED};
static xrce_secure_ctx_t g_uplink_ctx;
#endif

static void uart_put_uint(uint32_t v) {
    char digits[10];
    int n = 0;
    if (v == 0) {
        uart_putc('0');
        return;
    }
    while (v > 0 && n < 10) {
        digits[n++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (n > 0) {
        uart_putc(digits[--n]);
    }
}

#define SEND_FRAME_BUF_LEN XRCE_SERIAL_MAX_ENCODED_LEN(256 + XRCE_SECURE_OVERHEAD)

static void send_frame(const uint8_t *payload, size_t len) {
#if SECURE_LINK
    uint8_t wrapped[256 + XRCE_SECURE_OVERHEAD];
    size_t wrapped_len = xrce_secure_wrap(&g_uplink_ctx, payload, len, wrapped, sizeof(wrapped));
    if (wrapped_len == 0) {
        uart_puts("ERROR: secure wrap failed\r\n");
        return;
    }
    payload = wrapped;
    len = wrapped_len;
#endif
    uint8_t frame[SEND_FRAME_BUF_LEN];
    size_t frame_len = xrce_serial_frame_encode(frame, sizeof(frame), SERIAL_LOCAL_ADDR,
                                                 SERIAL_REMOTE_ADDR, payload, len);
    if (frame_len == 0) {
        uart_puts("ERROR: send_frame buffer too small for this payload\r\n");
        return;
    }
    for (size_t i = 0; i < frame_len; i++) {
        uart_putc((char)frame[i]);
    }
}

/* A pure register-only busy loop (the first version of this function) ran
 * at roughly 4800 Hz under QEMU's TCG JIT with PACE_BUSY_LOOPS=5000 --
 * confirmed live via `ros2 topic hz /model/vehicle_blue/cmd_vel` -- the
 * same class of "QEMU's core clock is uncalibrated" surprise this
 * project's docs already flag elsewhere, sharpened here: a loop with NO
 * memory access at all is dramatically faster than ros2_demo.c's own
 * pacing loop, which happens to poll a real (emulated) MMIO register each
 * iteration and is therefore realistically slow. Reusing that same
 * MMIO-touching poll here (its return value is simply discarded -- this
 * firmware never reads a downlink command) throttles this loop to the
 * same real-world pace ros2_demo.c already runs at, rather than
 * calibrating a magic iteration count that would only be valid for this
 * exact QEMU version/host. */
static void pace(int n) {
    for (volatile int i = 0; i < n; i++) {
        (void)uart_getc_nonblocking();
    }
}

static xrce_object_id_t g_datawriter_id;

static void announce_entities(xrce_session_t *session) {
    /* 192 wasn't enough here -- the exact same class of bug ros2_demo.c's
     * own header comment already documents: the datawriter CREATE's XML
     * wraps this file's longer "rt/model/vehicle_blue/cmd_vel" topic name
     * in extra <data_writer><topic>...</topic></data_writer> tags on top
     * of what CREATE topic itself needs, pushing it past 192 bytes.
     * xrce_session_build_create_xml() fails safe (returns 0) on overflow
     * rather than corrupting memory, but that meant this specific message
     * was silently never sent -- caught the same way ros2_demo.c's
     * original instance of this bug was: the datawriter never showing up
     * in the agent's log despite participant/topic/publisher all
     * succeeding. */
    uint8_t msg[256];
    size_t len;

    uart_puts("-> CREATE_CLIENT\r\n");
    len = xrce_session_build_create_client(session, msg, sizeof(msg));
    send_frame(msg, len);
    pace(PACE_BUSY_LOOPS);

    xrce_object_id_t participant_id = xrce_object_id(0x001, XRCE_OBJK_PARTICIPANT);
    uart_puts("-> CREATE participant\r\n");
    len = xrce_session_build_create_participant(
        session, BEST_EFFORT_STREAM_0, participant_id, 0,
        "<dds><participant><rtps><name>rtos_gazebo_demo" NODE_SUFFIX
        "</name></rtps></participant></dds>",
        msg, sizeof(msg));
    send_frame(msg, len);
    pace(PACE_BUSY_LOOPS);

    xrce_object_id_t topic_id = xrce_object_id(0x001, XRCE_OBJK_TOPIC);
    uart_puts("-> CREATE topic (cmd_vel)\r\n");
    len = xrce_session_build_create_xml(
        session, BEST_EFFORT_STREAM_0, topic_id, participant_id,
        "<dds><topic><name>rt/model/vehicle_blue" NODE_SUFFIX "/cmd_vel</name>"
        "<dataType>geometry_msgs::msg::dds_::Twist_</dataType></topic></dds>",
        msg, sizeof(msg));
    send_frame(msg, len);
    pace(PACE_BUSY_LOOPS);

    xrce_object_id_t publisher_id = xrce_object_id(0x001, XRCE_OBJK_PUBLISHER);
    uart_puts("-> CREATE publisher\r\n");
    len = xrce_session_build_create_xml(session, BEST_EFFORT_STREAM_0, publisher_id,
                                         participant_id, "", msg, sizeof(msg));
    send_frame(msg, len);
    pace(PACE_BUSY_LOOPS);

    g_datawriter_id = xrce_object_id(0x001, XRCE_OBJK_DATAWRITER);
    uart_puts("-> CREATE datawriter (cmd_vel)\r\n");
    len = xrce_session_build_create_xml(
        session, BEST_EFFORT_STREAM_0, g_datawriter_id, publisher_id,
        "<dds><data_writer><topic><kind>NO_KEY</kind>"
        "<name>rt/model/vehicle_blue" NODE_SUFFIX "/cmd_vel</name>"
        "<dataType>geometry_msgs::msg::dds_::Twist_</dataType>"
        "</topic></data_writer></dds>",
        msg, sizeof(msg));
    send_frame(msg, len);
    pace(PACE_BUSY_LOOPS);
}

int main(void) {
    uart_init();
    uart_puts("RTOS Gazebo demo: booting, driving vehicle_blue's diff-drive plugin\r\n");

    uint8_t key[4] = {0x67, 0x7A, 0x62, (uint8_t)(0x30 + NODE_ID)}; /* "gzb0" + NODE_ID */
    xrce_session_t session;
    xrce_session_init(&session, 0x01, key, 512);
#if SECURE_LINK
    xrce_secure_init(&g_uplink_ctx, g_secure_key, sizeof(g_secure_key));
    uart_puts("SECURE_LINK enabled: every frame HMAC-authenticated\r\n");
#endif

    announce_entities(&session);

    uart_puts("Driving: straight, then turn, repeating...\r\n");
    for (uint32_t i = 0;; i++) {
        if (i > 0 && (i % (PUBLISHES_PER_PHASE * 8)) == 0) {
            announce_entities(&session);
        }

        uint32_t phase = (i / PUBLISHES_PER_PHASE) % 2;
        geometry_msgs_Twist cmd = {0};
        if (phase == 0) {
            cmd.linear.x = 0.5;
            cmd.angular.z = 0.0;
        } else {
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.4;
        }

        uint8_t sample[64];
        size_t sample_len;
        if (!geometry_msgs_Twist_encode(&cmd, sample, sizeof(sample), &sample_len)) {
            uart_puts("ERROR: Twist encode failed\r\n");
            for (;;) {
            }
        }
        /* Strip our own 4-byte CDR encapsulation header -- same reason as
         * ros2_demo.c's rt/chatter publish: the agent's generic topic type
         * prepends its own, so sending ours too double-headers the sample. */
        uint8_t msg[96];
        size_t len = xrce_session_build_write_data(&session, BEST_EFFORT_STREAM_0, g_datawriter_id,
                                                     sample + 4, sample_len - 4, msg, sizeof(msg));
        send_frame(msg, len);
        uart_puts("cmd_vel phase=");
        uart_put_uint(phase);
        uart_puts("\r\n");
        pace(PACE_BUSY_LOOPS);
    }
}
