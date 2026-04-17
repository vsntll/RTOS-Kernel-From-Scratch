/* Phase 4 (ROS2 layer): the actual "RTOS task in QEMU talks to a real
 * agent" deliverable -- runs the same xrce/ session layer already verified
 * over UDP (host/live_publish_demo.c) here instead over UART1, framed with
 * xrce/src/serial_transport.c, so the bytes go QEMU -serial -> a real
 * MicroXRCEAgent's `serial` transport instead of a raw UDP socket.
 *
 * No RTOS task/scheduler involvement here -- this is a plain sequential
 * main(), like Phase 1/3's demos before Phase 8 added preemption; SysTick
 * is never started, so kernel_arm.c/context_switch.s are linked in only
 * to satisfy startup.s's vector table (PendSV_Handler/SysTick_Handler
 * symbols), not actually exercised. Wiring a real RTOS task around this
 * (blocking on a queue for outgoing samples, matching Phase 3's original
 * "publish task" framing) is a reasonable next step, not this one --
 * this demo's job is just proving the transport, not re-litigating the
 * scheduler.
 *
 * Serial addressing: local_addr/remote_addr both 0x00, matching the
 * common default used across eProsima's own client examples when neither
 * side customizes it.
 *
 * CREATE_CLIENT + entity creation is re-announced every few publishes
 * rather than sent once at boot, with no on-device reply parsing (no UART
 * RX here -- see the header comment above): QEMU boots and starts sending
 * within a fraction of a real second, but the host-side agent process
 * takes a few real seconds to attach to the pty/serial device, so a
 * send-once handshake at boot reliably loses the race and is never seen.
 * Re-announcing periodically means whenever the agent does attach, it
 * catches a fresh handshake within a bounded time -- duplicate CREATEs
 * for already-existing entities are harmless (the agent just logs
 * ALREADY_EXISTS and moves on), which is a fine tradeoff for a demo. */

#include <stdint.h>

#include "uart.h"
#include "xrce/msgs.h"
#include "xrce/serial_transport.h"
#include "xrce/session.h"

#define SERIAL_LOCAL_ADDR 0x00
#define SERIAL_REMOTE_ADDR 0x00
#define BEST_EFFORT_STREAM_0 1
#define REANNOUNCE_EVERY_N_PUBLISHES 5

static void busy_wait_ticks(int n) {
    for (volatile int i = 0; i < n; i++) {
    }
}

/* Unverified real-world duration under QEMU TCG emulation (see main.c's
 * SYSTICK_RELOAD comment for the same caveat) -- observed to run much
 * faster than a real second in practice. Only paces UART output for a
 * human watching the terminal; periodic re-announcement above is what
 * actually makes this demo's correctness independent of this value. */
#define PACE_BUSY_LOOPS 900000

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

/* Sized for the largest message this demo ever sends (datawriter CREATE,
 * whose XML string is the longest), worst-case-stuffed. A previous
 * version of this function hardcoded XRCE_SERIAL_MAX_ENCODED_LEN(64) --
 * enough for CREATE_CLIENT and the empty-XML publisher CREATE, too small
 * for topic/datawriter CREATE's ~100+ byte XML payloads. xrce_serial_frame_encode()
 * fails safe (returns 0) on overflow rather than corrupting memory, but
 * that meant those two messages were silently never sent at all -- caught
 * by the datawriter never showing up in the agent's log despite
 * participant/topic/publisher all succeeding. */
#define SEND_FRAME_BUF_LEN XRCE_SERIAL_MAX_ENCODED_LEN(256)

static void send_frame(const uint8_t *payload, size_t len) {
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

static xrce_object_id_t g_participant_id;
static xrce_object_id_t g_topic_id;
static xrce_object_id_t g_publisher_id;
static xrce_object_id_t g_datawriter_id;

static void announce_entities(xrce_session_t *session) {
    uint8_t msg[192];
    size_t len;

    uart_puts("-> CREATE_CLIENT\r\n");
    len = xrce_session_build_create_client(session, msg, sizeof(msg));
    send_frame(msg, len);
    busy_wait_ticks(PACE_BUSY_LOOPS);

    g_participant_id = xrce_object_id(0x001, XRCE_OBJK_PARTICIPANT);
    uart_puts("-> CREATE participant\r\n");
    len = xrce_session_build_create_participant(
        session, BEST_EFFORT_STREAM_0, g_participant_id, 0,
        "<dds><participant><rtps><name>rtos_qemu_demo</name></rtps></participant></dds>", msg,
        sizeof(msg));
    send_frame(msg, len);
    busy_wait_ticks(PACE_BUSY_LOOPS);

    g_topic_id = xrce_object_id(0x001, XRCE_OBJK_TOPIC);
    uart_puts("-> CREATE topic\r\n");
    len = xrce_session_build_create_xml(session, BEST_EFFORT_STREAM_0, g_topic_id,
                                         g_participant_id,
                                         "<dds><topic><name>rt/chatter</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType></topic></dds>",
                                         msg, sizeof(msg));
    send_frame(msg, len);
    busy_wait_ticks(PACE_BUSY_LOOPS);

    g_publisher_id = xrce_object_id(0x001, XRCE_OBJK_PUBLISHER);
    uart_puts("-> CREATE publisher\r\n");
    len = xrce_session_build_create_xml(session, BEST_EFFORT_STREAM_0, g_publisher_id,
                                         g_participant_id, "", msg, sizeof(msg));
    send_frame(msg, len);
    busy_wait_ticks(PACE_BUSY_LOOPS);

    g_datawriter_id = xrce_object_id(0x001, XRCE_OBJK_DATAWRITER);
    uart_puts("-> CREATE datawriter\r\n");
    len = xrce_session_build_create_xml(session, BEST_EFFORT_STREAM_0, g_datawriter_id,
                                         g_publisher_id,
                                         "<dds><data_writer><topic><kind>NO_KEY</kind>"
                                         "<name>rt/chatter</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType>"
                                         "</topic></data_writer></dds>",
                                         msg, sizeof(msg));
    send_frame(msg, len);
    busy_wait_ticks(PACE_BUSY_LOOPS);
}

int main(void) {
    uart_init();
    uart_puts("RTOS ROS2 demo: booting, sending XRCE frames over UART1\r\n");

    uint8_t key[4] = {0x52, 0x54, 0x4F, 0x53}; /* "RTOS" */
    xrce_session_t session;
    xrce_session_init(&session, 0x01, key, 128);

    announce_entities(&session);

    uart_puts("Publishing std_msgs/Int32 on rt/chatter...\r\n");
    for (int32_t i = 0;; i++) {
        if (i > 0 && (i % REANNOUNCE_EVERY_N_PUBLISHES) == 0) {
            announce_entities(&session);
        }

        std_msgs_Int32 sample_msg = {.data = i};
        uint8_t sample[16];
        size_t sample_len;
        if (!std_msgs_Int32_encode(&sample_msg, sample, sizeof(sample), &sample_len)) {
            uart_puts("ERROR: encode failed\r\n");
            for (;;) {
            }
        }
        /* Strip our own 4-byte CDR encapsulation header: the agent's
         * generic/dynamic topic type (TopicPubSubType::serialize() in its
         * own source) always prepends its own CDR_LE header to whatever
         * bytes it receives. Sending ours too produced a double-header
         * sample the RTPS reader rejected -- see xrce/docs/design.md. */
        uint8_t msg[64];
        size_t len = xrce_session_build_write_data(&session, BEST_EFFORT_STREAM_0,
                                                     g_datawriter_id, sample + 4, sample_len - 4,
                                                     msg, sizeof(msg));
        send_frame(msg, len);
        uart_puts("published data=");
        uart_put_uint((uint32_t)i);
        uart_puts("\r\n");
        busy_wait_ticks(PACE_BUSY_LOOPS);
    }
}
