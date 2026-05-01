/* Phase 4/5/6 (ROS2 layer): the actual "RTOS task in QEMU talks to a real
 * agent, both directions" deliverable. Runs the same xrce/ session layer
 * already verified over UDP (host/live_publish_demo.c,
 * host/live_subscribe_demo.c) here instead over UART1, framed with
 * xrce/src/serial_transport.c, so the bytes go QEMU -serial -> a real
 * MicroXRCEAgent's `serial` transport instead of a raw UDP socket.
 *
 * Publishes an incrementing std_msgs/Int32 on rt/chatter (Phase 4), and
 * subscribes to rt/setpoint (Phase 5): a real `ros2 topic pub
 * /setpoint std_msgs/msg/Int32 "{data: N}"` prints the new value over
 * UART -- the "host sends a command, the RTOS task receives and acts on
 * it" scenario from the original project brief. It also immediately
 * echoes the value back out on rt/pong (Phase 6), which is what
 * `host/bench_latency.c` uses to measure real host<->RTOS round-trip
 * latency through this exact firmware, not a simulated stand-in for it.
 *
 * No RTOS task/scheduler involvement here -- this is a plain sequential
 * main(), like Phase 1/3's demos before Phase 8 added preemption; SysTick
 * is never started, so kernel_arm.c/context_switch.s are linked in only
 * to satisfy startup.s's vector table (PendSV_Handler/SysTick_Handler
 * symbols), not actually exercised.
 *
 * Serial addressing: local_addr/remote_addr both 0x00, matching the
 * common default used across eProsima's own client examples when neither
 * side customizes it.
 *
 * CREATE_CLIENT + entity creation is re-announced every few publishes
 * rather than sent once at boot: QEMU boots and starts sending within a
 * fraction of a real second, but the host-side agent process takes a few
 * real seconds to attach to the pty/serial device, so a send-once
 * handshake at boot reliably loses the race and is never seen.
 * Re-announcing periodically means whenever the agent does attach, it
 * catches a fresh handshake within a bounded time -- duplicate CREATEs
 * for already-existing entities (and duplicate READ_DATA requests) are
 * harmless, which is a fine tradeoff for a demo. */

#include <stdint.h>

#include "uart.h"
#include "xrce/msgs.h"
#include "xrce/serial_transport.h"
#include "xrce/session.h"

#define SERIAL_LOCAL_ADDR 0x00
#define SERIAL_REMOTE_ADDR 0x00
#define BEST_EFFORT_STREAM_0 1
/* Deliberately not small: every re-announce resends READ_DATA too, and the
 * agent's DataReader::read() always calls stop_reading() before
 * start_reading() on its internal delivery thread (Reader<>::start_reading()
 * in the agent's own Reader.hpp) -- re-announcing too often (every ~170ms,
 * what a 5-publish interval worked out to under QEMU's fast loop) keeps
 * killing and restarting that thread. This value gives it a real window to
 * stay alive between restarts, confirmed by testing: setpoint delivery over
 * the real serial transport works reliably at this interval (see
 * xrce/docs/design.md's Phase 5 section for the full story, including two
 * debugging-tool bugs that made it look broken before this was found). */
#define REANNOUNCE_EVERY_N_PUBLISHES 200

/* Unverified real-world duration under QEMU TCG emulation (see main.c's
 * SYSTICK_RELOAD comment for the same caveat), and NOT comparable to the
 * plain busy_wait_ticks() this replaced: each iteration here also polls a
 * real (emulated) MMIO register (uart_getc_nonblocking()), and MMIO
 * accesses are dramatically slower per-iteration under QEMU's TCG JIT
 * than a register-only loop -- 900000 iterations of *this* loop (copied
 * over unchanged from the old MMIO-free constant) took several real
 * seconds just to get through the first few CREATEs, observed via a
 * file-backed UART capture. Cut down accordingly; still only paces UART
 * output for a human watching the terminal, and periodic re-announcement
 * is what actually makes this demo's correctness independent of the
 * value either way. */
#define PACE_BUSY_LOOPS 5000

static xrce_serial_reader_t g_rx_reader;
static uint8_t g_rx_payload_buf[256];

static xrce_object_id_t g_participant_id;
static xrce_object_id_t g_topic_id;
static xrce_object_id_t g_publisher_id;
static xrce_object_id_t g_datawriter_id;
static xrce_object_id_t g_cmd_topic_id;
static xrce_object_id_t g_subscriber_id;
static xrce_object_id_t g_datareader_id;
static xrce_object_id_t g_pong_topic_id;
static xrce_object_id_t g_pong_datawriter_id;
static xrce_session_t *g_session; /* handle_incoming_frame() needs this to echo a pong */

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

static void uart_put_int(int32_t v) {
    if (v < 0) {
        uart_putc('-');
        uart_put_uint((uint32_t)(-(int64_t)v));
    } else {
        uart_put_uint((uint32_t)v);
    }
}

static void send_frame(const uint8_t *payload, size_t len); /* defined below */

/* Handles one fully-decoded incoming frame: if it's a DATA submessage from
 * our datareader, decode it as a raw (header-less -- see session.h) int32,
 * print it, and immediately echo it back out on rt/pong (Phase 6) --
 * that echo is what host/bench_latency.c times to measure real
 * host<->RTOS round-trip latency through this exact firmware. Anything
 * else (STATUS replies to our own CREATEs, etc.) is silently ignored --
 * this demo doesn't correlate replies to requests. */
static void handle_incoming_frame(xrce_object_id_t datareader_id) {
    xrce_object_id_t from_id;
    const uint8_t *sample;
    size_t sample_len;
    if (!xrce_session_parse_data(g_rx_payload_buf, g_rx_reader.pos, &from_id, &sample,
                                  &sample_len)) {
        return;
    }
    if (from_id.id != datareader_id.id || from_id.kind != datareader_id.kind || sample_len < 4) {
        return;
    }
    int32_t setpoint = (int32_t)((uint32_t)sample[0] | ((uint32_t)sample[1] << 8) |
                                  ((uint32_t)sample[2] << 16) | ((uint32_t)sample[3] << 24));
    uart_puts("setpoint updated: ");
    uart_put_int(setpoint);
    uart_puts("\r\n");

    uint8_t msg[64];
    size_t len = xrce_session_build_write_data(g_session, BEST_EFFORT_STREAM_0,
                                                g_pong_datawriter_id, sample, 4, msg, sizeof(msg));
    send_frame(msg, len);
}

/* Paces the demo (same busy-loop role Phase 8's demo already used) while
 * also draining and decoding any bytes the agent has sent back -- doing
 * both here, rather than a plain busy-wait plus a separate poll pass, is
 * what makes RX responsive without a real scheduler/interrupt-driven RX
 * to hang this off of. */
static void pace_and_poll_rx(xrce_object_id_t datareader_id, int n) {
    for (volatile int i = 0; i < n; i++) {
        int c = uart_getc_nonblocking();
        if (c < 0) {
            continue;
        }
        uint8_t src_addr;
        uint16_t frame_len;
        xrce_serial_feed_result_t res =
            xrce_serial_reader_feed(&g_rx_reader, (uint8_t)c, &src_addr, &frame_len);
        if (res == XRCE_SERIAL_FEED_FRAME_READY) {
            handle_incoming_frame(datareader_id);
        }
    }
}

/* Sized for the largest message this demo ever sends (datawriter/
 * datareader CREATE, whose XML strings are the longest), worst-case-stuffed.
 * A previous version of this function hardcoded XRCE_SERIAL_MAX_ENCODED_LEN(64)
 * -- enough for CREATE_CLIENT and the empty-XML publisher CREATE, too small
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

static void announce_entities(xrce_session_t *session) {
    uint8_t msg[192];
    size_t len;

    uart_puts("-> CREATE_CLIENT\r\n");
    len = xrce_session_build_create_client(session, msg, sizeof(msg));
    send_frame(msg, len);
    pace_and_poll_rx(g_datareader_id, PACE_BUSY_LOOPS);

    g_participant_id = xrce_object_id(0x001, XRCE_OBJK_PARTICIPANT);
    uart_puts("-> CREATE participant\r\n");
    len = xrce_session_build_create_participant(
        session, BEST_EFFORT_STREAM_0, g_participant_id, 0,
        "<dds><participant><rtps><name>rtos_qemu_demo</name></rtps></participant></dds>", msg,
        sizeof(msg));
    send_frame(msg, len);
    pace_and_poll_rx(g_datareader_id, PACE_BUSY_LOOPS);

    g_topic_id = xrce_object_id(0x001, XRCE_OBJK_TOPIC);
    uart_puts("-> CREATE topic (rt/chatter)\r\n");
    len = xrce_session_build_create_xml(session, BEST_EFFORT_STREAM_0, g_topic_id,
                                         g_participant_id,
                                         "<dds><topic><name>rt/chatter</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType></topic></dds>",
                                         msg, sizeof(msg));
    send_frame(msg, len);
    pace_and_poll_rx(g_datareader_id, PACE_BUSY_LOOPS);

    g_publisher_id = xrce_object_id(0x001, XRCE_OBJK_PUBLISHER);
    uart_puts("-> CREATE publisher\r\n");
    len = xrce_session_build_create_xml(session, BEST_EFFORT_STREAM_0, g_publisher_id,
                                         g_participant_id, "", msg, sizeof(msg));
    send_frame(msg, len);
    pace_and_poll_rx(g_datareader_id, PACE_BUSY_LOOPS);

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
    pace_and_poll_rx(g_datareader_id, PACE_BUSY_LOOPS);

    /* Subscribe side (Phase 5): a distinct topic id (0x002) since ObjectId
     * uniqueness is per (id, kind) -- reusing 0x001 here would collide
     * with the chatter topic, which also has kind TOPIC. */
    g_cmd_topic_id = xrce_object_id(0x002, XRCE_OBJK_TOPIC);
    uart_puts("-> CREATE topic (rt/setpoint)\r\n");
    len = xrce_session_build_create_xml(session, BEST_EFFORT_STREAM_0, g_cmd_topic_id,
                                         g_participant_id,
                                         "<dds><topic><name>rt/setpoint</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType></topic></dds>",
                                         msg, sizeof(msg));
    send_frame(msg, len);
    pace_and_poll_rx(g_datareader_id, PACE_BUSY_LOOPS);

    g_subscriber_id = xrce_object_id(0x001, XRCE_OBJK_SUBSCRIBER);
    uart_puts("-> CREATE subscriber\r\n");
    len = xrce_session_build_create_xml(session, BEST_EFFORT_STREAM_0, g_subscriber_id,
                                         g_participant_id, "", msg, sizeof(msg));
    send_frame(msg, len);
    pace_and_poll_rx(g_datareader_id, PACE_BUSY_LOOPS);

    g_datareader_id = xrce_object_id(0x001, XRCE_OBJK_DATAREADER);
    uart_puts("-> CREATE datareader\r\n");
    len = xrce_session_build_create_xml(session, BEST_EFFORT_STREAM_0, g_datareader_id,
                                         g_subscriber_id,
                                         "<dds><data_reader><topic><kind>NO_KEY</kind>"
                                         "<name>rt/setpoint</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType>"
                                         "</topic></data_reader></dds>",
                                         msg, sizeof(msg));
    send_frame(msg, len);
    pace_and_poll_rx(g_datareader_id, PACE_BUSY_LOOPS);

    /* Resent every re-announce, same as the CREATEs -- see the
     * REANNOUNCE_EVERY_N_PUBLISHES comment above for why the interval
     * matters here specifically (too frequent starves real deliveries by
     * constantly restarting the agent's internal read thread; too rare
     * -- or send-once -- loses the same boot-timing race CREATE_CLIENT
     * has to solve in the first place). */
    uart_puts("-> READ_DATA (rt/setpoint)\r\n");
    len = xrce_session_build_read_data(session, BEST_EFFORT_STREAM_0, g_datareader_id,
                                        BEST_EFFORT_STREAM_0, msg, sizeof(msg));
    send_frame(msg, len);
    pace_and_poll_rx(g_datareader_id, PACE_BUSY_LOOPS);

    /* Phase 6: rt/pong, echoed by handle_incoming_frame() -- what
     * host/bench_latency.c times for real host<->RTOS round-trip latency. */
    g_pong_topic_id = xrce_object_id(0x003, XRCE_OBJK_TOPIC);
    uart_puts("-> CREATE topic (rt/pong)\r\n");
    len = xrce_session_build_create_xml(session, BEST_EFFORT_STREAM_0, g_pong_topic_id,
                                         g_participant_id,
                                         "<dds><topic><name>rt/pong</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType></topic></dds>",
                                         msg, sizeof(msg));
    send_frame(msg, len);
    pace_and_poll_rx(g_datareader_id, PACE_BUSY_LOOPS);

    g_pong_datawriter_id = xrce_object_id(0x002, XRCE_OBJK_DATAWRITER);
    uart_puts("-> CREATE datawriter (rt/pong)\r\n");
    len = xrce_session_build_create_xml(session, BEST_EFFORT_STREAM_0, g_pong_datawriter_id,
                                         g_publisher_id,
                                         "<dds><data_writer><topic><kind>NO_KEY</kind>"
                                         "<name>rt/pong</name>"
                                         "<dataType>std_msgs::msg::dds_::Int32_</dataType>"
                                         "</topic></data_writer></dds>",
                                         msg, sizeof(msg));
    send_frame(msg, len);
    pace_and_poll_rx(g_datareader_id, PACE_BUSY_LOOPS);
}

int main(void) {
    uart_init();
    uart_puts("RTOS ROS2 demo: booting, sending XRCE frames over UART1\r\n");

    uint8_t key[4] = {0x52, 0x54, 0x4F, 0x53}; /* "RTOS" */
    xrce_session_t session;
    xrce_session_init(&session, 0x01, key, 512);
    xrce_serial_reader_init(&g_rx_reader, SERIAL_LOCAL_ADDR, g_rx_payload_buf,
                             sizeof(g_rx_payload_buf));
    g_session = &session; /* handle_incoming_frame() echoes through this */

    announce_entities(&session);

    uart_puts("Publishing std_msgs/Int32 on rt/chatter, watching rt/setpoint...\r\n");
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
        pace_and_poll_rx(g_datareader_id, PACE_BUSY_LOOPS);
    }
}
