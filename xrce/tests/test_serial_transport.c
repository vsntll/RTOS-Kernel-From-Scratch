/* Phase 1 milestone (ROS2 layer): exercises the serial framing/CRC against
 * a known-good CRC-16/ARC check value, round-trips payloads (including ones
 * that force byte-stuffing), and injects corruption/loss to prove the
 * reader detects and resyncs instead of wedging -- see the design note in
 * xrce/include/xrce/serial_transport.h for why reliability itself isn't
 * this layer's job. */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../include/xrce/serial_transport.h"

typedef void (*test_case_fn)(void);
static int g_tests_run;

static void run_case(const char *name, test_case_fn fn) {
    printf("[case] %s\n", name);
    fn();
    g_tests_run++;
}

/* Feeds a fully encoded frame's bytes into a fresh reader one at a time,
 * asserting exactly one FRAME_READY at the end and everything before it
 * IN_PROGRESS. Returns the decoded payload length. */
static uint16_t feed_all(xrce_serial_reader_t *r, const uint8_t *frame,
                          size_t frame_len, uint8_t *out_src) {
    uint16_t out_len = 0;
    for (size_t i = 0; i < frame_len; i++) {
        xrce_serial_feed_result_t res =
            xrce_serial_reader_feed(r, frame[i], out_src, &out_len);
        if (i + 1 < frame_len) {
            assert(res == XRCE_SERIAL_FEED_IN_PROGRESS);
        } else {
            assert(res == XRCE_SERIAL_FEED_FRAME_READY);
        }
    }
    return out_len;
}

/* Case 1: CRC-16/ARC implementation matches the standard published check
 * value for "123456789" (0xBB3D) -- an external, independently-verifiable
 * fact about this exact CRC variant (poly 0x8005, reflected, init 0),
 * rather than something that could only be checked against itself. */
static void case_crc_known_vector(void) {
    const uint8_t check[] = "123456789";
    uint16_t crc = xrce_serial_crc16(check, 9);
    assert(crc == 0xBB3D);
}

/* Case 2: a plain payload with nothing needing escape round-trips exactly. */
static void case_round_trip_plain(void) {
    const uint8_t payload[] = {0x01, 0x02, 0x03, 0xAA, 0xFF, 0x00};
    uint8_t frame[XRCE_SERIAL_MAX_ENCODED_LEN(sizeof(payload))];
    size_t frame_len = xrce_serial_frame_encode(frame, sizeof(frame), 0x01,
                                                 0x02, payload, sizeof(payload));
    assert(frame_len > 0);
    assert(frame[0] == XRCE_SERIAL_FLAG);

    uint8_t payload_buf[64];
    xrce_serial_reader_t r;
    xrce_serial_reader_init(&r, 0x02, payload_buf, sizeof(payload_buf));

    uint8_t src = 0;
    uint16_t len = feed_all(&r, frame, frame_len, &src);
    assert(src == 0x01);
    assert(len == sizeof(payload));
    assert(memcmp(payload_buf, payload, sizeof(payload)) == 0);
}

/* Case 3: a payload containing every byte that forces stuffing (0x7E, 0x7D)
 * round-trips exactly -- proves escape/unescape symmetry, not just the
 * common case. */
static void case_round_trip_needs_stuffing(void) {
    const uint8_t payload[] = {0x7E, 0x7D, 0x00, 0x7E, 0x7E, 0x7D, 0x01};
    uint8_t frame[XRCE_SERIAL_MAX_ENCODED_LEN(sizeof(payload))];
    size_t frame_len = xrce_serial_frame_encode(frame, sizeof(frame), 0x05,
                                                 0x06, payload, sizeof(payload));
    assert(frame_len > 0);
    /* Every payload byte here needed a 2-byte escape, so the frame must be
     * longer than payload_len + the unstuffed header/crc/flag overhead. */
    assert(frame_len > 1 + XRCE_SERIAL_HEADER_LEN + sizeof(payload) + XRCE_SERIAL_CRC_LEN);

    uint8_t payload_buf[64];
    xrce_serial_reader_t r;
    xrce_serial_reader_init(&r, 0x06, payload_buf, sizeof(payload_buf));

    uint8_t src = 0;
    uint16_t len = feed_all(&r, frame, frame_len, &src);
    assert(len == sizeof(payload));
    assert(memcmp(payload_buf, payload, sizeof(payload)) == 0);
}

/* Case 4: flipping a payload byte after encoding must be caught by the CRC
 * check on the last frame byte, not silently accepted. */
static void case_corruption_detected(void) {
    const uint8_t payload[] = {0x10, 0x20, 0x30, 0x40};
    uint8_t frame[XRCE_SERIAL_MAX_ENCODED_LEN(sizeof(payload))];
    size_t frame_len = xrce_serial_frame_encode(frame, sizeof(frame), 0x01,
                                                 0x02, payload, sizeof(payload));
    assert(frame_len > 0);

    /* Corrupt one payload byte (index 5 = flag, addr, addr, len, len,
     * first payload byte). None of 0x7E/0x7D/0x81 need stuffing so the
     * layout here is exactly [flag][src][dst][len_lsb][len_msb][payload...]. */
    frame[5] ^= 0xFF;

    uint8_t payload_buf[64];
    xrce_serial_reader_t r;
    xrce_serial_reader_init(&r, 0x02, payload_buf, sizeof(payload_buf));

    uint8_t src = 0;
    uint16_t len = 0;
    xrce_serial_feed_result_t res = XRCE_SERIAL_FEED_IN_PROGRESS;
    for (size_t i = 0; i < frame_len; i++) {
        res = xrce_serial_reader_feed(&r, frame[i], &src, &len);
    }
    assert(res == XRCE_SERIAL_FEED_ERROR);
}

/* Case 5: garbage bytes (simulating line noise / dropped bytes) before a
 * well-formed frame must not prevent that frame from decoding. */
static void case_resync_after_garbage(void) {
    const uint8_t garbage[] = {0x00, 0xFF, 0x7D, 0x11, 0x22};
    const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t frame[XRCE_SERIAL_MAX_ENCODED_LEN(sizeof(payload))];
    size_t frame_len = xrce_serial_frame_encode(frame, sizeof(frame), 0x03,
                                                 0x04, payload, sizeof(payload));
    assert(frame_len > 0);

    uint8_t payload_buf[64];
    xrce_serial_reader_t r;
    xrce_serial_reader_init(&r, 0x04, payload_buf, sizeof(payload_buf));

    uint8_t src = 0;
    uint16_t len = 0;
    for (size_t i = 0; i < sizeof(garbage); i++) {
        xrce_serial_feed_result_t res =
            xrce_serial_reader_feed(&r, garbage[i], &src, &len);
        assert(res == XRCE_SERIAL_FEED_IN_PROGRESS);
    }
    len = feed_all(&r, frame, frame_len, &src);
    assert(src == 0x03);
    assert(len == sizeof(payload));
    assert(memcmp(payload_buf, payload, sizeof(payload)) == 0);
}

/* Case 6: a frame truncated mid-payload (bytes lost) followed immediately
 * by a fresh, complete frame must resync on the second frame's flag byte
 * rather than staying stuck waiting for the first frame's missing bytes. */
static void case_resync_after_truncated_frame(void) {
    const uint8_t payload1[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    const uint8_t payload2[] = {0x99, 0x98};
    uint8_t frame1[XRCE_SERIAL_MAX_ENCODED_LEN(sizeof(payload1))];
    uint8_t frame2[XRCE_SERIAL_MAX_ENCODED_LEN(sizeof(payload2))];
    size_t frame1_len = xrce_serial_frame_encode(frame1, sizeof(frame1), 0x07,
                                                  0x08, payload1, sizeof(payload1));
    size_t frame2_len = xrce_serial_frame_encode(frame2, sizeof(frame2), 0x07,
                                                  0x08, payload2, sizeof(payload2));
    assert(frame1_len > 0 && frame2_len > 0);

    uint8_t payload_buf[64];
    xrce_serial_reader_t r;
    xrce_serial_reader_init(&r, 0x08, payload_buf, sizeof(payload_buf));

    uint8_t src = 0;
    uint16_t len = 0;
    /* Feed only the first half of frame1 -- simulates the rest being lost. */
    for (size_t i = 0; i < frame1_len / 2; i++) {
        xrce_serial_feed_result_t res =
            xrce_serial_reader_feed(&r, frame1[i], &src, &len);
        assert(res == XRCE_SERIAL_FEED_IN_PROGRESS);
    }
    /* frame2 starts with its own 0x7E, which must resync the reader even
     * though it was mid-payload waiting for frame1's remaining bytes. */
    len = feed_all(&r, frame2, frame2_len, &src);
    assert(src == 0x07);
    assert(len == sizeof(payload2));
    assert(memcmp(payload_buf, payload2, sizeof(payload2)) == 0);
}

/* Case 7: a frame addressed to a different destination is dropped (not
 * delivered to us), and doesn't disrupt decoding the next, correctly
 * addressed frame. */
static void case_address_filtering(void) {
    const uint8_t payload[] = {0x42};
    uint8_t frame_other[XRCE_SERIAL_MAX_ENCODED_LEN(sizeof(payload))];
    uint8_t frame_mine[XRCE_SERIAL_MAX_ENCODED_LEN(sizeof(payload))];
    size_t other_len = xrce_serial_frame_encode(frame_other, sizeof(frame_other),
                                                 0x01, 0x99 /* not us */, payload,
                                                 sizeof(payload));
    size_t mine_len = xrce_serial_frame_encode(frame_mine, sizeof(frame_mine),
                                                0x01, 0x02 /* us */, payload,
                                                sizeof(payload));
    assert(other_len > 0 && mine_len > 0);

    uint8_t payload_buf[64];
    xrce_serial_reader_t r;
    xrce_serial_reader_init(&r, 0x02, payload_buf, sizeof(payload_buf));

    uint8_t src = 0;
    uint16_t len = 0;
    bool saw_error = false;
    for (size_t i = 0; i < other_len; i++) {
        xrce_serial_feed_result_t res =
            xrce_serial_reader_feed(&r, frame_other[i], &src, &len);
        if (res == XRCE_SERIAL_FEED_ERROR) {
            saw_error = true;
        } else {
            assert(res == XRCE_SERIAL_FEED_IN_PROGRESS);
        }
    }
    assert(saw_error);

    len = feed_all(&r, frame_mine, mine_len, &src);
    assert(len == sizeof(payload));
    assert(payload_buf[0] == 0x42);
}

int main(void) {
    run_case("CRC-16/ARC matches standard check vector", case_crc_known_vector);
    run_case("round trip, no stuffing needed", case_round_trip_plain);
    run_case("round trip, every byte needs stuffing", case_round_trip_needs_stuffing);
    run_case("corrupted payload byte is detected", case_corruption_detected);
    run_case("resync after leading garbage bytes", case_resync_after_garbage);
    run_case("resync after a truncated frame", case_resync_after_truncated_frame);
    run_case("frames to a different address are dropped", case_address_filtering);

    printf("PASS: %d test cases\n", g_tests_run);
    return 0;
}
