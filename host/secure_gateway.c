/* Phase 11 (security layer): the trusted boundary between a SECURE_LINK=1
 * firmware build (rtos/arm/ros2_demo.c) and a real, unmodified
 * MicroXRCEAgent. See xrce/include/xrce/secure_transport.h for the full
 * threat model; short version: the real agent can never be taught about
 * this project's authentication envelope, so this program sits in the
 * middle, terminating the authenticated hop and speaking plain,
 * completely standard XRCE-DDS serial framing on to the agent -- the
 * same framing/CRC this project has spoken since Phase 1, unmodified.
 *
 *   firmware (QEMU) <--authenticated--> this gateway <--plain XRCE--> real agent
 *
 * Usage:
 *   host/secure_gateway <board-pty-path> [key-hex]
 *
 * <board-pty-path> is the /dev/pts/N QEMU printed for its `-serial pty`
 * (same value the README's single-node walkthrough hands straight to
 * `MicroXRCEAgent serial -D`; here it goes to this program instead).
 * [key-hex] defaults to the same demo pre-shared key
 * rtos/arm/ros2_demo.c's g_secure_key hardcodes when built with
 * SECURE_LINK=1 -- pass a different one to reproduce the "wrong key
 * rejected" demo in xrce/docs/design.md's Phase 11 section.
 *
 * Prints "AGENT_PTY:/dev/pts/M" once ready -- point a real
 * `MicroXRCEAgent serial -D /dev/pts/M -b 115200` at that path, same as
 * every earlier phase's manual instructions, just one hop further out.
 *
 * Every accepted uplink (board->agent) and downlink (agent->board) frame,
 * and every rejection (bad tag / replay / malformed), is logged to
 * stderr with a reason -- this is what makes the tamper/replay/wrong-key
 * demos observable live, not just "silently correct". */

#include <errno.h>
#include <fcntl.h>
#include <pty.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <poll.h>

#include "xrce/secure_transport.h"
#include "xrce/serial_transport.h"

/* Matches rtos/arm/ros2_demo.c's g_secure_key exactly -- see that file's
 * comment for why a hardcoded demo PSK is an honest simplification here,
 * not an oversight. */
static const uint8_t DEFAULT_KEY[16] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE,
                                          0x13, 0x37, 0xC0, 0xDE, 0xF0, 0x0D, 0x5E, 0xED};

#define MAX_PAYLOAD 512
#define ADDR 0x00 /* matches ros2_demo.c's SERIAL_LOCAL_ADDR/SERIAL_REMOTE_ADDR, both 0x00 */

static void set_raw(int fd, const char *what) {
    struct termios t;
    if (tcgetattr(fd, &t) != 0) {
        fprintf(stderr, "warning: tcgetattr(%s) failed: %s\n", what, strerror(errno));
        return;
    }
    cfmakeraw(&t);
    if (tcsetattr(fd, TCSANOW, &t) != 0) {
        fprintf(stderr, "warning: tcsetattr(%s) failed: %s\n", what, strerror(errno));
    }
}

static int parse_hex_key(const char *hex, uint8_t *out, size_t out_cap) {
    size_t hex_len = strlen(hex);
    if (hex_len == 0 || hex_len % 2 != 0 || hex_len / 2 > out_cap) {
        return -1;
    }
    for (size_t i = 0; i < hex_len / 2; i++) {
        unsigned int byte;
        if (sscanf(hex + i * 2, "%2x", &byte) != 1) {
            return -1;
        }
        out[i] = (uint8_t)byte;
    }
    return (int)(hex_len / 2);
}

static const char *secure_result_str(xrce_secure_result_t r) {
    switch (r) {
        case XRCE_SECURE_OK:
            return "ok";
        case XRCE_SECURE_TOO_SHORT:
            return "too short (malformed)";
        case XRCE_SECURE_BAD_TAG:
            return "bad tag (corrupted in transit, or wrong key)";
        case XRCE_SECURE_REPLAYED:
            return "replayed (stale counter)";
    }
    return "unknown";
}

/* Shared by both directions: decodes one already-fully-received serial
 * frame's payload as a secure envelope, and on success re-encodes the
 * plain inner payload as an ordinary (unauthenticated) serial frame into
 * `out`. Returns the encoded length, or 0 if the frame was rejected (a
 * log line explaining why has already been printed) or malformed. */
static size_t process_secure_frame(const char *direction, xrce_secure_ctx_t *ctx,
                                    const uint8_t *frame_payload, size_t frame_payload_len,
                                    uint8_t *out, size_t out_cap) {
    const uint8_t *plain;
    size_t plain_len;
    xrce_secure_result_t res =
        xrce_secure_unwrap(ctx, frame_payload, frame_payload_len, &plain, &plain_len);
    if (res != XRCE_SECURE_OK) {
        fprintf(stderr, "[%s] REJECTED frame (%s) -- dropped, not forwarded\n", direction,
                secure_result_str(res));
        return 0;
    }
    size_t encoded_len = xrce_serial_frame_encode(out, out_cap, ADDR, ADDR, plain, plain_len);
    if (encoded_len == 0) {
        fprintf(stderr, "[%s] verified frame too large to re-encode (%zu bytes) -- dropped\n",
                direction, plain_len);
        return 0;
    }
    fprintf(stderr, "[%s] verified, %zu byte(s) forwarded\n", direction, plain_len);
    return encoded_len;
}

/* The reverse of process_secure_frame(): wraps a plain inner payload
 * (already extracted from a decoded plain frame) into a secure envelope
 * and re-encodes it as a serial frame. */
static size_t wrap_and_encode(xrce_secure_ctx_t *ctx, const uint8_t *plain, size_t plain_len,
                               uint8_t *out, size_t out_cap) {
    uint8_t wrapped[MAX_PAYLOAD + XRCE_SECURE_OVERHEAD];
    size_t wrapped_len = xrce_secure_wrap(ctx, plain, plain_len, wrapped, sizeof(wrapped));
    if (wrapped_len == 0) {
        fprintf(stderr, "[downlink] wrap failed (payload too large, %zu bytes) -- dropped\n",
                plain_len);
        return 0;
    }
    return xrce_serial_frame_encode(out, out_cap, ADDR, ADDR, wrapped, wrapped_len);
}

int main(int argc, char **argv) {
    if (argc < 2 || argc > 3) {
        fprintf(stderr, "usage: %s <board-pty-path> [key-hex]\n", argv[0]);
        return 1;
    }
    const char *board_path = argv[1];

    uint8_t key_buf[64];
    const uint8_t *key = DEFAULT_KEY;
    size_t key_len = sizeof(DEFAULT_KEY);
    if (argc == 3) {
        int parsed = parse_hex_key(argv[2], key_buf, sizeof(key_buf));
        if (parsed < 0) {
            fprintf(stderr, "error: key must be an even-length hex string\n");
            return 1;
        }
        key = key_buf;
        key_len = (size_t)parsed;
    }

    int board_fd = open(board_path, O_RDWR | O_NOCTTY);
    if (board_fd < 0) {
        fprintf(stderr, "error opening board pty '%s': %s\n", board_path, strerror(errno));
        return 1;
    }
    set_raw(board_fd, "board");

    int agent_master, agent_slave;
    char agent_slave_name[256];
    if (openpty(&agent_master, &agent_slave, agent_slave_name, NULL, NULL) < 0) {
        perror("openpty");
        return 1;
    }
    set_raw(agent_slave, "agent slave");

    printf("AGENT_PTY:%s\n", agent_slave_name);
    fflush(stdout);
    fprintf(stderr, "secure_gateway: board=%s <-> agent=%s, key=%zu bytes\n", board_path,
            agent_slave_name, key_len);

    /* uplink: board -> gateway -> agent (verify+strip).
     * downlink: agent -> gateway -> board (wrap+attach). */
    xrce_secure_ctx_t uplink_ctx, downlink_ctx;
    xrce_secure_init(&uplink_ctx, key, key_len);
    xrce_secure_init(&downlink_ctx, key, key_len);

    static uint8_t board_payload_buf[MAX_PAYLOAD + XRCE_SECURE_OVERHEAD];
    static uint8_t agent_payload_buf[MAX_PAYLOAD];
    xrce_serial_reader_t board_reader, agent_reader;
    xrce_serial_reader_init(&board_reader, ADDR, board_payload_buf, sizeof(board_payload_buf));
    xrce_serial_reader_init(&agent_reader, ADDR, agent_payload_buf, sizeof(agent_payload_buf));

    struct pollfd fds[2];
    fds[0].fd = board_fd;
    fds[0].events = POLLIN;
    fds[1].fd = agent_master;
    fds[1].events = POLLIN;

    for (;;) {
        int n = poll(fds, 2, -1);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            perror("poll");
            break;
        }

        if (fds[0].revents & POLLIN) {
            uint8_t byte;
            ssize_t r = read(board_fd, &byte, 1);
            if (r <= 0) {
                fprintf(stderr, "board link closed\n");
                break;
            }
            uint8_t src_addr;
            uint16_t frame_len;
            xrce_serial_feed_result_t res =
                xrce_serial_reader_feed(&board_reader, byte, &src_addr, &frame_len);
            if (res == XRCE_SERIAL_FEED_ERROR) {
                fprintf(stderr, "[uplink] malformed/corrupted serial frame from board -- dropped\n");
            } else if (res == XRCE_SERIAL_FEED_FRAME_READY) {
                uint8_t out[XRCE_SERIAL_MAX_ENCODED_LEN(MAX_PAYLOAD)];
                size_t out_len = process_secure_frame("uplink", &uplink_ctx, board_payload_buf,
                                                        frame_len, out, sizeof(out));
                if (out_len > 0) {
                    ssize_t w = write(agent_master, out, out_len);
                    (void)w; /* best-effort; a write failure here surfaces as the agent stalling */
                }
            }
        }

        if (fds[1].revents & POLLIN) {
            uint8_t byte;
            ssize_t r = read(agent_master, &byte, 1);
            if (r <= 0) {
                fprintf(stderr, "agent link closed\n");
                break;
            }
            uint8_t src_addr;
            uint16_t frame_len;
            xrce_serial_feed_result_t res =
                xrce_serial_reader_feed(&agent_reader, byte, &src_addr, &frame_len);
            if (res == XRCE_SERIAL_FEED_ERROR) {
                fprintf(stderr, "[downlink] malformed/corrupted serial frame from agent -- dropped\n");
            } else if (res == XRCE_SERIAL_FEED_FRAME_READY) {
                uint8_t out[XRCE_SERIAL_MAX_ENCODED_LEN(MAX_PAYLOAD + XRCE_SECURE_OVERHEAD)];
                size_t out_len = wrap_and_encode(&downlink_ctx, agent_payload_buf, frame_len, out,
                                                  sizeof(out));
                if (out_len > 0) {
                    ssize_t w = write(board_fd, out, out_len);
                    (void)w;
                    fprintf(stderr, "[downlink] wrapped, %u byte(s) forwarded\n", frame_len);
                }
            }
        }
    }

    close(board_fd);
    close(agent_master);
    close(agent_slave);
    return 0;
}
