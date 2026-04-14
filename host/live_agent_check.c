/* Not part of `make test` (needs a live, external MicroXRCEAgent process
 * and BSD sockets, so it isn't portable/embeddable like everything under
 * xrce/) -- a manual, one-off proof that xrce_session_build_create_client()
 * produces bytes the real, unmodified agent actually accepts. This is the
 * actual point of choosing Option A: not "our own reader can parse our own
 * writer's output" (the unit tests already prove that), but "a real agent
 * neither of us wrote accepts it."
 *
 * Usage (agent already running: `MicroXRCEAgent udp4 -p 8888`):
 *   gcc -Ixrce/include host/live_agent_check.c xrce/build/libxrce.a -o /tmp/live_agent_check
 *   /tmp/live_agent_check 127.0.0.1 8888
 */
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "xrce/session.h"

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s <agent_ip> <agent_port>\n", argv[0]);
        return 2;
    }

    uint8_t key[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    xrce_session_t session;
    xrce_session_init(&session, 0x01, key, 512);

    uint8_t out[64];
    size_t out_len = xrce_session_build_create_client(&session, out, sizeof(out));
    if (out_len == 0) {
        fprintf(stderr, "FAIL: xrce_session_build_create_client returned 0\n");
        return 1;
    }
    printf("Sending %zu-byte CREATE_CLIENT to %s:%s\n", out_len, argv[1], argv[2]);

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)atoi(argv[2]));
    if (inet_pton(AF_INET, argv[1], &addr.sin_addr) != 1) {
        fprintf(stderr, "FAIL: bad IP address %s\n", argv[1]);
        return 1;
    }

    if (sendto(fd, out, out_len, 0, (struct sockaddr *)&addr, sizeof(addr)) != (ssize_t)out_len) {
        perror("sendto");
        return 1;
    }

    struct timeval tv = {.tv_sec = 3, .tv_usec = 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t in[128];
    ssize_t n = recv(fd, in, sizeof(in), 0);
    if (n <= 0) {
        fprintf(stderr, "FAIL: no reply from agent within 3s (is it running on %s:%s?)\n",
                argv[1], argv[2]);
        return 1;
    }
    printf("Received %zd-byte reply, raw bytes:", n);
    for (ssize_t i = 0; i < n; i++) {
        printf(" %02x", in[i]);
    }
    printf("\n");

    if (xrce_session_parse_create_client_reply(in, (size_t)n)) {
        printf("PASS: real MicroXRCEAgent accepted CREATE_CLIENT and replied STATUS_AGENT/OK\n");
        return 0;
    }
    fprintf(stderr, "FAIL: reply did not parse as a successful STATUS_AGENT\n");
    return 1;
}
