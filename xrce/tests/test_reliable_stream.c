/* Phase 7c: xrce_reliable_out_t's window/retransmit bookkeeping, tested
 * against simulated loss -- the actual "does retransmit recover a dropped
 * message" behavior, not just "does it round-trip when nothing is lost". */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../include/xrce/reliable_stream.h"

typedef void (*test_case_fn)(void);
static int g_tests_run;

static void run_case(const char *name, test_case_fn fn) {
    printf("[case] %s\n", name);
    fn();
    g_tests_run++;
}

static void case_tracked_message_freed_on_ack(void) {
    xrce_reliable_out_t ro;
    xrce_reliable_out_init(&ro);

    uint8_t msg[4] = {1, 2, 3, 4};
    assert(xrce_reliable_out_track(&ro, 1, msg, sizeof(msg)));

    /* ACKNACK reporting first_unacked=2 (i.e. seq 1 was received), bitmap
     * 0 -- nothing missing, no retransmit should be armed. */
    xrce_reliable_out_on_acknack(&ro, 2, 0);
    const uint8_t *out_msg;
    size_t out_len;
    assert(!xrce_reliable_out_next_retransmit(&ro, &out_msg, &out_len));
}

static void case_lost_message_gets_retransmitted(void) {
    xrce_reliable_out_t ro;
    xrce_reliable_out_init(&ro);

    uint8_t msg1[3] = {1, 1, 1};
    uint8_t msg2[3] = {2, 2, 2};
    uint8_t msg3[3] = {3, 3, 3};
    assert(xrce_reliable_out_track(&ro, 1, msg1, sizeof(msg1)));
    assert(xrce_reliable_out_track(&ro, 2, msg2, sizeof(msg2)));
    assert(xrce_reliable_out_track(&ro, 3, msg3, sizeof(msg3)));

    /* Simulated loss: seq 2 never arrived. The agent's real ACKNACK would
     * report first_unacked=1 (seq 1 was actually the first one lost in
     * this scenario -- pick first_unacked=1, bitmap!=0) to ask for a
     * resend of everything from 1 onward. */
    xrce_reliable_out_on_acknack(&ro, 1, 0x0001);

    const uint8_t *out_msg;
    size_t out_len;
    int count = 0;
    uint8_t seen[3][3];
    while (xrce_reliable_out_next_retransmit(&ro, &out_msg, &out_len)) {
        assert(out_len == 3);
        memcpy(seen[count], out_msg, 3);
        count++;
    }
    assert(count == 3); /* the whole outstanding window gets resent, matching the reference client */
    assert(memcmp(seen[0], msg1, 3) == 0);
    assert(memcmp(seen[1], msg2, 3) == 0);
    assert(memcmp(seen[2], msg3, 3) == 0);
    assert(ro.retransmit_count == 3);
}

static void case_window_backpressure(void) {
    xrce_reliable_out_t ro;
    xrce_reliable_out_init(&ro);

    uint8_t msg[1] = {0xAA};
    uint16_t seq = 1;
    int tracked = 0;
    for (; seq <= XRCE_RELIABLE_WINDOW + 2; seq++) {
        if (xrce_reliable_out_track(&ro, seq, msg, sizeof(msg))) {
            tracked++;
        } else {
            break;
        }
    }
    /* Nothing has been acked, so the window fills at exactly
     * XRCE_RELIABLE_WINDOW outstanding messages -- proving this is real
     * backpressure, not an unbounded buffer. */
    assert(tracked == XRCE_RELIABLE_WINDOW);
}

int main(void) {
    run_case("a fully-acked message is not retransmitted", case_tracked_message_freed_on_ack);
    run_case("a lost message triggers retransmit of the outstanding window",
              case_lost_message_gets_retransmitted);
    run_case("the output window applies real backpressure once full", case_window_backpressure);

    printf("PASS: %d test cases\n", g_tests_run);
    return 0;
}
