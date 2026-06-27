#include "xrce/reliable_stream.h"

#include <string.h>

static size_t slot_for(uint16_t seq) {
    return (size_t)(seq % XRCE_RELIABLE_WINDOW);
}

void xrce_reliable_out_init(xrce_reliable_out_t *ro) {
    memset(ro, 0, sizeof(*ro));
}

bool xrce_reliable_out_track(xrce_reliable_out_t *ro, uint16_t seq, const uint8_t *msg, size_t len) {
    if (len > XRCE_RELIABLE_MSG_MAX) {
        return false;
    }
    if (ro->any_sent) {
        uint16_t outstanding = (uint16_t)(seq - ro->last_acknown);
        if (outstanding > XRCE_RELIABLE_WINDOW) {
            return false; /* window full: caller must wait for an ACKNACK to free room */
        }
    }
    size_t slot = slot_for(seq);
    memcpy(ro->buf[slot], msg, len);
    ro->len[slot] = len;
    ro->occupied[slot] = true;
    ro->last_sent = seq;
    ro->any_sent = true;
    return true;
}

void xrce_reliable_out_on_acknack(xrce_reliable_out_t *ro, uint16_t first_unacked, uint16_t bitmap) {
    if (ro->any_sent) {
        /* Free every tracked slot strictly before first_unacked -- it's
         * been delivered, matching uxr_process_acknack()'s
         * last_acknown = first_unacked - 1. */
        uint16_t seq = ro->last_acknown;
        while (seq != first_unacked) {
            size_t slot = slot_for(seq);
            ro->occupied[slot] = false;
            seq++;
        }
        ro->last_acknown = (uint16_t)(first_unacked - 1);
    }
    ro->send_lost = (bitmap != 0);
    ro->retransmit_cursor = ro->last_acknown;
}

bool xrce_reliable_out_next_retransmit(xrce_reliable_out_t *ro, const uint8_t **out_msg, size_t *out_len) {
    if (!ro->send_lost) {
        return false;
    }
    while (ro->retransmit_cursor != ro->last_sent) {
        ro->retransmit_cursor++;
        size_t slot = slot_for(ro->retransmit_cursor);
        if (ro->occupied[slot]) {
            *out_msg = ro->buf[slot];
            *out_len = ro->len[slot];
            ro->retransmit_count++;
            return true;
        }
    }
    ro->send_lost = false;
    return false;
}
