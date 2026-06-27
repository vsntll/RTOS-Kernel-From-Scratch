#ifndef XRCE_RELIABLE_STREAM_H
#define XRCE_RELIABLE_STREAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Output reliable stream (Phase 7c): the sender-side half of XRCE reliable
 * delivery -- the agent's own input reliable stream (its reader side)
 * already implements the other half correctly since it's a real,
 * unmodified agent, so this project only ever needs to build this
 * direction (see xrce/docs/design.md's Phase 7c section for why).
 *
 * Strategy is deliberately the same one the reference client itself uses
 * (uxr_process_acknack()/uxr_next_reliable_nack_buffer_to_send(),
 * src/c/core/session/stream/output_reliable_stream.c): don't decode
 * individual ACKNACK bitmap bits, just retransmit the whole outstanding
 * window whenever the bitmap is nonzero. A fixed-size ring buffer (not a
 * caller-supplied pointer, unlike rtos/'s queue_t) since every message
 * this project's demos send is small and bounded -- see
 * XRCE_RELIABLE_MSG_MAX. */

#define XRCE_RELIABLE_WINDOW 8
#define XRCE_RELIABLE_MSG_MAX 256

typedef struct {
    uint8_t buf[XRCE_RELIABLE_WINDOW][XRCE_RELIABLE_MSG_MAX];
    size_t len[XRCE_RELIABLE_WINDOW];
    bool occupied[XRCE_RELIABLE_WINDOW];
    uint16_t last_acknown; /* highest seq_num fully acked so far */
    uint16_t last_sent;
    bool any_sent;
    bool send_lost; /* set by a nonzero-bitmap ACKNACK, cleared once the retransmit sweep finishes */
    uint16_t retransmit_cursor;
    uint32_t retransmit_count; /* diagnostics -- Phase 8 exposes this */
} xrce_reliable_out_t;

void xrce_reliable_out_init(xrce_reliable_out_t *ro);

/* Stores an already-built message (carrying seq_num `seq`) so it can be
 * retransmitted later. Returns false (message NOT tracked, caller should
 * back off) if the window between `last_acknown` and `seq` would exceed
 * XRCE_RELIABLE_WINDOW -- real backpressure, not just a bookkeeping
 * limit, matching "keep reliable delivery bounded" QoS. */
bool xrce_reliable_out_track(xrce_reliable_out_t *ro, uint16_t seq, const uint8_t *msg, size_t len);

/* Applies a received ACKNACK: everything strictly before `first_unacked`
 * is now known-delivered and freed from the window; a nonzero `bitmap`
 * arms a retransmit sweep of what's left. */
void xrce_reliable_out_on_acknack(xrce_reliable_out_t *ro, uint16_t first_unacked, uint16_t bitmap);

/* Call in a loop after on_acknack() armed a retransmit sweep; returns
 * false once nothing more needs resending this round. */
bool xrce_reliable_out_next_retransmit(xrce_reliable_out_t *ro, const uint8_t **out_msg, size_t *out_len);

#endif
