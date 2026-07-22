/* Phase 11: authenticates the actual physical link (UART/serial) between
 * an RTOS board and its trusted gateway -- NOT a change to the XRCE-DDS
 * wire protocol itself, which is why this composes with, rather than
 * modifies, xrce/include/xrce/serial_transport.h.
 *
 * Threat model and why the boundary is where it is: Option A's whole
 * value is that a real, unmodified `MicroXRCEAgent` talks to this
 * project's firmware with zero changes on the agent side. That means the
 * bytes the agent receives must stay plain, standard XRCE-DDS -- there is
 * nowhere to add authentication tags the agent would understand. What
 * *can* be authenticated without breaking that is the link this project
 * actually controls both ends of: firmware <-> a gateway this project
 * also owns (host/secure_gateway.c), which then speaks unmodified
 * XRCE-DDS to the real agent on a second, separate hop. This mirrors a
 * real industrial/automotive pattern (authenticate the field bus/ECU
 * link; a trusted gateway is the boundary to the untrusted-in-a-different-
 * way backend), not a compromise specific to this project.
 *
 * Wire format of the "secure payload" this layer produces -- this is what
 * gets handed to xrce_serial_frame_encode() as *its* payload, so the
 * outer framing (flag/addr/len/CRC/byte-stuffing) is completely
 * unchanged and reused as-is:
 *
 *   counter (4 bytes, big-endian) | tag (8 bytes) | inner payload (N bytes)
 *
 * `tag` is HMAC-SHA256(key, counter || inner payload), truncated to the
 * first 8 bytes -- a real, common practice (e.g. AUTOSAR SecOC's typical
 * truncated-MAC lengths), not this project inventing weaker crypto:
 * truncation trades a bit of forgery resistance (2^-64 vs 2^-256) for
 * much less per-frame overhead, appropriate for a link that also carries
 * this project's own CRC-16 for accidental corruption and where the
 * threat being defended against is message injection/tampering, not
 * state-level adversaries.
 *
 * `counter` is a strictly-increasing per-sender sequence number, checked
 * on receive: a valid tag with a counter <= the highest one already
 * accepted is rejected as a replay (xrce_secure_unwrap() ->
 * XRCE_SECURE_REPLAYED) rather than re-delivered. Each direction
 * (board->gateway, gateway->board) has its own independent counter and
 * its own xrce_secure_ctx_t.
 *
 * Explicitly NOT provided (stated plainly, same policy as every other
 * "known limitations" note in xrce/docs/design.md): confidentiality
 * (payload bytes are authenticated, not encrypted -- a real DTLS-lite
 * layer would add that, and was scoped out of this phase) and key
 * exchange/rotation (the pre-shared key is provisioned out of band,
 * matching how a real embedded device's initial key is usually
 * provisioned at manufacture time, not negotiated at runtime). */

#ifndef XRCE_SECURE_TRANSPORT_H
#define XRCE_SECURE_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define XRCE_SECURE_COUNTER_LEN 4
#define XRCE_SECURE_TAG_LEN 8
#define XRCE_SECURE_OVERHEAD (XRCE_SECURE_COUNTER_LEN + XRCE_SECURE_TAG_LEN)

typedef struct {
    const uint8_t *key;
    size_t key_len;
    uint32_t send_counter;      /* next value to use when wrapping an outgoing frame */
    uint32_t last_recv_counter; /* highest counter accepted so far; 0 means "none yet" */
    bool have_received;
} xrce_secure_ctx_t;

/* `key`/`key_len` must outlive ctx -- not copied, same convention as
 * xrce_session_init()'s client_key. */
void xrce_secure_init(xrce_secure_ctx_t *ctx, const uint8_t *key, size_t key_len);

/* Wraps `payload` as `counter | tag | payload` into `out`, advancing
 * ctx->send_counter. Returns bytes written (payload_len +
 * XRCE_SECURE_OVERHEAD), or 0 if out_cap is too small. The result is
 * meant to be passed straight to xrce_serial_frame_encode() as its
 * payload argument. */
size_t xrce_secure_wrap(xrce_secure_ctx_t *ctx, const uint8_t *payload, size_t payload_len,
                         uint8_t *out, size_t out_cap);

typedef enum {
    XRCE_SECURE_OK,
    XRCE_SECURE_TOO_SHORT, /* fewer than XRCE_SECURE_OVERHEAD bytes -- malformed, not even attempted */
    XRCE_SECURE_BAD_TAG,   /* HMAC verification failed: corrupted in transit, or wrong key */
    XRCE_SECURE_REPLAYED,  /* tag valid, but counter <= last accepted -- stale/replayed frame */
} xrce_secure_result_t;

/* Verifies and unwraps a secure payload (the exact bytes xrce_secure_wrap
 * produced, already extracted from a decoded serial frame by the
 * caller). On XRCE_SECURE_OK, out_plain/out_plain_len (via their pointers)
 * alias into `secure_payload` (no copy) and ctx->last_recv_counter is
 * advanced. On
 * any other result, ctx is unchanged and the frame must be dropped. */
xrce_secure_result_t xrce_secure_unwrap(xrce_secure_ctx_t *ctx, const uint8_t *secure_payload,
                                         size_t secure_payload_len, const uint8_t **out_plain,
                                         size_t *out_plain_len);

#endif /* XRCE_SECURE_TRANSPORT_H */
