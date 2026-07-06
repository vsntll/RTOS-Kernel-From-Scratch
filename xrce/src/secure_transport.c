/* See xrce/include/xrce/secure_transport.h for the wire format, threat
 * model, and why this composes with (rather than modifies)
 * serial_transport.c. */

#include "xrce/secure_transport.h"

#include <string.h>

#include "xrce/hmac.h"

void xrce_secure_init(xrce_secure_ctx_t *ctx, const uint8_t *key, size_t key_len) {
    ctx->key = key;
    ctx->key_len = key_len;
    ctx->send_counter = 0;
    ctx->last_recv_counter = 0;
    ctx->have_received = false;
}

static void put_counter_be(uint8_t *out, uint32_t counter) {
    out[0] = (uint8_t)(counter >> 24);
    out[1] = (uint8_t)(counter >> 16);
    out[2] = (uint8_t)(counter >> 8);
    out[3] = (uint8_t)(counter);
}

static uint32_t get_counter_be(const uint8_t *in) {
    return ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) | ((uint32_t)in[2] << 8) |
           (uint32_t)in[3];
}

/* HMAC input is counter || payload -- computed into a caller-provided
 * scratch buffer rather than two separate xrce_hmac_sha256() calls, since
 * HMAC has no "update with two chunks" API of its own (xrce/include/xrce/
 * hmac.h intentionally keeps HMAC one-shot; only the SHA-256 primitive
 * underneath it streams). Fine here: frames are small (well under 256
 * bytes even for the largest CREATE this project sends, see ros2_demo.c's
 * SEND_FRAME_BUF_LEN), so a fixed-size stack scratch buffer is simpler
 * than plumbing a second streaming API through for one caller. */
#define MAC_INPUT_SCRATCH_LEN 320

static void compute_tag(const xrce_secure_ctx_t *ctx, uint32_t counter, const uint8_t *payload,
                         size_t payload_len, uint8_t tag_out[XRCE_SHA256_DIGEST_LEN]) {
    uint8_t scratch[MAC_INPUT_SCRATCH_LEN];
    put_counter_be(scratch, counter);
    memcpy(scratch + XRCE_SECURE_COUNTER_LEN, payload, payload_len);
    xrce_hmac_sha256(ctx->key, ctx->key_len, scratch, XRCE_SECURE_COUNTER_LEN + payload_len,
                      tag_out);
}

size_t xrce_secure_wrap(xrce_secure_ctx_t *ctx, const uint8_t *payload, size_t payload_len,
                         uint8_t *out, size_t out_cap) {
    if (payload_len > MAC_INPUT_SCRATCH_LEN - XRCE_SECURE_COUNTER_LEN ||
        out_cap < payload_len + XRCE_SECURE_OVERHEAD) {
        return 0;
    }

    uint32_t counter = ctx->send_counter;
    uint8_t full_tag[XRCE_SHA256_DIGEST_LEN];
    compute_tag(ctx, counter, payload, payload_len, full_tag);

    put_counter_be(out, counter);
    memcpy(out + XRCE_SECURE_COUNTER_LEN, full_tag, XRCE_SECURE_TAG_LEN);
    memcpy(out + XRCE_SECURE_OVERHEAD, payload, payload_len);

    ctx->send_counter++;
    return payload_len + XRCE_SECURE_OVERHEAD;
}

xrce_secure_result_t xrce_secure_unwrap(xrce_secure_ctx_t *ctx, const uint8_t *secure_payload,
                                         size_t secure_payload_len, const uint8_t **out_plain,
                                         size_t *out_plain_len) {
    if (secure_payload_len < XRCE_SECURE_OVERHEAD ||
        secure_payload_len - XRCE_SECURE_OVERHEAD > MAC_INPUT_SCRATCH_LEN - XRCE_SECURE_COUNTER_LEN) {
        return XRCE_SECURE_TOO_SHORT;
    }

    uint32_t counter = get_counter_be(secure_payload);
    const uint8_t *recv_tag = secure_payload + XRCE_SECURE_COUNTER_LEN;
    const uint8_t *plain = secure_payload + XRCE_SECURE_OVERHEAD;
    size_t plain_len = secure_payload_len - XRCE_SECURE_OVERHEAD;

    uint8_t expected_tag[XRCE_SHA256_DIGEST_LEN];
    compute_tag(ctx, counter, plain, plain_len, expected_tag);

    /* Constant-time-ish compare over just the truncated tag length: not
     * claiming full side-channel hardening here (this project's threat
     * model is message injection/tampering over a link, not a timing
     * attacker co-located with the gateway), but avoiding the obvious
     * early-exit-on-first-mismatch-byte shortcut costs nothing. */
    uint8_t diff = 0;
    for (size_t i = 0; i < XRCE_SECURE_TAG_LEN; i++) {
        diff |= (uint8_t)(recv_tag[i] ^ expected_tag[i]);
    }
    if (diff != 0) {
        return XRCE_SECURE_BAD_TAG;
    }

    if (ctx->have_received && counter <= ctx->last_recv_counter) {
        return XRCE_SECURE_REPLAYED;
    }

    ctx->last_recv_counter = counter;
    ctx->have_received = true;
    *out_plain = plain;
    *out_plain_len = plain_len;
    return XRCE_SECURE_OK;
}
