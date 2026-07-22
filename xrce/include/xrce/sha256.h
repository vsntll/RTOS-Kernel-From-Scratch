/* SHA-256 (FIPS 180-4), from scratch -- same policy as the rest of xrce/:
 * implemented against the published spec's exact constants/algorithm, then
 * checked byte-exact against NIST's own published test vectors (see
 * xrce/tests/test_sha256.c), not just self-consistency. Portable C, no
 * libc beyond memcpy (already required by xrce/ elsewhere, and already
 * supplied on the freestanding ARM build by rtos/arm/libc_shim.c).
 *
 * Streaming (init/update/final) rather than one-shot, since Phase 11's
 * xrce/src/hmac.c needs to run SHA-256 over inner and outer padded keys
 * plus message data as several separate chunks. */

#ifndef XRCE_SHA256_H
#define XRCE_SHA256_H

#include <stddef.h>
#include <stdint.h>

#define XRCE_SHA256_DIGEST_LEN 32
#define XRCE_SHA256_BLOCK_LEN 64

typedef struct {
    uint32_t state[8];
    uint64_t total_len; /* total message bytes fed so far, for the length suffix */
    uint8_t buf[XRCE_SHA256_BLOCK_LEN];
    size_t buf_len; /* bytes currently held in buf, always < XRCE_SHA256_BLOCK_LEN */
} xrce_sha256_ctx_t;

void xrce_sha256_init(xrce_sha256_ctx_t *ctx);
void xrce_sha256_update(xrce_sha256_ctx_t *ctx, const uint8_t *data, size_t len);
void xrce_sha256_final(xrce_sha256_ctx_t *ctx, uint8_t out[XRCE_SHA256_DIGEST_LEN]);

/* Convenience one-shot wrapper over the three calls above. */
void xrce_sha256(const uint8_t *data, size_t len, uint8_t out[XRCE_SHA256_DIGEST_LEN]);

#endif /* XRCE_SHA256_H */
