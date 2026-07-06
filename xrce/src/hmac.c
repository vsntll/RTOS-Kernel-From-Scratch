/* HMAC-SHA256 (RFC 2104 section 2 / FIPS 198-1) -- see
 * xrce/include/xrce/hmac.h. */

#include "xrce/hmac.h"

#include <string.h>

void xrce_hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *data, size_t data_len,
                       uint8_t out[XRCE_SHA256_DIGEST_LEN]) {
    uint8_t key_block[XRCE_SHA256_BLOCK_LEN];

    /* RFC 2104 step (1)/(2): keys longer than one block are themselves
     * hashed down to digest length first; shorter keys are zero-padded
     * out to block length. Both cases end with a full block-length key. */
    if (key_len > XRCE_SHA256_BLOCK_LEN) {
        uint8_t hashed_key[XRCE_SHA256_DIGEST_LEN];
        xrce_sha256(key, key_len, hashed_key);
        memcpy(key_block, hashed_key, XRCE_SHA256_DIGEST_LEN);
        memset(key_block + XRCE_SHA256_DIGEST_LEN, 0,
               XRCE_SHA256_BLOCK_LEN - XRCE_SHA256_DIGEST_LEN);
    } else {
        memcpy(key_block, key, key_len);
        memset(key_block + key_len, 0, XRCE_SHA256_BLOCK_LEN - key_len);
    }

    uint8_t ipad[XRCE_SHA256_BLOCK_LEN];
    uint8_t opad[XRCE_SHA256_BLOCK_LEN];
    for (size_t i = 0; i < XRCE_SHA256_BLOCK_LEN; i++) {
        ipad[i] = key_block[i] ^ 0x36;
        opad[i] = key_block[i] ^ 0x5c;
    }

    /* inner = SHA256(ipad || data) */
    uint8_t inner_digest[XRCE_SHA256_DIGEST_LEN];
    xrce_sha256_ctx_t ctx;
    xrce_sha256_init(&ctx);
    xrce_sha256_update(&ctx, ipad, XRCE_SHA256_BLOCK_LEN);
    xrce_sha256_update(&ctx, data, data_len);
    xrce_sha256_final(&ctx, inner_digest);

    /* out = SHA256(opad || inner) */
    xrce_sha256_init(&ctx);
    xrce_sha256_update(&ctx, opad, XRCE_SHA256_BLOCK_LEN);
    xrce_sha256_update(&ctx, inner_digest, XRCE_SHA256_DIGEST_LEN);
    xrce_sha256_final(&ctx, out);
}
