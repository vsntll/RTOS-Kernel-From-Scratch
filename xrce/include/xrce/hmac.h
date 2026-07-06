/* HMAC-SHA256 (RFC 2104 / FIPS 198-1), built on xrce/include/xrce/sha256.h.
 * Checked byte-exact against RFC 4231's published HMAC-SHA256 test cases
 * (xrce/tests/test_hmac.c), the same "ground truth from the spec's own
 * vectors, not self-consistency" policy as the rest of xrce/. */

#ifndef XRCE_HMAC_H
#define XRCE_HMAC_H

#include <stddef.h>
#include <stdint.h>

#include "xrce/sha256.h"

/* Full, untruncated HMAC-SHA256 -- out must hold XRCE_SHA256_DIGEST_LEN
 * (32) bytes. key_len may be any length (RFC 2104 handles keys both
 * shorter and longer than one block); this project's actual usage
 * (xrce/src/secure_transport.c) always passes a fixed-length pre-shared
 * key, but the implementation doesn't assume that. */
void xrce_hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *data, size_t data_len,
                       uint8_t out[XRCE_SHA256_DIGEST_LEN]);

#endif /* XRCE_HMAC_H */
