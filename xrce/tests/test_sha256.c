/* Phase 11 (security layer): SHA-256 checked byte-exact against NIST's
 * own published example message digests (FIPS 180-4's "Appendix B"-style
 * one-block/multi-block/long-message vectors, the same ones every SHA-256
 * implementation is conventionally checked against), not just round-trips
 * or self-consistency. */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../include/xrce/sha256.h"

typedef void (*test_case_fn)(void);
static int g_tests_run;

static void run_case(const char *name, test_case_fn fn) {
    printf("[case] %s\n", name);
    fn();
    g_tests_run++;
}

static void assert_digest(const uint8_t *got, const char *expected_hex) {
    char hex[XRCE_SHA256_DIGEST_LEN * 2 + 1];
    for (int i = 0; i < XRCE_SHA256_DIGEST_LEN; i++) {
        snprintf(hex + i * 2, 3, "%02x", got[i]);
    }
    if (strcmp(hex, expected_hex) != 0) {
        printf("  expected %s\n  got      %s\n", expected_hex, hex);
        assert(0 && "digest mismatch");
    }
}

/* NIST-published: SHA-256("") */
static void case_empty_string(void) {
    uint8_t out[XRCE_SHA256_DIGEST_LEN];
    xrce_sha256((const uint8_t *)"", 0, out);
    assert_digest(out, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

/* NIST-published one-block message: SHA-256("abc") */
static void case_one_block(void) {
    uint8_t out[XRCE_SHA256_DIGEST_LEN];
    xrce_sha256((const uint8_t *)"abc", 3, out);
    assert_digest(out, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

/* NIST-published multi-block message (two 512-bit blocks after padding):
 * SHA-256("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") */
static void case_two_block(void) {
    const char *msg = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    uint8_t out[XRCE_SHA256_DIGEST_LEN];
    xrce_sha256((const uint8_t *)msg, strlen(msg), out);
    assert_digest(out, "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
}

/* Same two-block vector, but fed via update() in several small chunks
 * rather than one xrce_sha256() call -- proves the streaming
 * init/update/update/.../final path (what xrce/src/hmac.c actually uses)
 * gives the identical digest to the one-shot wrapper, not just that the
 * wrapper itself is correct. */
static void case_streamed_matches_one_shot(void) {
    const char *msg = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    size_t len = strlen(msg);

    xrce_sha256_ctx_t ctx;
    xrce_sha256_init(&ctx);
    size_t chunk = 7;
    for (size_t off = 0; off < len; off += chunk) {
        size_t n = (len - off < chunk) ? (len - off) : chunk;
        xrce_sha256_update(&ctx, (const uint8_t *)msg + off, n);
    }
    uint8_t out[XRCE_SHA256_DIGEST_LEN];
    xrce_sha256_final(&ctx, out);
    assert_digest(out, "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
}

/* NIST-published: one million repetitions of "a" -- exercises padding
 * across many block boundaries, not just one or two. */
static void case_million_a(void) {
    xrce_sha256_ctx_t ctx;
    xrce_sha256_init(&ctx);
    uint8_t chunk[1000];
    memset(chunk, 'a', sizeof(chunk));
    for (int i = 0; i < 1000; i++) {
        xrce_sha256_update(&ctx, chunk, sizeof(chunk));
    }
    uint8_t out[XRCE_SHA256_DIGEST_LEN];
    xrce_sha256_final(&ctx, out);
    assert_digest(out, "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

int main(void) {
    run_case("empty string", case_empty_string);
    run_case("one-block \"abc\"", case_one_block);
    run_case("two-block vector", case_two_block);
    run_case("streamed update() matches one-shot", case_streamed_matches_one_shot);
    run_case("one million 'a' characters", case_million_a);
    printf("%d test case(s) passed\n", g_tests_run);
    return 0;
}
