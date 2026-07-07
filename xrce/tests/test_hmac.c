/* Phase 11 (security layer): HMAC-SHA256 checked byte-exact against
 * RFC 4231's published test cases -- Test Case 1 (short key), Test Case 2
 * (key shorter than "Jefe", the RFC's own deliberately odd short-key
 * example), and Test Case 6 (a 131-byte key, longer than SHA-256's
 * 64-byte block size, exercising the RFC 2104 key-hashing-down path
 * xrce/src/hmac.c's `key_len > XRCE_SHA256_BLOCK_LEN` branch handles).
 * Independently cross-checked against Python's `hmac`+`hashlib` before
 * being hardcoded here, same ground-truth-over-memory policy as the rest
 * of xrce/. */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../include/xrce/hmac.h"

typedef void (*test_case_fn)(void);
static int g_tests_run;

static void run_case(const char *name, test_case_fn fn) {
    printf("[case] %s\n", name);
    fn();
    g_tests_run++;
}

static void assert_mac(const uint8_t *got, const char *expected_hex) {
    char hex[XRCE_SHA256_DIGEST_LEN * 2 + 1];
    for (int i = 0; i < XRCE_SHA256_DIGEST_LEN; i++) {
        snprintf(hex + i * 2, 3, "%02x", got[i]);
    }
    if (strcmp(hex, expected_hex) != 0) {
        printf("  expected %s\n  got      %s\n", expected_hex, hex);
        assert(0 && "HMAC mismatch");
    }
}

/* RFC 4231 Test Case 1: 20-byte key. */
static void case_rfc4231_tc1(void) {
    uint8_t key[20];
    memset(key, 0x0b, sizeof(key));
    const uint8_t *data = (const uint8_t *)"Hi There";
    uint8_t out[XRCE_SHA256_DIGEST_LEN];
    xrce_hmac_sha256(key, sizeof(key), data, strlen((const char *)data), out);
    assert_mac(out, "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7");
}

/* RFC 4231 Test Case 2: short ASCII key ("Jefe"), the RFC's own example
 * of a key shorter than the digest length. */
static void case_rfc4231_tc2(void) {
    const uint8_t *key = (const uint8_t *)"Jefe";
    const uint8_t *data = (const uint8_t *)"what do ya want for nothing?";
    uint8_t out[XRCE_SHA256_DIGEST_LEN];
    xrce_hmac_sha256(key, 4, data, strlen((const char *)data), out);
    assert_mac(out, "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843");
}

/* RFC 4231 Test Case 6: 131-byte key, longer than SHA-256's 64-byte
 * block -- exercises the "hash the key down first" branch. */
static void case_rfc4231_tc6_long_key(void) {
    uint8_t key[131];
    memset(key, 0xaa, sizeof(key));
    const uint8_t *data =
        (const uint8_t *)"Test Using Larger Than Block-Size Key - Hash Key First";
    uint8_t out[XRCE_SHA256_DIGEST_LEN];
    xrce_hmac_sha256(key, sizeof(key), data, strlen((const char *)data), out);
    assert_mac(out, "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54");
}

/* Not an RFC vector: confirms a single differing key byte produces a
 * completely different MAC rather than, say, silently truncating/ignoring
 * part of the key -- the actual property xrce/src/secure_transport.c
 * depends on for rejecting a wrong-key sender. */
static void case_wrong_key_differs(void) {
    uint8_t key_a[16], key_b[16];
    memset(key_a, 0x42, sizeof(key_a));
    memset(key_b, 0x42, sizeof(key_b));
    key_b[15] ^= 0x01;

    const uint8_t *data = (const uint8_t *)"same message";
    uint8_t mac_a[XRCE_SHA256_DIGEST_LEN], mac_b[XRCE_SHA256_DIGEST_LEN];
    xrce_hmac_sha256(key_a, sizeof(key_a), data, strlen((const char *)data), mac_a);
    xrce_hmac_sha256(key_b, sizeof(key_b), data, strlen((const char *)data), mac_b);
    assert(memcmp(mac_a, mac_b, XRCE_SHA256_DIGEST_LEN) != 0);
}

int main(void) {
    run_case("RFC 4231 Test Case 1 (20-byte key)", case_rfc4231_tc1);
    run_case("RFC 4231 Test Case 2 (\"Jefe\")", case_rfc4231_tc2);
    run_case("RFC 4231 Test Case 6 (131-byte key)", case_rfc4231_tc6_long_key);
    run_case("one-bit key difference changes the MAC", case_wrong_key_differs);
    printf("%d test case(s) passed\n", g_tests_run);
    return 0;
}
