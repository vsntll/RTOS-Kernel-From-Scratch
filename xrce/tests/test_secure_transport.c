/* Phase 11 (security layer): exercises xrce_secure_wrap()/unwrap() against
 * the actual failure modes this layer exists to catch -- a flipped bit in
 * transit, a replayed old frame, and a wrong key -- not just the
 * happy-path round trip. Mirrors this project's existing pattern of
 * testing failure injection explicitly (test_serial_transport.c's
 * corruption/resync cases are the direct precedent). */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../include/xrce/secure_transport.h"

typedef void (*test_case_fn)(void);
static int g_tests_run;

static void run_case(const char *name, test_case_fn fn) {
    printf("[case] %s\n", name);
    fn();
    g_tests_run++;
}

static const uint8_t KEY[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                               0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10};

static void case_round_trip(void) {
    xrce_secure_ctx_t sender, receiver;
    xrce_secure_init(&sender, KEY, sizeof(KEY));
    xrce_secure_init(&receiver, KEY, sizeof(KEY));

    const uint8_t payload[] = "hello over the wire";
    uint8_t wrapped[128];
    size_t wrapped_len =
        xrce_secure_wrap(&sender, payload, sizeof(payload), wrapped, sizeof(wrapped));
    assert(wrapped_len == sizeof(payload) + XRCE_SECURE_OVERHEAD);

    const uint8_t *plain;
    size_t plain_len;
    xrce_secure_result_t res =
        xrce_secure_unwrap(&receiver, wrapped, wrapped_len, &plain, &plain_len);
    assert(res == XRCE_SECURE_OK);
    assert(plain_len == sizeof(payload));
    assert(memcmp(plain, payload, sizeof(payload)) == 0);
}

/* Several frames in a row must all verify, with a strictly-increasing
 * counter each time -- proves this isn't a one-shot ctx state fluke. */
static void case_several_frames_in_sequence(void) {
    xrce_secure_ctx_t sender, receiver;
    xrce_secure_init(&sender, KEY, sizeof(KEY));
    xrce_secure_init(&receiver, KEY, sizeof(KEY));

    for (int i = 0; i < 10; i++) {
        uint8_t payload[4] = {(uint8_t)i, 0, 0, 0};
        uint8_t wrapped[64];
        size_t wrapped_len =
            xrce_secure_wrap(&sender, payload, sizeof(payload), wrapped, sizeof(wrapped));
        const uint8_t *plain;
        size_t plain_len;
        xrce_secure_result_t res =
            xrce_secure_unwrap(&receiver, wrapped, wrapped_len, &plain, &plain_len);
        assert(res == XRCE_SECURE_OK);
        assert(plain[0] == (uint8_t)i);
    }
}

/* A single flipped payload bit after wrapping (simulating link corruption
 * or an active injection attempt) must be rejected, not silently
 * accepted -- the actual property this whole layer exists for. */
static void case_tampered_payload_rejected(void) {
    xrce_secure_ctx_t sender, receiver;
    xrce_secure_init(&sender, KEY, sizeof(KEY));
    xrce_secure_init(&receiver, KEY, sizeof(KEY));

    const uint8_t payload[] = {0x11, 0x22, 0x33, 0x44};
    uint8_t wrapped[64];
    size_t wrapped_len =
        xrce_secure_wrap(&sender, payload, sizeof(payload), wrapped, sizeof(wrapped));

    wrapped[wrapped_len - 1] ^= 0x01; /* flip one bit of the inner payload's last byte */

    const uint8_t *plain;
    size_t plain_len;
    xrce_secure_result_t res =
        xrce_secure_unwrap(&receiver, wrapped, wrapped_len, &plain, &plain_len);
    assert(res == XRCE_SECURE_BAD_TAG);
}

/* Flipping a tag byte directly (forging without the key) must also fail --
 * distinct from tampering the payload, exercises the other half of the
 * authenticated blob. */
static void case_tampered_tag_rejected(void) {
    xrce_secure_ctx_t sender, receiver;
    xrce_secure_init(&sender, KEY, sizeof(KEY));
    xrce_secure_init(&receiver, KEY, sizeof(KEY));

    const uint8_t payload[] = {0xaa, 0xbb};
    uint8_t wrapped[64];
    size_t wrapped_len =
        xrce_secure_wrap(&sender, payload, sizeof(payload), wrapped, sizeof(wrapped));

    wrapped[XRCE_SECURE_COUNTER_LEN] ^= 0x80; /* flip a bit inside the tag itself */

    const uint8_t *plain;
    size_t plain_len;
    xrce_secure_result_t res =
        xrce_secure_unwrap(&receiver, wrapped, wrapped_len, &plain, &plain_len);
    assert(res == XRCE_SECURE_BAD_TAG);
}

/* Replaying a previously-accepted, otherwise perfectly valid frame must
 * be rejected on the second delivery -- the counter-freshness check, not
 * the HMAC, is what catches this (the tag itself is completely valid). */
static void case_replay_rejected(void) {
    xrce_secure_ctx_t sender, receiver;
    xrce_secure_init(&sender, KEY, sizeof(KEY));
    xrce_secure_init(&receiver, KEY, sizeof(KEY));

    const uint8_t payload[] = {0x01};
    uint8_t wrapped[64];
    size_t wrapped_len =
        xrce_secure_wrap(&sender, payload, sizeof(payload), wrapped, sizeof(wrapped));

    const uint8_t *plain;
    size_t plain_len;
    assert(xrce_secure_unwrap(&receiver, wrapped, wrapped_len, &plain, &plain_len) ==
           XRCE_SECURE_OK);
    /* Same bytes, fed again (a captured-and-replayed frame, or a duplicate
     * delivery) -- must not succeed a second time. */
    assert(xrce_secure_unwrap(&receiver, wrapped, wrapped_len, &plain, &plain_len) ==
           XRCE_SECURE_REPLAYED);
}

/* A receiver initialized with the wrong key must reject every frame from
 * a legitimate sender using the real key -- proves the key actually
 * matters, not just that mismatched bytes happen to fail. */
static void case_wrong_key_rejected(void) {
    static const uint8_t WRONG_KEY[sizeof(KEY)] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                                     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    xrce_secure_ctx_t sender, receiver;
    xrce_secure_init(&sender, KEY, sizeof(KEY));
    xrce_secure_init(&receiver, WRONG_KEY, sizeof(WRONG_KEY));

    const uint8_t payload[] = {0x42};
    uint8_t wrapped[64];
    size_t wrapped_len =
        xrce_secure_wrap(&sender, payload, sizeof(payload), wrapped, sizeof(wrapped));

    const uint8_t *plain;
    size_t plain_len;
    xrce_secure_result_t res =
        xrce_secure_unwrap(&receiver, wrapped, wrapped_len, &plain, &plain_len);
    assert(res == XRCE_SECURE_BAD_TAG);
}

/* A buffer shorter than the fixed counter+tag overhead can't even be
 * attempted -- must fail cleanly, not read past the end of it. */
static void case_too_short_rejected(void) {
    xrce_secure_ctx_t receiver;
    xrce_secure_init(&receiver, KEY, sizeof(KEY));

    uint8_t too_short[XRCE_SECURE_OVERHEAD - 1];
    memset(too_short, 0, sizeof(too_short));
    const uint8_t *plain;
    size_t plain_len;
    xrce_secure_result_t res =
        xrce_secure_unwrap(&receiver, too_short, sizeof(too_short), &plain, &plain_len);
    assert(res == XRCE_SECURE_TOO_SHORT);
}

int main(void) {
    run_case("round trip", case_round_trip);
    run_case("several frames in sequence", case_several_frames_in_sequence);
    run_case("tampered payload byte is rejected", case_tampered_payload_rejected);
    run_case("tampered tag byte is rejected", case_tampered_tag_rejected);
    run_case("replayed frame is rejected", case_replay_rejected);
    run_case("wrong key is rejected", case_wrong_key_rejected);
    run_case("too-short buffer is rejected", case_too_short_rejected);
    printf("%d test case(s) passed\n", g_tests_run);
    return 0;
}
