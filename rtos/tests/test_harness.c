/* Phase 7 milestone: a small test harness that boots the kernel fresh
 * for each test case (via scheduler_reset()) and asserts scheduling
 * behavior, rather than one long-lived scheduler_run() per binary. */

#include <assert.h>
#include <stdio.h>

#include "../src/kernel.h"

typedef void (*test_case_fn)(void);

static int g_tests_run;

static void run_case(const char *name, test_case_fn fn) {
    printf("[case] %s\n", name);
    fn();
    scheduler_reset();
    g_tests_run++;
}

/* Case 1: two equal-priority tasks round-robin fairly. */
static int g_rr_counts[2];

static void rr_worker(void *arg) {
    int idx = *(int *)arg;
    for (int i = 0; i < 3; i++) {
        g_rr_counts[idx]++;
        task_yield();
    }
}

static void case_round_robin(void) {
    int idx0 = 0, idx1 = 1;
    g_rr_counts[0] = g_rr_counts[1] = 0;

    task_t *a = task_spawn("a", rr_worker, &idx0, 0);
    task_t *b = task_spawn("b", rr_worker, &idx1, 0);
    scheduler_run();

    assert(g_rr_counts[0] == 3);
    assert(g_rr_counts[1] == 3);
    task_destroy(a);
    task_destroy(b);
}

/* Case 2: a higher-priority task always runs before a lower-priority one
 * that's ready at the same time. */
static int g_seq_counter;
static int g_low_seq, g_high_seq;

static void seq_low(void *arg) {
    (void)arg;
    g_low_seq = ++g_seq_counter;
}

static void seq_high(void *arg) {
    (void)arg;
    g_high_seq = ++g_seq_counter;
}

static void case_priority_order(void) {
    g_seq_counter = 0;
    g_low_seq = 0;
    g_high_seq = 0;

    task_t *low = task_spawn("low", seq_low, NULL, 1);
    task_t *high = task_spawn("high", seq_high, NULL, 10);
    scheduler_run();

    assert(g_high_seq == 1); /* high ran first despite being spawned second */
    assert(g_low_seq == 2);
    task_destroy(low);
    task_destroy(high);
}

/* Case 3: scheduler_reset() really does start clean -- nothing left over
 * from a previous case can run again or get double-counted. */
static int g_solo_ran;

static void solo_worker(void *arg) {
    (void)arg;
    g_solo_ran++;
}

static void case_fresh_boot(void) {
    g_solo_ran = 0;
    task_t *t = task_spawn("solo", solo_worker, NULL, 0);
    scheduler_run();
    assert(g_solo_ran == 1);
    task_destroy(t);
}

int main(void) {
    run_case("round robin fairness", case_round_robin);
    run_case("priority ordering", case_priority_order);
    run_case("fresh boot per case", case_fresh_boot);

    printf("PASS: %d test cases ran, each against a freshly reset scheduler\n",
           g_tests_run);
    return 0;
}
