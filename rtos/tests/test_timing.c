/* Phase 6 milestone: task_sleep() blocks a task until its wake-up tick,
 * and per-task run/ready timing stats accumulate sensibly. */

#include <assert.h>
#include <stdio.h>

#include "../src/kernel.h"

#define TICK_MS 5
#define SLEEP_MS 100 /* 20 ticks at TICK_MS=5 */

static task_t *g_sleeper;
static task_t *g_busy;
static uint64_t g_wake_tick;
static volatile int g_stop_busy;

static void sleeper_task(void *arg) {
    (void)arg;
    uint64_t before = scheduler_tick_count();
    task_sleep(SLEEP_MS);
    g_wake_tick = scheduler_tick_count();
    (void)before;
    g_stop_busy = 1;
}

static void busy_task(void *arg) {
    (void)arg;
    /* Pure spin, deliberately never calling task_yield() -- same pattern
     * as test_preemption.c's hog_task. Rapid *voluntary* yields raced
     * against the timer tick during development and intermittently hung
     * (see the note above scheduler.c's block_alarm()); relying purely on
     * preemption here avoids that hazard entirely. */
    while (!g_stop_busy) {
        /* busy */
    }
}

int main(void) {
    scheduler_enable_preemption(TICK_MS, 1);

    g_sleeper = task_spawn("sleeper", sleeper_task, NULL, 0);
    g_busy = task_spawn("busy", busy_task, NULL, 0);

    uint64_t start_tick = scheduler_tick_count();
    scheduler_run();

    scheduler_disable_preemption();

    uint64_t elapsed = g_wake_tick - start_tick;
    printf("sleep requested %dms, woke after %llu ticks (%dms/tick)\n",
           SLEEP_MS, (unsigned long long)elapsed, TICK_MS);
    printf("sleeper: ticks_run=%llu ticks_ready=%llu\n",
           (unsigned long long)g_sleeper->ticks_run,
           (unsigned long long)g_sleeper->ticks_ready);
    printf("busy:    ticks_run=%llu ticks_ready=%llu\n",
           (unsigned long long)g_busy->ticks_run,
           (unsigned long long)g_busy->ticks_ready);

    /* Slept for roughly the right number of ticks (allow scheduling
     * slop -- it can wake a tick or two late, never early). */
    long expected_ticks = SLEEP_MS / TICK_MS;
    assert((long)elapsed >= expected_ticks);
    assert((long)elapsed <= expected_ticks + 5);

    /* The busy task spins the whole time the sleeper is asleep, so it
     * should show far more accumulated RUNNING time than the sleeper. */
    assert(g_busy->ticks_run > g_sleeper->ticks_run);

    task_destroy(g_sleeper);
    task_destroy(g_busy);

    printf("PASS: task_sleep() woke on schedule, timing stats look sane\n");
    return 0;
}
