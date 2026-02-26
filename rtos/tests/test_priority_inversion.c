/* Phase 4 milestone: 3 tasks (low, medium, high priority) sharing a mutex.
 * Show inversion happening (medium blocks high indirectly by starving the
 * low task that holds the lock), then fix it with priority inheritance.
 *
 * Both scenarios run sequentially in the same process: the scheduler's
 * ready-queue array only grows, and pick_next_ready()/any_task_active()
 * only look at task state, so registering a fresh batch of tasks after
 * the first scheduler_run() returns works cleanly without needing a
 * kernel reset (that lands later, in Phase 7's test harness). */

#include <assert.h>
#include <stdio.h>

#include "../src/kernel.h"
#include "../src/sync.h"

#define TICK_MS 5
#define MEDIUM_TIMEOUT_TICKS 40
#define LOW_WORK_ITERS 150000000L

static mutex_t g_mutex;
static volatile int g_stop_medium;
static volatile int g_medium_timed_out;
static long g_low_work_remaining;
static uint64_t g_scenario_start_tick;
static uint64_t g_high_done_tick;
static int g_high_done;

static void medium_task(void *arg);
static void high_task(void *arg);

static void low_task(void *arg) {
    (void)arg;
    /* Lock first, *then* spawn medium/high -- otherwise the scheduler's
     * priority ordering would run high (and grab the uncontended mutex)
     * before low ever gets a turn, and there'd be no inversion to show. */
    mutex_lock(&g_mutex);
    task_spawn("medium", medium_task, NULL, 5);
    task_spawn("high", high_task, NULL, 10);
    while (g_low_work_remaining > 0) {
        g_low_work_remaining--;
    }
    mutex_unlock(&g_mutex);
}

static void medium_task(void *arg) {
    (void)arg;
    while (!g_stop_medium) {
        if (scheduler_tick_count() - g_scenario_start_tick > MEDIUM_TIMEOUT_TICKS) {
            g_medium_timed_out = 1;
            break;
        }
    }
}

static void high_task(void *arg) {
    (void)arg;
    mutex_lock(&g_mutex);
    mutex_unlock(&g_mutex);
    g_high_done_tick = scheduler_tick_count();
    g_high_done = 1;
    g_stop_medium = 1;
}

static void run_scenario(int inheritance_enabled) {
    mutex_init(&g_mutex, inheritance_enabled);
    g_stop_medium = 0;
    g_medium_timed_out = 0;
    g_low_work_remaining = LOW_WORK_ITERS;
    g_high_done_tick = 0;
    g_high_done = 0;
    g_scenario_start_tick = scheduler_tick_count();

    task_spawn("low", low_task, NULL, 1);

    scheduler_run();
}

int main(void) {
    scheduler_enable_preemption(TICK_MS, 1);

    /* Scenario 1: no priority inheritance -- inversion should occur.
     * Medium (higher priority than low, unrelated to the mutex) hogs the
     * CPU and starves low out of finishing its critical section, so
     * medium's own timeout is what ends up ending the standoff, not high
     * completing on time. */
    run_scenario(0);
    printf("[no inheritance] medium_timed_out=%d high_done=%d ticks=%llu\n",
           g_medium_timed_out, g_high_done,
           (unsigned long long)(g_high_done_tick - g_scenario_start_tick));
    assert(g_medium_timed_out == 1);
    assert(g_high_done == 1); /* still completes eventually once medium gives up */

    /* Scenario 2: priority inheritance enabled -- low inherits high's
     * priority the moment high blocks on the mutex, so low outranks
     * medium immediately and finishes its critical section right away. */
    run_scenario(1);
    printf("[with inheritance] medium_timed_out=%d high_done=%d ticks=%llu\n",
           g_medium_timed_out, g_high_done,
           (unsigned long long)(g_high_done_tick - g_scenario_start_tick));
    assert(g_medium_timed_out == 0);
    assert(g_high_done == 1);

    scheduler_disable_preemption();

    printf("PASS: priority inversion demonstrated, then fixed via inheritance\n");
    return 0;
}
