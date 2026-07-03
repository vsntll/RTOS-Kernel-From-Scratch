/* Runs the classic priority inversion scenario once with priority
 * inheritance disabled (showing the bug) and once with it enabled
 * (showing the fix), printing timing for each so the difference is
 * visible instead of just asserted. */

#include <stdio.h>

#include "../src/kernel.h"
#include "../src/sync.h"

#define TICK_MS 5
#define MEDIUM_TIMEOUT_TICKS 40
#define LOW_WORK_ITERS 150000000L

static mutex_t g_mutex;
static volatile int g_stop_medium;
static long g_low_work_remaining;
static uint64_t g_scenario_start_tick;
static uint64_t g_high_done_tick;

static void medium_task(void *arg);
static void high_task(void *arg);

static void low_task(void *arg) {
    (void)arg;
    printf("  [low]    acquiring mutex\n");
    mutex_lock(&g_mutex);
    printf("  [low]    holding mutex, spawning medium + high\n");
    task_spawn("medium", medium_task, NULL, 5);
    task_spawn("high", high_task, NULL, 10);
    while (g_low_work_remaining > 0) {
        g_low_work_remaining--;
    }
    printf("  [low]    critical section done, releasing mutex\n");
    mutex_unlock(&g_mutex);
}

static void medium_task(void *arg) {
    (void)arg;
    printf("  [medium] running (never touches the mutex, just outranks low)\n");
    while (!g_stop_medium) {
        if (scheduler_tick_count() - g_scenario_start_tick > MEDIUM_TIMEOUT_TICKS) {
            printf("  [medium] giving up after %d ticks with no progress from high\n",
                   MEDIUM_TIMEOUT_TICKS);
            break;
        }
    }
}

static void high_task(void *arg) {
    (void)arg;
    printf("  [high]   requesting mutex\n");
    mutex_lock(&g_mutex);
    mutex_unlock(&g_mutex);
    g_high_done_tick = scheduler_tick_count();
    printf("  [high]   got the mutex and finished after %llu ticks\n",
           (unsigned long long)(g_high_done_tick - g_scenario_start_tick));
    g_stop_medium = 1;
}

static void run_scenario(const char *label, int inheritance_enabled) {
    printf("\n=== %s ===\n", label);
    mutex_init(&g_mutex, inheritance_enabled);
    g_stop_medium = 0;
    g_low_work_remaining = LOW_WORK_ITERS;
    g_high_done_tick = 0;
    g_scenario_start_tick = scheduler_tick_count();

    task_spawn("low", low_task, NULL, 1);
    scheduler_run();
}

int main(void) {
    scheduler_enable_preemption(TICK_MS, 1);

    run_scenario("Without priority inheritance (inversion)", 0);
    run_scenario("With priority inheritance (fixed)", 1);

    scheduler_disable_preemption();
    return 0;
}
