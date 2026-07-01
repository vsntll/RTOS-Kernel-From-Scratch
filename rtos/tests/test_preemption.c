/* Phase 3 milestone: a task in an infinite loop that never yields still
 * gets preempted, and other tasks still get to run. */

#include <assert.h>
#include <stdio.h>

#include "../src/kernel.h"

#define TICK_MS 5
#define STOP_AFTER_TICKS 200

static volatile int g_should_stop = 0;
static long g_hog_iterations = 0;
static long g_polite_iterations = 0;

/* Pure busy loop -- deliberately never calls task_yield(). Without
 * preemption this would starve every other task forever. */
static void hog_task(void *arg) {
    (void)arg;
    while (!g_should_stop) {
        g_hog_iterations++;
        if (scheduler_tick_count() >= STOP_AFTER_TICKS) {
            g_should_stop = 1;
        }
    }
}

static void polite_task(void *arg) {
    (void)arg;
    while (!g_should_stop) {
        g_polite_iterations++;
        task_yield();
    }
}

int main(void) {
    task_spawn("hog", hog_task, NULL, 0);
    task_spawn("polite", polite_task, NULL, 0);

    scheduler_enable_preemption(TICK_MS, 1);
    scheduler_run();
    scheduler_disable_preemption();

    printf("ticks=%llu hog_iterations=%ld polite_iterations=%ld\n",
           (unsigned long long)scheduler_tick_count(), g_hog_iterations,
           g_polite_iterations);

    /* If preemption never kicked in, polite_task would never run at all
     * (hog_task never yields), so this is the load-bearing assertion. */
    assert(g_polite_iterations > 0);
    assert(g_hog_iterations > 0);
    assert(scheduler_tick_count() >= STOP_AFTER_TICKS);

    printf("PASS: non-yielding task was preempted; other tasks still ran\n");
    return 0;
}
