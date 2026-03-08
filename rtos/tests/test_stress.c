/* Phase 7 milestone: spawn many short-lived tasks rapidly across several
 * waves, verifying no crashes/corruption (run under valgrind
 * --leak-check=full to also confirm no leaks). Each wave resets the
 * scheduler so the fixed-size ready-queue array never has to hold more
 * than one wave's worth of tasks at a time. */

#include <assert.h>
#include <stdio.h>

#include "../src/kernel.h"

#define TASKS_PER_WAVE 20
#define NUM_WAVES 30

static int g_completed;

static void short_task(void *arg) {
    (void)arg;
    volatile int x = 0;
    for (int i = 0; i < 500; i++) {
        x += i;
        task_yield();
    }
    g_completed++;
}

int main(void) {
    int total_tasks = 0;

    for (int wave = 0; wave < NUM_WAVES; wave++) {
        g_completed = 0;
        task_t *tasks[TASKS_PER_WAVE];

        for (int i = 0; i < TASKS_PER_WAVE; i++) {
            tasks[i] = task_spawn("short", short_task, NULL, 0);
            assert(tasks[i] != NULL);
        }

        scheduler_run();
        assert(g_completed == TASKS_PER_WAVE);

        for (int i = 0; i < TASKS_PER_WAVE; i++) {
            assert(task_canary_ok(tasks[i]));
            task_destroy(tasks[i]);
        }

        scheduler_reset();
        total_tasks += TASKS_PER_WAVE;
    }

    printf("PASS: %d short-lived tasks across %d waves, no crashes/corruption "
           "(re-run under valgrind --leak-check=full to confirm no leaks)\n",
           total_tasks, NUM_WAVES);
    return 0;
}
