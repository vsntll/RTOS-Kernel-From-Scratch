/* Phase 2 milestone: 3-4 tasks each printing on a loop with task_yield()
 * calls -- output should interleave correctly, no crashes, no memory
 * corruption (run this under valgrind / ASan). */

#include <assert.h>
#include <stdio.h>

#include "../src/kernel.h"

#define NUM_TASKS 4
#define ITERATIONS 5

static int g_run_count[NUM_TASKS];

typedef struct {
    int index;
} task_arg_t;

static void worker(void *arg) {
    task_arg_t *a = (task_arg_t *)arg;
    for (int i = 0; i < ITERATIONS; i++) {
        g_run_count[a->index]++;
        printf("[task %d] iteration %d\n", a->index, i + 1);
        task_yield();
    }
}

int main(void) {
    task_arg_t args[NUM_TASKS];

    for (int i = 0; i < NUM_TASKS; i++) {
        args[i].index = i;
        char name[16];
        snprintf(name, sizeof(name), "worker%d", i);
        task_t *t = task_spawn(name, worker, &args[i], 0);
        assert(t != NULL);
    }

    scheduler_run();

    for (int i = 0; i < NUM_TASKS; i++) {
        assert(g_run_count[i] == ITERATIONS);
    }

    printf("PASS: %d tasks completed %d cooperative iterations each\n",
           NUM_TASKS, ITERATIONS);
    return 0;
}
