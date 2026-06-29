#define _XOPEN_SOURCE 700

#include "scheduler.h"

#include <stddef.h>
#include <ucontext.h>

static task_t *g_tasks[SCHED_MAX_TASKS];
static int g_num_tasks = 0;
static int g_current_idx = -1;
static ucontext_t g_sched_ctx;

void scheduler_add_task(task_t *task) {
    if (g_num_tasks >= SCHED_MAX_TASKS) {
        return;
    }
    g_tasks[g_num_tasks++] = task;
}

ucontext_t *scheduler_return_context(void) {
    return &g_sched_ctx;
}

task_t *scheduler_current_task(void) {
    if (g_current_idx < 0) {
        return NULL;
    }
    return g_tasks[g_current_idx];
}

static int pick_next_ready(void) {
    if (g_num_tasks == 0) {
        return -1;
    }
    int start = (g_current_idx < 0) ? 0 : (g_current_idx + 1) % g_num_tasks;
    for (int i = 0; i < g_num_tasks; i++) {
        int idx = (start + i) % g_num_tasks;
        if (g_tasks[idx]->state == TASK_READY) {
            return idx;
        }
    }
    return -1;
}

static int any_task_active(void) {
    for (int i = 0; i < g_num_tasks; i++) {
        if (g_tasks[i]->state != TASK_TERMINATED) {
            return 1;
        }
    }
    return 0;
}

void scheduler_run(void) {
    while (any_task_active()) {
        int next = pick_next_ready();
        if (next < 0) {
            /* Every remaining task is BLOCKED; nothing to do until one
             * wakes up (sync primitives / timers flip state elsewhere). */
            continue;
        }
        g_current_idx = next;
        g_tasks[next]->state = TASK_RUNNING;
        swapcontext(&g_sched_ctx, &g_tasks[next]->context);
    }
    g_current_idx = -1;
}

void task_yield(void) {
    if (g_current_idx < 0) {
        return;
    }
    task_t *cur = g_tasks[g_current_idx];
    if (cur->state == TASK_RUNNING) {
        cur->state = TASK_READY;
    }
    swapcontext(&cur->context, &g_sched_ctx);
}
