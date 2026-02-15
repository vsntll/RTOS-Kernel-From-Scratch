#define _XOPEN_SOURCE 700

/* Phase 1 milestone: two tasks that increment a counter and print,
 * manually switching between them by calling context_switch() directly.
 * There is no scheduler yet -- the tasks themselves decide who runs next. */

#include <stdio.h>
#include <ucontext.h>

#include "../src/task.h"

static task_t *g_task_a;
static task_t *g_task_b;
static ucontext_t g_main_ctx;
static int g_remaining_prints = 10;

static void task_a_entry(void *arg) {
    (void)arg;
    int counter = 0;
    while (g_remaining_prints > 0) {
        counter++;
        printf("[task A] counter=%d\n", counter);
        g_remaining_prints--;
        if (g_remaining_prints <= 0) {
            break;
        }
        context_switch(g_task_a, g_task_b);
    }
}

static void task_b_entry(void *arg) {
    (void)arg;
    int counter = 0;
    while (g_remaining_prints > 0) {
        counter++;
        printf("[task B] counter=%d\n", counter);
        g_remaining_prints--;
        if (g_remaining_prints <= 0) {
            break;
        }
        context_switch(g_task_b, g_task_a);
    }
}

int main(void) {
    g_task_a = task_create("A", task_a_entry, NULL, 0, 0);
    g_task_b = task_create("B", task_b_entry, NULL, 0, 0);

    /* Whichever task is running when g_remaining_prints hits zero just
     * returns from its entry function; uc_link sends control back here. */
    g_task_a->context.uc_link = &g_main_ctx;
    g_task_b->context.uc_link = &g_main_ctx;

    printf("Phase 1 demo: manual context switching, no scheduler\n");
    swapcontext(&g_main_ctx, &g_task_a->context);

    printf("Both tasks finished.\n");
    task_destroy(g_task_a);
    task_destroy(g_task_b);
    return 0;
}
