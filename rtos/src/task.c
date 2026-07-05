#define _XOPEN_SOURCE 700

#include "task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_next_task_id = 0;

/* makecontext only guarantees portable passing of `int` arguments, so a
 * pointer-sized value has to be split into two ints and reassembled here.
 * This is the standard trick for stashing a `task_t *` in a ucontext_t. */
static void task_trampoline(unsigned int hi, unsigned int lo) {
    uint64_t packed = ((uint64_t)hi << 32) | (uint64_t)lo;
    task_t *task = (task_t *)(uintptr_t)packed;

    task->entry(task->arg);
    task->state = TASK_TERMINATED;
}

task_t *task_create(const char *name, task_entry_t entry, void *arg,
                     int priority, size_t stack_size, ucontext_t *return_ctx) {
    if (stack_size == 0) {
        stack_size = TASK_DEFAULT_STACK_SIZE;
    }

    task_t *task = calloc(1, sizeof(task_t));
    if (!task) {
        return NULL;
    }

    task->stack = malloc(stack_size);
    if (!task->stack) {
        free(task);
        return NULL;
    }
    task->stack_size = stack_size;
    task->entry = entry;
    task->arg = arg;
    task->priority = priority;
    task->state = TASK_READY;
    task->id = g_next_task_id++;

    if (name) {
        strncpy(task->name, name, TASK_NAME_MAX - 1);
    } else {
        snprintf(task->name, TASK_NAME_MAX, "task%d", task->id);
    }

    if (getcontext(&task->context) == -1) {
        free(task->stack);
        free(task);
        return NULL;
    }
    /* Bottom guard only -- see task_canary_ok() for why a "top" guard at
     * this end of the buffer doesn't work (makecontext(), called below,
     * writes into it immediately as part of setting up the initial
     * frame, before the task ever runs once). */
    if (task->stack_size >= 2 * TASK_CANARY_SIZE) {
        memset(task->stack, TASK_CANARY_BYTE, TASK_CANARY_SIZE);
    }

    task->context.uc_stack.ss_sp = task->stack;
    task->context.uc_stack.ss_size = task->stack_size;
    /* Must happen before makecontext() -- see the note on return_ctx in
     * task.h for why setting this afterwards doesn't work. */
    task->context.uc_link = return_ctx;

    uint64_t packed = (uint64_t)(uintptr_t)task;
    unsigned int hi = (unsigned int)(packed >> 32);
    unsigned int lo = (unsigned int)(packed & 0xffffffffu);
    makecontext(&task->context, (void (*)(void))task_trampoline, 2, hi, lo);

    return task;
}

void task_destroy(task_t *task) {
    if (!task) {
        return;
    }
    free(task->stack);
    free(task);
}

void context_switch(task_t *old, task_t *new) {
    swapcontext(&old->context, &new->context);
}

int task_canary_ok(const task_t *task) {
    if (task->stack_size < 2 * TASK_CANARY_SIZE) {
        return 1; /* stack too small to have planted canaries at all */
    }
    /* Only the bottom (low-address) guard is checked. x86_64 stacks grow
     * downward from uc_stack.ss_sp+ss_size, so the bottom is where deep
     * recursion/overflow actually lands -- confirmed by dumping a freshly
     * created, never-run task: makecontext() itself already writes into
     * the last several bytes of the buffer while setting up the initial
     * frame, so a "top" guard there would misfire before the task even
     * runs once, not after real corruption. */
    for (size_t i = 0; i < TASK_CANARY_SIZE; i++) {
        if (task->stack[i] != TASK_CANARY_BYTE) {
            return 0;
        }
    }
    return 1;
}

void task_check_canary_or_abort(const task_t *task) {
    if (!task_canary_ok(task)) {
        fprintf(stderr,
                "FATAL: stack canary corrupted for task '%s' (id=%d) -- "
                "stack overflow\n",
                task->name, task->id);
        abort();
    }
}
