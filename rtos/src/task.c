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
                     int priority, size_t stack_size) {
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
    task->context.uc_stack.ss_sp = task->stack;
    task->context.uc_stack.ss_size = task->stack_size;
    /* Left NULL: whoever queues this task (scheduler, or a manual demo)
     * decides where control goes if the task's entry function returns. */
    task->context.uc_link = NULL;

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
