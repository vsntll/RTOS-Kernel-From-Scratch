#ifndef RTOS_TASK_H
#define RTOS_TASK_H

#include <stddef.h>
#include <stdint.h>
#include <ucontext.h>

#define TASK_DEFAULT_STACK_SIZE (64 * 1024)
#define TASK_NAME_MAX 16

typedef enum {
    TASK_READY,
    TASK_RUNNING,
    TASK_BLOCKED,
    TASK_TERMINATED
} task_state_t;

typedef void (*task_entry_t)(void *arg);

typedef struct task {
    int id;
    char name[TASK_NAME_MAX];
    ucontext_t context;
    unsigned char *stack;
    size_t stack_size;
    task_state_t state;
    int priority; /* higher value = higher priority */
    task_entry_t entry;
    void *arg;

    /* Set by task_sleep(); 0 means "not sleeping" so the tick handler
     * doesn't confuse a sleeping task with one BLOCKED on a mutex/sem. */
    uint64_t wake_tick;

    /* Timing stats (Phase 6): ticks spent actually RUNNING vs ticks spent
     * READY-but-not-running, sampled once per timer tick. */
    uint64_t ticks_run;
    uint64_t ticks_ready;
} task_t;

/* Allocates a stack and sets up an initial context so that when the task
 * is first switched to, it starts executing `entry(arg)`. The task starts
 * in TASK_READY state; nothing runs until something switches to it.
 *
 * `return_ctx` is where control goes if `entry` ever returns (NULL means
 * the process exits, per ucontext's default uc_link behavior). It must be
 * passed in here rather than set on task->context afterwards: glibc's
 * makecontext() bakes uc_link into the trampoline at call time, so
 * assigning task->context.uc_link post-creation is silently ignored. */
task_t *task_create(const char *name, task_entry_t entry, void *arg,
                     int priority, size_t stack_size, ucontext_t *return_ctx);

void task_destroy(task_t *task);

/* Low-level primitive: saves the caller's register/stack state into `old`
 * and restores `new`'s. No scheduler bookkeeping happens here -- that's
 * layered on top in scheduler.c. */
void context_switch(task_t *old, task_t *new);

#endif
