#ifndef RTOS_SCHEDULER_H
#define RTOS_SCHEDULER_H

#include <stdint.h>

#include "task.h"

#define SCHED_MAX_TASKS 32

/* Registers a task with the ready queue. Safe to call before
 * scheduler_run() starts, or from within an already-running task. */
void scheduler_add_task(task_t *task);

/* The context a task's return_ctx (see task_create) should point at so
 * that a task falling off the end of its entry function resumes the
 * scheduler loop instead of exiting the process. */
ucontext_t *scheduler_return_context(void);

/* The kernel's main loop: round-robins over READY tasks until every
 * registered task has reached TASK_TERMINATED. Never returns early. */
void scheduler_run(void);

/* Current task voluntarily gives up the CPU; the scheduler picks the next
 * READY task round-robin and switches to it. */
void task_yield(void);

task_t *scheduler_current_task(void);

/* Starts a POSIX interval timer that raises SIGALRM every tick_ms
 * milliseconds; every ticks_per_slice ticks, the currently RUNNING task
 * is force-switched out regardless of whether it ever calls task_yield().
 * This is the host-native stand-in for a hardware SysTick ISR forcing a
 * PendSV-style context switch. */
void scheduler_enable_preemption(int tick_ms, int ticks_per_slice);
void scheduler_disable_preemption(void);
uint64_t scheduler_tick_count(void);

/* Blocks the current task until at least `ms` milliseconds of ticks have
 * elapsed. Requires scheduler_enable_preemption() to be active -- ticks
 * (and therefore sleep wakeups) only advance while the timer is running. */
void task_sleep(int ms);

#endif
