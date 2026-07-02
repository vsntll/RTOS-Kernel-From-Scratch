#ifndef RTOS_SYNC_H
#define RTOS_SYNC_H

#include "scheduler.h"
#include "task.h"

typedef struct {
    int locked;
    task_t *owner;
    int owner_saved_priority;
    int inheritance_enabled;
    task_t *waiters[SCHED_MAX_TASKS];
    int num_waiters;
} mutex_t;

/* enable_priority_inheritance: when a higher-priority task blocks on a
 * mutex held by a lower-priority one, temporarily boost the holder's
 * priority to the waiter's so it can't be starved out by anything in
 * between -- the classic fix for priority inversion. Passing 0 disables
 * this (useful for demonstrating the inversion bug itself). */
void mutex_init(mutex_t *m, int enable_priority_inheritance);
void mutex_lock(mutex_t *m);
void mutex_unlock(mutex_t *m);

#endif
