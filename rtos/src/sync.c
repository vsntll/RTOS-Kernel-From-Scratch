#include "sync.h"

#include <stddef.h>

void mutex_init(mutex_t *m, int enable_priority_inheritance) {
    m->locked = 0;
    m->owner = NULL;
    m->owner_saved_priority = 0;
    m->inheritance_enabled = enable_priority_inheritance;
    m->num_waiters = 0;
}

static void add_waiter(mutex_t *m, task_t *task) {
    for (int i = 0; i < m->num_waiters; i++) {
        if (m->waiters[i] == task) {
            return;
        }
    }
    if (m->num_waiters < SCHED_MAX_TASKS) {
        m->waiters[m->num_waiters++] = task;
    }
}

void mutex_lock(mutex_t *m) {
    task_t *cur = scheduler_current_task();

    while (m->locked) {
        if (m->inheritance_enabled && m->owner &&
            cur->priority > m->owner->priority) {
            /* Boost: the lock holder inherits the waiter's priority so it
             * can't be preempted by anything in between, closing the
             * window that causes priority inversion. */
            m->owner->priority = cur->priority;
        }
        add_waiter(m, cur);
        cur->state = TASK_BLOCKED;
        task_yield();
    }

    m->locked = 1;
    m->owner = cur;
    m->owner_saved_priority = cur->priority;
}

void mutex_unlock(mutex_t *m) {
    task_t *cur = scheduler_current_task();

    cur->priority = m->owner_saved_priority;
    m->locked = 0;
    m->owner = NULL;

    for (int i = 0; i < m->num_waiters; i++) {
        m->waiters[i]->state = TASK_READY;
    }
    m->num_waiters = 0;
}
