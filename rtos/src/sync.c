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

void sem_init(semaphore_t *s, int initial_count) {
    s->count = initial_count;
    s->num_waiters = 0;
}

static void sem_add_waiter(semaphore_t *s, task_t *task) {
    for (int i = 0; i < s->num_waiters; i++) {
        if (s->waiters[i] == task) {
            return;
        }
    }
    if (s->num_waiters < SCHED_MAX_TASKS) {
        s->waiters[s->num_waiters++] = task;
    }
}

void sem_wait(semaphore_t *s) {
    task_t *cur = scheduler_current_task();

    while (s->count <= 0) {
        sem_add_waiter(s, cur);
        cur->state = TASK_BLOCKED;
        task_yield();
    }
    s->count--;
}

void sem_post(semaphore_t *s) {
    s->count++;

    /* Wake every waiter and let them race to re-check the count -- same
     * wake-all-and-recheck approach as mutex_unlock. Simpler than picking
     * exactly one waiter per post, at the cost of some spurious wakeups. */
    for (int i = 0; i < s->num_waiters; i++) {
        s->waiters[i]->state = TASK_READY;
    }
    s->num_waiters = 0;
}

void queue_init(queue_t *q, void **buffer, int capacity) {
    q->buffer = buffer;
    q->capacity = capacity;
    q->head = 0;
    q->tail = 0;
    mutex_init(&q->lock, 1);
    sem_init(&q->slots_empty, capacity);
    sem_init(&q->slots_filled, 0);
}

void queue_send(queue_t *q, void *item) {
    sem_wait(&q->slots_empty);
    mutex_lock(&q->lock);
    q->buffer[q->tail] = item;
    q->tail = (q->tail + 1) % q->capacity;
    mutex_unlock(&q->lock);
    sem_post(&q->slots_filled);
}

void *queue_receive(queue_t *q) {
    sem_wait(&q->slots_filled);
    mutex_lock(&q->lock);
    void *item = q->buffer[q->head];
    q->head = (q->head + 1) % q->capacity;
    mutex_unlock(&q->lock);
    sem_post(&q->slots_empty);
    return item;
}
