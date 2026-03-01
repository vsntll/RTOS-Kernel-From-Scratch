/* Phase 5 milestone: classic producer-consumer test -- one task produces
 * data into a queue, another consumes it. Asserts every item arrives
 * exactly once, in order, with nothing dropped or duplicated.
 *
 * There's no real OS-level concurrency here (cooperative/preemptive green
 * threads all run on one OS thread), so -fsanitize=thread has nothing to
 * instrument; the correctness check that actually matters for this
 * architecture is the data-integrity assertion below, run under a queue
 * small enough (relative to item count) to force both producer and
 * consumer to block repeatedly. */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../src/kernel.h"
#include "../src/sync.h"

#define QUEUE_CAPACITY 3
#define ITEM_COUNT 50

static void *g_queue_storage[QUEUE_CAPACITY];
static queue_t g_queue;
static long g_received[ITEM_COUNT];
static int g_received_count;

static void producer_task(void *arg) {
    (void)arg;
    for (intptr_t i = 0; i < ITEM_COUNT; i++) {
        queue_send(&g_queue, (void *)i);
        task_yield();
    }
}

static void consumer_task(void *arg) {
    (void)arg;
    for (int i = 0; i < ITEM_COUNT; i++) {
        void *item = queue_receive(&g_queue);
        g_received[g_received_count++] = (long)(intptr_t)item;
        task_yield();
    }
}

int main(void) {
    queue_init(&g_queue, g_queue_storage, QUEUE_CAPACITY);

    task_spawn("producer", producer_task, NULL, 0);
    task_spawn("consumer", consumer_task, NULL, 0);

    scheduler_run();

    assert(g_received_count == ITEM_COUNT);
    for (int i = 0; i < ITEM_COUNT; i++) {
        assert(g_received[i] == i); /* in order, nothing dropped/duplicated */
    }

    printf("PASS: %d items sent through a capacity-%d queue, all arrived "
           "in order\n",
           ITEM_COUNT, QUEUE_CAPACITY);
    return 0;
}
