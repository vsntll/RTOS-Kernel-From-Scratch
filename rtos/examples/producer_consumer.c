/* One producer task pushes integers into a bounded queue; one consumer
 * task pops and prints them. Demonstrates queue_send()/queue_receive()
 * blocking correctly in both directions (producer blocks when the queue
 * is full, consumer blocks when it's empty). */

#include <stdint.h>
#include <stdio.h>

#include "../src/kernel.h"
#include "../src/sync.h"

#define QUEUE_CAPACITY 4
#define ITEM_COUNT 20

static void *g_queue_storage[QUEUE_CAPACITY];
static queue_t g_queue;

static void producer_task(void *arg) {
    (void)arg;
    for (intptr_t i = 1; i <= ITEM_COUNT; i++) {
        printf("[producer] sending %ld\n", (long)i);
        queue_send(&g_queue, (void *)i);
        task_yield();
    }
}

static void consumer_task(void *arg) {
    (void)arg;
    for (int i = 0; i < ITEM_COUNT; i++) {
        void *item = queue_receive(&g_queue);
        printf("[consumer] received %ld\n", (long)(intptr_t)item);
        task_yield();
    }
}

int main(void) {
    queue_init(&g_queue, g_queue_storage, QUEUE_CAPACITY);

    task_spawn("producer", producer_task, NULL, 0);
    task_spawn("consumer", consumer_task, NULL, 0);

    scheduler_run();
    return 0;
}
