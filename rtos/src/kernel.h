#ifndef RTOS_KERNEL_H
#define RTOS_KERNEL_H

#include "task.h"
#include "scheduler.h"

/* Creates a task with the default stack size and immediately registers it
 * with the scheduler's ready queue -- the usual entry point for spawning
 * a task, versus calling task_create()+scheduler_add_task() separately. */
task_t *task_spawn(const char *name, task_entry_t entry, void *arg,
                    int priority);

#endif
