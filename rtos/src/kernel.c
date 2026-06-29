#include "kernel.h"

task_t *task_spawn(const char *name, task_entry_t entry, void *arg,
                    int priority) {
    task_t *task = task_create(name, entry, arg, priority, 0,
                                scheduler_return_context());
    if (task) {
        scheduler_add_task(task);
    }
    return task;
}
