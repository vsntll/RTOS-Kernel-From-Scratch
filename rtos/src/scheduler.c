#define _XOPEN_SOURCE 700

#include "scheduler.h"

#include <signal.h>
#include <stddef.h>
#include <string.h>
#include <sys/time.h>
#include <ucontext.h>

static task_t *g_tasks[SCHED_MAX_TASKS];
static int g_num_tasks = 0;
static int g_current_idx = -1;
static ucontext_t g_sched_ctx;

void scheduler_add_task(task_t *task) {
    if (g_num_tasks >= SCHED_MAX_TASKS) {
        return;
    }
    g_tasks[g_num_tasks++] = task;
}

ucontext_t *scheduler_return_context(void) {
    return &g_sched_ctx;
}

task_t *scheduler_current_task(void) {
    if (g_current_idx < 0) {
        return NULL;
    }
    return g_tasks[g_current_idx];
}

static int pick_next_ready(void) {
    if (g_num_tasks == 0) {
        return -1;
    }
    int start = (g_current_idx < 0) ? 0 : (g_current_idx + 1) % g_num_tasks;
    for (int i = 0; i < g_num_tasks; i++) {
        int idx = (start + i) % g_num_tasks;
        if (g_tasks[idx]->state == TASK_READY) {
            return idx;
        }
    }
    return -1;
}

static int any_task_active(void) {
    for (int i = 0; i < g_num_tasks; i++) {
        if (g_tasks[i]->state != TASK_TERMINATED) {
            return 1;
        }
    }
    return 0;
}

void scheduler_run(void) {
    while (any_task_active()) {
        int next = pick_next_ready();
        if (next < 0) {
            /* Every remaining task is BLOCKED; nothing to do until one
             * wakes up (sync primitives / timers flip state elsewhere). */
            continue;
        }
        g_current_idx = next;
        g_tasks[next]->state = TASK_RUNNING;
        swapcontext(&g_sched_ctx, &g_tasks[next]->context);
    }
    g_current_idx = -1;
}

void task_yield(void) {
    if (g_current_idx < 0) {
        return;
    }
    task_t *cur = g_tasks[g_current_idx];
    if (cur->state == TASK_RUNNING) {
        cur->state = TASK_READY;
    }
    swapcontext(&cur->context, &g_sched_ctx);
}

/* --- Preemption -----------------------------------------------------
 *
 * Calling swapcontext() directly from inside the SIGALRM handler works
 * because POSIX specifies that swapcontext() also saves/restores
 * uc_sigmask (via an actual sigprocmask syscall, not just an in-memory
 * copy) -- so jumping to a different task mid-handler still leaves
 * SIGALRM correctly unblocked, without ever needing a normal handler
 * return through sigreturn.
 *
 * (An earlier version of this tried mutating the kernel-supplied
 * ucontext_t from a SA_SIGINFO handler's third argument instead, to force
 * the switch through the sigreturn path. That segfaulted -- the fpregs
 * pointer inside a signal-frame ucontext_t isn't interchangeable with one
 * from a plain getcontext()/makecontext() task, so overwriting it wholesale
 * corrupts FPU state restoration. Plain swapcontext() sidesteps that.) */

static volatile sig_atomic_t g_preempt_enabled = 0;
static volatile uint64_t g_tick_count = 0;
static int g_ticks_per_slice = 1;
static int g_ticks_since_switch = 0;
static struct sigaction g_prev_sigaction;

static void preempt_handler(int sig) {
    (void)sig;
    g_tick_count++;

    if (!g_preempt_enabled || g_current_idx < 0) {
        return;
    }

    task_t *cur = g_tasks[g_current_idx];
    if (cur->state != TASK_RUNNING) {
        return;
    }

    g_ticks_since_switch++;
    if (g_ticks_since_switch < g_ticks_per_slice) {
        return;
    }
    g_ticks_since_switch = 0;

    cur->state = TASK_READY;
    int next_idx = pick_next_ready();
    if (next_idx < 0) {
        /* No other READY task -- nothing to switch to, keep running cur. */
        cur->state = TASK_RUNNING;
        return;
    }

    g_current_idx = next_idx;
    g_tasks[next_idx]->state = TASK_RUNNING;
    swapcontext(&cur->context, &g_tasks[next_idx]->context);
    /* Control resumes here whenever cur is switched back in, at which
     * point this handler invocation returns normally. */
}

void scheduler_enable_preemption(int tick_ms, int ticks_per_slice) {
    g_ticks_per_slice = (ticks_per_slice > 0) ? ticks_per_slice : 1;
    g_ticks_since_switch = 0;

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = preempt_handler;
    sa.sa_flags = SA_RESTART;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGALRM, &sa, &g_prev_sigaction);

    struct itimerval timer;
    timer.it_value.tv_sec = tick_ms / 1000;
    timer.it_value.tv_usec = (tick_ms % 1000) * 1000;
    timer.it_interval = timer.it_value;
    setitimer(ITIMER_REAL, &timer, NULL);

    g_preempt_enabled = 1;
}

void scheduler_disable_preemption(void) {
    g_preempt_enabled = 0;

    struct itimerval timer;
    memset(&timer, 0, sizeof(timer));
    setitimer(ITIMER_REAL, &timer, NULL);

    sigaction(SIGALRM, &g_prev_sigaction, NULL);
}

uint64_t scheduler_tick_count(void) {
    return g_tick_count;
}
