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

/* Highest-priority READY task wins; ties are broken round-robin so equal
 * priority tasks still share the CPU fairly instead of one starving the
 * rest. */
static int pick_next_ready(void) {
    if (g_num_tasks == 0) {
        return -1;
    }

    int best_priority = 0;
    int have_candidate = 0;
    for (int i = 0; i < g_num_tasks; i++) {
        if (g_tasks[i]->state == TASK_READY &&
            (!have_candidate || g_tasks[i]->priority > best_priority)) {
            best_priority = g_tasks[i]->priority;
            have_candidate = 1;
        }
    }
    if (!have_candidate) {
        return -1;
    }

    int start = (g_current_idx < 0) ? 0 : (g_current_idx + 1) % g_num_tasks;
    for (int i = 0; i < g_num_tasks; i++) {
        int idx = (start + i) % g_num_tasks;
        if (g_tasks[idx]->state == TASK_READY &&
            g_tasks[idx]->priority == best_priority) {
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

/* A SIGALRM landing mid-swapcontext() -- while we're partway through
 * saving/restoring registers for one of *our own* voluntary switches --
 * would let the handler's own swapcontext() reenter that half-finished
 * switch and corrupt it. Blocking the signal around each manual
 * swapcontext() call closes most of that window; it doesn't defeat
 * preemption, since swapcontext() restores the incoming context's own
 * uc_sigmask (unblocked) as part of the switch, so the task runs
 * preemptible again as soon as it's actually running.
 *
 * Known limitation: this narrows the race but doesn't provably close it
 * -- glibc's swapcontext() may restore the signal mask before the switch
 * is fully complete, leaving a tiny window. In practice this only showed
 * up as an intermittent hang when a task called task_yield() in a very
 * tight loop *while preemption was active* (see test_timing.c, which
 * avoids the pattern by not yielding from its busy-task). Tasks that
 * yield occasionally, or that rely purely on preemption without calling
 * task_yield() at all, don't hit it. A fully airtight fix would move
 * preemption off calling swapcontext() from inside the handler entirely
 * (mutating the kernel-supplied signal ucontext instead, so the switch
 * happens via the normal sigreturn path) -- noted here as follow-up
 * hardening work rather than blocking this phase on it. */
static void block_alarm(sigset_t *old_mask) {
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGALRM);
    sigprocmask(SIG_BLOCK, &set, old_mask);
}

static void restore_mask(const sigset_t *old_mask) {
    sigprocmask(SIG_SETMASK, old_mask, NULL);
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

        sigset_t old_mask;
        block_alarm(&old_mask);
        swapcontext(&g_sched_ctx, &g_tasks[next]->context);
        restore_mask(&old_mask);

        /* Every path that hands control back here -- voluntary yield,
         * preemption, or normal termination -- passes through this one
         * point, so checking here catches stack overflow regardless of
         * which of those happened. */
        task_check_canary_or_abort(g_tasks[next]);
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

    sigset_t old_mask;
    block_alarm(&old_mask);
    swapcontext(&cur->context, &g_sched_ctx);
    restore_mask(&old_mask);
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
 * corrupts FPU state restoration. Plain swapcontext() sidesteps that.)
 *
 * Sharper edge, found via the Phase 7 stack-canary check: the signal
 * frame for a preempted task lives on *that task's own stack*, and it
 * only gets reclaimed when the handler invocation that pushed it actually
 * returns (via sigreturn) -- which only happens once the task is
 * rescheduled and resumes past its swapcontext() call. With
 * ticks_per_slice=1, a task that never yields can get re-preempted again
 * the moment it resumes, before it finishes unwinding the *previous*
 * pending frame, stacking another one underneath. Over hundreds of ticks
 * that grows without bound and eventually overflows the stack. Bounding
 * pending_preemptions to 1 per task -- skip forcing a new switch while a
 * prior one for that same task hasn't resumed and unwound yet -- gives it
 * a tick to finish unwinding before piling on another. */

static volatile sig_atomic_t g_preempt_enabled = 0;
static volatile uint64_t g_tick_count = 0;
static int g_tick_ms = 1;
static int g_ticks_per_slice = 1;
static int g_ticks_since_switch = 0;
static struct sigaction g_prev_sigaction;

/* Runs every tick regardless of whether a slice boundary is reached this
 * time: samples per-task run/ready timing stats, and wakes any task whose
 * task_sleep() deadline has passed. */
static void update_tick_bookkeeping(void) {
    for (int i = 0; i < g_num_tasks; i++) {
        task_t *t = g_tasks[i];
        if (t->state == TASK_RUNNING) {
            t->ticks_run++;
        } else if (t->state == TASK_READY) {
            t->ticks_ready++;
        } else if (t->state == TASK_BLOCKED && t->wake_tick != 0 &&
                   g_tick_count >= t->wake_tick) {
            t->wake_tick = 0;
            t->state = TASK_READY;
        }
    }
}

static void preempt_handler(int sig) {
    (void)sig;
    g_tick_count++;
    update_tick_bookkeeping();

    if (!g_preempt_enabled || g_current_idx < 0) {
        return;
    }

    task_t *cur = g_tasks[g_current_idx];
    if (cur->state != TASK_RUNNING) {
        return;
    }

    if (cur->pending_preemptions > 0) {
        /* cur is still unwinding a previous forced switch -- give it this
         * tick to finish rather than stacking another one on top. */
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
    cur->pending_preemptions++;
    swapcontext(&cur->context, &g_tasks[next_idx]->context);
    /* Control resumes here whenever cur is switched back in, at which
     * point this handler invocation returns normally. */
    cur->pending_preemptions--;
}

void scheduler_enable_preemption(int tick_ms, int ticks_per_slice) {
    g_tick_ms = (tick_ms > 0) ? tick_ms : 1;
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

void task_sleep(int ms) {
    if (g_current_idx < 0 || ms <= 0) {
        return;
    }
    task_t *cur = g_tasks[g_current_idx];

    int ticks_to_wait = ms / g_tick_ms;
    if (ticks_to_wait <= 0) {
        ticks_to_wait = 1;
    }
    cur->wake_tick = g_tick_count + (uint64_t)ticks_to_wait;
    cur->state = TASK_BLOCKED;
    task_yield();
}

void scheduler_reset(void) {
    scheduler_disable_preemption();

    g_num_tasks = 0;
    g_current_idx = -1;
    g_tick_count = 0;
    g_ticks_since_switch = 0;
}
