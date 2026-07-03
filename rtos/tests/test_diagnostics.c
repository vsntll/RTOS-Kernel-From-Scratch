/* Phase 8 milestone: the two small additions live diagnostics needs --
 * task_stack_high_water_mark() and scheduler_switch_count() -- actually
 * reflect real usage, not just "a number that changes". */

#include <assert.h>
#include <stdio.h>

#include "../src/kernel.h"

typedef void (*test_case_fn)(void);
static int g_tests_run;

static void run_case(const char *name, test_case_fn fn) {
    printf("[case] %s\n", name);
    fn();
    scheduler_reset();
    g_tests_run++;
}

/* Deeper recursion should leave a higher (larger) high-water mark than a
 * shallow task -- not just "nonzero", an actual proportional relationship. */
static volatile int g_recurse_sink;

static void recurse(int depth) {
    volatile char pad[256]; /* force real stack growth per call, not TCO'd away */
    pad[0] = (char)depth;
    g_recurse_sink += pad[0];
    if (depth > 0) {
        recurse(depth - 1);
    }
}

static void shallow_task(void *arg) {
    (void)arg;
    recurse(2);
}

static void deep_task(void *arg) {
    (void)arg;
    recurse(40);
}

static void case_hwm_reflects_real_usage(void) {
    task_t *shallow = task_spawn("shallow", shallow_task, NULL, 0);
    scheduler_run();
    size_t shallow_hwm = task_stack_high_water_mark(shallow);
    task_destroy(shallow);
    scheduler_reset();

    task_t *deep = task_spawn("deep", deep_task, NULL, 0);
    scheduler_run();
    size_t deep_hwm = task_stack_high_water_mark(deep);
    task_destroy(deep);

    assert(shallow_hwm > 0);
    assert(deep_hwm > shallow_hwm);
}

/* A never-run task's HWM should be small (bounded by whatever
 * makecontext() itself sets up), never the full stack size. */
static void noop_task(void *arg) {
    (void)arg;
}

static void case_hwm_bounded_for_trivial_task(void) {
    task_t *t = task_spawn("noop", noop_task, NULL, 0);
    scheduler_run();
    size_t hwm = task_stack_high_water_mark(t);
    assert(hwm < t->stack_size);
    task_destroy(t);
}

static void yielder(void *arg) {
    int *iterations = arg;
    for (int i = 0; i < *iterations; i++) {
        task_yield();
    }
}

static void case_switch_count_reflects_real_switches(void) {
    uint64_t before = scheduler_switch_count();
    int iterations = 5;
    task_t *a = task_spawn("a", yielder, &iterations, 0);
    task_t *b = task_spawn("b", yielder, &iterations, 0);
    scheduler_run();
    uint64_t after = scheduler_switch_count();

    /* Each yield is one switch away and (eventually) one back -- at least
     * 2*iterations switches happened, not just "some nonzero delta". */
    assert(after - before >= (uint64_t)(2 * iterations));
    task_destroy(a);
    task_destroy(b);
}

int main(void) {
    run_case("stack high-water mark reflects real recursion depth", case_hwm_reflects_real_usage);
    run_case("a trivial task's HWM is bounded, not the whole stack",
              case_hwm_bounded_for_trivial_task);
    run_case("scheduler_switch_count() reflects real switches, not just ticks",
              case_switch_count_reflects_real_switches);

    printf("PASS: %d test cases\n", g_tests_run);
    return 0;
}
