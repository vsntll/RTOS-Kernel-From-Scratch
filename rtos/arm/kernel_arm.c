#include "kernel_arm.h"

#define NUM_TASKS 2
#define STACK_WORDS 256 /* 1KB per task -- plenty for two print loops */

/* Reserved at the top of every task's stack: r4-r11 (8 words) followed by
 * the hardware exception frame r0,r1,r2,r3,r12,lr,pc,xpsr (8 words). */
#define FRAME_WORDS 16

static uint32_t task_stacks[NUM_TASKS][STACK_WORDS];
static uint32_t *task_sp[NUM_TASKS];
static int current_task_idx = -1;

/* Used only for the very first PendSV's "save the outgoing (nonexistent)
 * context" step -- sized to exactly one exception frame, since that's
 * all it should ever need to hold. */
#define BOOT_SCRATCH_WORDS FRAME_WORDS
static uint32_t boot_scratch_stack[BOOT_SCRATCH_WORDS];

static void task_return_trap(void) {
    /* A task's entry function loops forever in this demo, so this should
     * never actually run -- it's just a safe place to land instead of
     * returning to garbage if one ever does. */
    for (;;) {
    }
}

void task_stack_init(int idx, task_entry_t entry) {
    uint32_t *top = &task_stacks[idx][STACK_WORDS];
    uint32_t *sp = top - FRAME_WORDS;

    for (int i = 0; i < 8; i++) {
        sp[i] = 0; /* r4-r11 */
    }
    sp[8] = 0;                            /* r0 */
    sp[9] = 0;                            /* r1 */
    sp[10] = 0;                           /* r2 */
    sp[11] = 0;                           /* r3 */
    sp[12] = 0;                           /* r12 */
    sp[13] = (uint32_t)task_return_trap;  /* lr */
    sp[14] = (uint32_t)entry;             /* pc -- where execution starts */
    sp[15] = 0x01000000u;                 /* xpsr: Thumb bit set */

    task_sp[idx] = sp;
}

void *scheduler_switch_context(void *current_sp) {
    if (current_task_idx >= 0) {
        task_sp[current_task_idx] = (uint32_t *)current_sp;
    }
    current_task_idx = (current_task_idx + 1) % NUM_TASKS;
    return task_sp[current_task_idx];
}

void prime_psp_for_first_switch(void) {
    uint32_t *top = &boot_scratch_stack[BOOT_SCRATCH_WORDS];
    __asm volatile(
        "msr psp, %0    \n"
        "movs r1, #2    \n" /* CONTROL.SPSEL = 1: Thread mode uses PSP */
        "msr control, r1 \n"
        "isb            \n"
        :
        : "r"(top)
        : "r1");
}

void systick_init(uint32_t reload) {
    volatile uint32_t *syst_rvr = (volatile uint32_t *)0xE000E014u;
    volatile uint32_t *syst_cvr = (volatile uint32_t *)0xE000E018u;
    volatile uint32_t *syst_csr = (volatile uint32_t *)0xE000E010u;

    *syst_rvr = reload - 1;
    *syst_cvr = 0;
    *syst_csr = (1u << 0) | (1u << 1) | (1u << 2); /* ENABLE, TICKINT, CLKSOURCE */
}

void SysTick_Handler(void) {
    volatile uint32_t *icsr = (volatile uint32_t *)0xE000ED04u;
    *icsr = (1u << 28); /* PENDSVSET -- request a context switch */
}
