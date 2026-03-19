/* Phase 8: the same two-tasks-taking-turns demo as Phase 1/3, this time
 * booting bare-metal on a Cortex-M4 (STM32F4 Discovery, under Renode)
 * instead of running host-native on top of ucontext.h. SysTick forces a
 * PendSV-driven context switch every tick, purely preemptively -- there
 * is no voluntary yield in this demo, same as test_preemption.c's
 * hog_task, just implemented in real ARM assembly this time. */

#include "kernel_arm.h"
#include "uart.h"

/* ~111ms per switch assuming the 72MHz SysTick reference clock Renode's
 * stm32f4_discovery platform is configured with. */
#define SYSTICK_RELOAD 8000000u

static void request_context_switch(void) {
    *(volatile unsigned int *)0xE000ED04u = (1u << 28); /* PENDSVSET */
}

static void busy_wait(void) {
    for (volatile int i = 0; i < 300000; i++) {
    }
}

static void task_a(void) {
    for (;;) {
        uart_puts("[task A] tick\r\n");
        busy_wait();
    }
}

static void task_b(void) {
    for (;;) {
        uart_puts("[task B] tick\r\n");
        busy_wait();
    }
}

int main(void) {
    uart_init();
    uart_puts("RTOS Phase 8: Cortex-M port booting\r\n");

    task_stack_init(0, task_a);
    task_stack_init(1, task_b);

    systick_init(SYSTICK_RELOAD);

    /* Give PendSV somewhere valid to "save" a nonexistent outgoing
     * context, then trigger the very first switch -- from here on,
     * every switch (including this one) goes through the exact same
     * PendSV_Handler path. */
    prime_psp_for_first_switch();
    request_context_switch();

    /* If PendSV actually fired, control never reaches here again -- it
     * transfers straight to task_a. This print only shows up if the very
     * first switch silently failed to happen, which is exactly the kind
     * of failure that's otherwise invisible on real hardware. */
    uart_puts("ERROR: first context switch did not occur\r\n");

    for (;;) {
        __asm volatile("wfi");
    }
}
