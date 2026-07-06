#ifndef RTOS_ARM_KERNEL_H
#define RTOS_ARM_KERNEL_H

#include <stdint.h>

typedef void (*task_entry_t)(void);

/* Builds a task's initial stack frame so that the first time it's
 * restored by PendSV_Handler, execution starts at `entry`. Mirrors what
 * task_create() does with getcontext()/makecontext() in the host-native
 * kernel, just laid out by hand to match the exact layout PendSV_Handler
 * expects (r4-r11, then the hardware exception frame). */
void task_stack_init(int idx, task_entry_t entry);

/* Called from PendSV_Handler with the outgoing task's saved stack
 * pointer (or a scratch pointer, for the very first call); returns the
 * next task's saved stack pointer round-robin. */
void *scheduler_switch_context(void *current_sp);

/* Configures SysTick to fire every `reload` core clock ticks; the
 * handler just requests a PendSV, same division of labor as the
 * SIGALRM-driven preempt_handler() in scheduler.c. */
void systick_init(uint32_t reload);

/* Primes PSP with a throwaway scratch stack and switches CONTROL.SPSEL
 * to use it, so the very first PendSV (triggered right after this) has
 * somewhere valid to "save" the (nonexistent) outgoing context before
 * scheduler_switch_context() picks task 0 for real. */
void prime_psp_for_first_switch(void);

#endif
