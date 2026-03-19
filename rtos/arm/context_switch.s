/* The real context switch: PendSV_Handler. This is the ARM-native
 * equivalent of task.c's swapcontext()-based context_switch() -- same
 * idea (save the outgoing task's registers, restore the incoming task's),
 * just done by hand against real Cortex-M exception-frame mechanics
 * instead of asking libc's ucontext.h to do it.
 *
 * On exception entry, hardware has already pushed {r0-r3, r12, lr, pc,
 * xpsr} onto whichever stack was active (PSP, for a task). All that's
 * left for us is the callee-saved half: r4-r11. scheduler_switch_context()
 * (kernel_arm.c) does the round-robin bookkeeping in C and hands back the
 * next task's saved stack pointer; forcing EXC_RETURN to 0xFFFFFFFD
 * guarantees "return to Thread mode using PSP" regardless of which mode
 * we were actually in when PendSV fired (matters for the very first
 * switch, which is triggered from Reset_Handler's MSP-based main()). */

.syntax unified
.cpu cortex-m4
.thumb

.section .text.PendSV_Handler
.thumb_func
.global PendSV_Handler
PendSV_Handler:
    mrs r0, psp
    stmdb r0!, {r4-r11}

    bl scheduler_switch_context   /* r0 (old sp) in, new sp out in r0 */

    ldmia r0!, {r4-r11}
    msr psp, r0
    isb

    ldr lr, =0xFFFFFFFD
    bx lr

.end
