/* Cortex-M4 startup: vector table + Reset_Handler. Everything before
 * main() runs on MSP (the "main" stack) -- tasks don't start using PSP
 * until main() primes it and triggers the first PendSV (see kernel_arm.c). */

.syntax unified
.cpu cortex-m4
.thumb

.section .isr_vector, "a"
.word _estack           /* initial main stack pointer */
.word Reset_Handler
.word Default_Handler   /* NMI */
.word Default_Handler   /* HardFault */
.word Default_Handler   /* MemManage */
.word Default_Handler   /* BusFault */
.word Default_Handler   /* UsageFault */
.word 0                 /* reserved */
.word 0
.word 0
.word 0
.word Default_Handler   /* SVCall */
.word Default_Handler   /* DebugMonitor */
.word 0                 /* reserved */
.word PendSV_Handler
.word SysTick_Handler

.section .text.Reset_Handler
.thumb_func
Reset_Handler:
    ldr r0, =_estack
    mov sp, r0

    /* copy .data from flash to sram */
    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata
copy_data_loop:
    cmp r1, r2
    bhs copy_data_done
    ldr r3, [r0]
    str r3, [r1]
    adds r0, r0, #4
    adds r1, r1, #4
    b copy_data_loop
copy_data_done:

    /* zero .bss */
    ldr r0, =_sbss
    ldr r1, =_ebss
zero_bss_loop:
    cmp r0, r1
    bhs zero_bss_done
    movs r2, #0
    str r2, [r0]
    adds r0, r0, #4
    b zero_bss_loop
zero_bss_done:

    bl main
hang:
    b hang

.section .text.Default_Handler
.thumb_func
Default_Handler:
    b Default_Handler

.end
