    .syntax unified
    .cpu cortex-m4
    .thumb

    .section .text
    .align 2
    .global SysTick_Handler
    .global SchedulerLaunch
    .extern currentPt

/* SysTick_Handler: save context and switch */
SysTick_Handler:
    cpsid   i
    push    {r4-r11}            /* save r4-r11 */
    ldr     r0, =currentPt      /* r0 -> &currentPt */
    ldr     r1, [r0]            /* r1 = currentPt */
    str     sp, [r1]            /* save SP */
    ldr     r1, [r1, #4]        /* r1 = currentPt->next */
    str     r1, [r0]            /* currentPt = r1 */
    push    {r0, lr}
    pop     {r0, lr}
    ldr     r1, [r0]            /* r1 = currentPt */
    ldr     sp, [r1]            /* SP = currentPt->stackPt */
    pop     {r4-r11}            /* restore r4-r11 */
    cpsie   i
    bx      lr

/* SchedulerLaunch: start first thread */
SchedulerLaunch:
    ldr     r0, =currentPt
    ldr     r2, [r0]            /* r2 = currentPt */
    ldr     sp, [r2]            /* SP = currentPt->stackPt */
    pop     {r4-r11}
    pop     {r0-r3}
    pop     {r12}
    add     sp, sp, #4
    pop     {lr}
    add     sp, sp, #4
    cpsie   i
    bx      lr

    .align 2
