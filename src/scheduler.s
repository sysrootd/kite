    .syntax unified
    .cpu cortex-m4
    .thumb

    .global SysTick_Handler
    .global SchedulerLaunch
    .extern currentPt

/* -------------------------------------------------------
 * SysTick_Handler: Context switch on SysTick interrupt
 * -----------------------------------------------------*/
SysTick_Handler:
    cpsid   i                  @ disable interrupts
    push    {r4-r11}           @ save callee-saved regs

    ldr     r0, =currentPt     @ r0 = &currentPt
    ldr     r1, [r0]           @ r1 = currentPt
    str     sp, [r1]           @ save SP into TCB

    ldr     r1, [r1, #4]       @ r1 = currentPt->next
    str     r1, [r0]           @ currentPt = currentPt->next

    ldr     r1, [r0]           @ r1 = currentPt (new thread)
    ldr     sp, [r1]           @ SP = currentPt->stackPt

    pop     {r4-r11}           @ restore callee-saved regs
    cpsie   i                  @ enable interrupts
    bx      lr

/* -------------------------------------------------------
 * SchedulerLaunch: Start first thread
 * -----------------------------------------------------*/
SchedulerLaunch:
    ldr     r0, =currentPt
    ldr     r2, [r0]           @ r2 = currentPt
    ldr     sp, [r2]           @ SP = currentPt->stackPt

    mov     r0, sp
    msr     psp, r0            @ set PSP = current stack pointer

    pop     {r4-r11}           @ restore callee-saved regs
    pop     {r0-r3}            @ restore caller-saved regs
    pop     {r12}
    add     sp, sp, #4         @ skip LR (special EXC_RETURN value)
    pop     {lr}               @ restore LR
    add     sp, sp, #4         @ skip xPSR (already set)
    cpsie   i
    bx      lr
