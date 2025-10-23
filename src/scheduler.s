.syntax unified
.cpu cortex-m4
.thumb

.global SchedulerLaunch
.global SysTick_Handler
.global PendSV_Handler
.extern currentPt

/********************************************************************
 * SysTick_Handler
 * - Just trigger PendSV for context switch
 ********************************************************************/
SysTick_Handler:
    cpsid   i
    ldr     r0, =0xE000ED04      @ ICSR register
    ldr     r1, =0x10000000      @ Set PendSVSET bit
    str     r1, [r0]
    cpsie   i
    bx      lr


/********************************************************************
 * SchedulerLaunch
 * - Start first thread (sets PSP and starts task)
 ********************************************************************/
SchedulerLaunch:
    cpsid   i
    ldr     r0, =currentPt
    ldr     r0, [r0]             @ r0 = currentPt
    ldr     r1, [r0]             @ r1 = currentPt->stackPt
    msr     psp, r1              @ set PSP = top of first task’s stack
    movs    r0, #2
    msr     CONTROL, r0          @ switch to PSP, thread mode
    isb

    @ restore callee-saved registers (R4–R11)
    pop     {r4-r11}

    @ manually build the exception return
    ldr     lr, =0xFFFFFFFD      @ return using PSP, thread mode
    bx      lr


/********************************************************************
 * PendSV_Handler
 * - Full context switch (save/restore)
 ********************************************************************/
PendSV_Handler:
    cpsid   i

    mrs     r0, psp              @ get current process stack pointer
    ldr     r1, =currentPt
    ldr     r2, [r1]             @ r2 = currentPt
    str     r0, [r2]             @ save current PSP into TCB

    stmdb   r0!, {r4-r11}        @ save callee-saved registers

    ldr     r2, [r2, #4]         @ r2 = currentPt->nextPt
    str     r2, [r1]             @ currentPt = currentPt->nextPt

    ldr     r0, [r2]             @ r0 = next thread's PSP
    ldmia   r0!, {r4-r11}        @ restore callee-saved registers
    msr     psp, r0              @ update PSP to new thread
    cpsie   i
    bx      lr
