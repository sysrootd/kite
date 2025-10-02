.syntax unified
.cpu cortex-m4
.thumb

.global SysTick_Handler
.global SchedulerLaunch
.extern currentPt

SysTick_Handler:
    cpsid   i
    push    {r4-r11}
    ldr     r0, =currentPt
    ldr     r1, [r0]
    str     sp, [r1]
    ldr     r1, [r1, #4]
    str     r1, [r0]
    ldr     r1, [r0]
    ldr     sp, [r1]
    pop     {r4-r11}
    cpsie   i
    bx      lr

/* -------------------------------------------------------
 * SchedulerLaunch: Start first thread - CORRECTED VERSION
 * -----------------------------------------------------*/
SchedulerLaunch:
    cpsid   i
    ldr     r0, =currentPt
    ldr     r1, [r0]
    ldr     sp, [r1]           @ Load stack pointer
    
    @ Simple approach: manually pop everything in correct order
    pop     {r4-r11}           @ Restore R4-R11
    
    @ Manually pop the exception frame
    pop     {r0}               @ This should be R0 from your stack frame
    pop     {r1}               @ R1
    pop     {r2}               @ R2  
    pop     {r3}               @ R3
    pop     {r12}              @ R12
    pop     {lr}               @ LR (ignore)
    pop     {pc}               @ PC - this should jump to your task