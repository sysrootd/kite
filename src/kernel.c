/* kernel.c - minimal PSP-based kernel */
#include "stm32f401.h"
#include "kernel.h"
#include <stdint.h>

#define NUM_OF_THREADS  2
#define STACKSIZE       256   // words (increase for safety)

#define BUS_FREQ        16000000U
static uint32_t MILLIS_PRESCALER;

typedef struct tcb {
    int32_t *stackPt;
    struct tcb *nextPt;
} tcbType;

tcbType tcbs[NUM_OF_THREADS];
tcbType *currentPt;

static int32_t TCB_STACK[NUM_OF_THREADS][STACKSIZE];

volatile uint32_t debug_stack_ok = 0;
volatile uint32_t debug_task0_addr = 0;
volatile uint32_t debug_task1_addr = 0;

/* initialize thread stack with exception frame + r4-r11 slots */
void KernelStackInit(int i, void(*task)(void)) {
    /* stackPt points to R4 slot (lowest stored callee regs) */
    tcbs[i].stackPt = &TCB_STACK[i][STACKSIZE - 16];

    /* Fill exception frame (will be popped by EXC_RETURN) */
    TCB_STACK[i][STACKSIZE - 1]  = 0x01000000;                   // xPSR (Thumb bit)
    TCB_STACK[i][STACKSIZE - 2]  = ((int32_t)task) | 1;          // PC (thumb)
    TCB_STACK[i][STACKSIZE - 3]  = 0xFFFFFFFD;                   // LR (EXC_RETURN variant)
    TCB_STACK[i][STACKSIZE - 4]  = 0x12121212;                   // R12
    TCB_STACK[i][STACKSIZE - 5]  = 0x03030303;                   // R3
    TCB_STACK[i][STACKSIZE - 6]  = 0x02020202;                   // R2
    TCB_STACK[i][STACKSIZE - 7]  = 0x01010101;                   // R1
    TCB_STACK[i][STACKSIZE - 8]  = 0x00000000;                   // R0

    /* Callee-saved registers R11..R4 */
    TCB_STACK[i][STACKSIZE - 9]  = 0x11111111;  // R11
    TCB_STACK[i][STACKSIZE -10]  = 0x10101010;  // R10
    TCB_STACK[i][STACKSIZE -11]  = 0x09090909;  // R9
    TCB_STACK[i][STACKSIZE -12]  = 0x08080808;  // R8
    TCB_STACK[i][STACKSIZE -13]  = 0x07070707;  // R7
    TCB_STACK[i][STACKSIZE -14]  = 0x06060606;  // R6
    TCB_STACK[i][STACKSIZE -15]  = 0x05050505;  // R5
    TCB_STACK[i][STACKSIZE -16]  = 0x04040404;  // R4
}

uint8_t KernelAddThreads(void(*task0)(void), void(*task1)(void)) {
    __disable_irq();

    debug_task0_addr = (uint32_t)task0;
    debug_task1_addr = (uint32_t)task1;

    tcbs[0].nextPt = &tcbs[1];
    tcbs[1].nextPt = &tcbs[0];

    KernelStackInit(0, task0);
    KernelStackInit(1, task1);

    currentPt = &tcbs[0];

    __enable_irq();
    return 1;
}

void KernelInit(void) {
    __disable_irq();
    MILLIS_PRESCALER = (BUS_FREQ / 1000U);
    __enable_irq();
}

/* ThreadYield: request PendSV for context switch */
void ThreadYield(void) {
    /* set PendSV (write-1) */
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

/* KernelLaunch: configure SysTick and priorities then start scheduler */
void KernelLaunch(uint32_t quanta_ms) {
    /* SysTick setup (count to quanta_ms ms) */
    SysTick->CTRL = 0;
    SysTick->VAL  = 0;
    SysTick->LOAD = (quanta_ms * MILLIS_PRESCALER) - 1;

    /* NVIC priority grouping not changed; set PendSV lowest, SysTick medium */
    SCB->SHP[10] = 0xFF;  /* PendSV lowest */
    SCB->SHP[11] = 0x80;  /* SysTick mid */
    SCB->SHP[7]  = 0x00;  /* SVC highest (if used) */

    /* enable SysTick with core clock, interrupt and start */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Jump to assembly SchedulerLaunch that sets PSP and does EXC_RETURN */
    extern void SchedulerLaunch(void);
    SchedulerLaunch();

    while (1);
}

/* Simple semaphores */
void SemInit(int32_t *semaphore, int32_t value) { *semaphore = value; }

void SemPost(int32_t *semaphore) {
    __disable_irq();
    *semaphore += 1;
    __enable_irq();
}

void SemWait(volatile int32_t *semaphore) {
    __disable_irq();
    while (*semaphore <= 0) {
        __enable_irq();
        ThreadYield();
        __disable_irq();
    }
    *semaphore -= 1;
    __enable_irq();
}

/* HardFault: capture PC for debugging */
void HardFault_Handler(void) {
    volatile uint32_t *sp;
    volatile uint32_t fault_pc = 0;

    __asm volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq %0, msp\n"
        "mrsne %0, psp\n"
        : "=r"(sp)
    );

    fault_pc = sp[6];   /* stacked PC */
    (void)fault_pc;

    while (1) { __NOP(); }
}
