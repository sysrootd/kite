#include "stm32f401.h"
#include "kernel.h"


volatile uint32_t debug_pc_value;
volatile uint32_t debug_stack_ok;
volatile uint32_t debug_task0_addr;
volatile uint32_t debug_task1_addr;

void SchedulerLaunch(void);

#define NUM_OF_THREADS  2        
#define STACKSIZE       100      

#define BUS_FREQ        16000000
uint32_t MILLIS_PRESCALER;

struct tcb {
    int32_t *stackPt;       
    struct tcb *nextPt;  
};
typedef struct tcb tcbType;

tcbType tcbs[NUM_OF_THREADS];
tcbType *currentPt;

int32_t TCB_STACK[NUM_OF_THREADS][STACKSIZE];

void KernelStackInit(int i, void(*task)(void)) {
    tcbs[i].stackPt = &TCB_STACK[i][STACKSIZE - 16];

    // Exception stack frame (will be auto-popped by exception return)
    TCB_STACK[i][STACKSIZE - 1]  = 0x01000000;  // xPSR (Thumb bit set)
    TCB_STACK[i][STACKSIZE - 2]  = (int32_t)task | 0x1;  // PC with Thumb bit
    TCB_STACK[i][STACKSIZE - 3]  = 0xFFFFFFFD;  // LR (EXC_RETURN value)
    TCB_STACK[i][STACKSIZE - 4]  = 0x12121212;  // R12
    TCB_STACK[i][STACKSIZE - 5]  = 0x03030303;  // R3
    TCB_STACK[i][STACKSIZE - 6]  = 0x02020202;  // R2  
    TCB_STACK[i][STACKSIZE - 7]  = 0x01010101;  // R1
    TCB_STACK[i][STACKSIZE - 8]  = 0x00000000;  // R0

    // Callee-saved registers (manually popped)
    TCB_STACK[i][STACKSIZE - 9]  = 0x11111111;  // R11
    TCB_STACK[i][STACKSIZE - 10] = 0x10101010;  // R10
    TCB_STACK[i][STACKSIZE - 11] = 0x09090909;  // R9
    TCB_STACK[i][STACKSIZE - 12] = 0x08080808;  // R8
    TCB_STACK[i][STACKSIZE - 13] = 0x07070707;  // R7
    TCB_STACK[i][STACKSIZE - 14] = 0x06060606;  // R6
    TCB_STACK[i][STACKSIZE - 15] = 0x05050505;  // R5
    TCB_STACK[i][STACKSIZE - 16] = 0x04040404;  // R4
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
    MILLIS_PRESCALER = (BUS_FREQ / 1000);
}

void KernelLaunch(uint32_t quanta) {
    SysTick->CTRL = 0;
    SysTick->VAL  = 0;
    SysTick->LOAD = (quanta * MILLIS_PRESCALER) - 1;

    // Exception priorities
    SCB->SHP[10] = 0xFF; // PendSV lowest
    SCB->SHP[11] = 0x80; // SysTick medium
    SCB->SHP[7]  = 0x00; // SVCall highest

    SysTick->CTRL = 0x07;
    SchedulerLaunch();
}

void ThreadYield(void) {
    SysTick->VAL = 0;
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

uint32_t period_tick;

void SemInit(int32_t *semaphore, int32_t value) {
    *semaphore = value;
}

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


void DebugStackFrame(void) {
    volatile uint32_t *stack = (volatile uint32_t*)&TCB_STACK[0][STACKSIZE - 16];
    
    debug_pc_value = TCB_STACK[0][STACKSIZE - 2];
    
    // Check critical values
    if((debug_pc_value & 0x1) && (debug_pc_value != 0xFFFFFFFF)) {
        debug_stack_ok = 1;
    } else {
        debug_stack_ok = 0;
    }
}