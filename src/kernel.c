#include "stm32f401.h"
#include "kernel.h"


void  SchedulerLaunch(void);

#define NUM_OF_THREADS  3        
#define STACKSIZE   100      


#define BUS_FREQ 				16000000
uint32_t MILLIS_PRESCALER;

struct tcb{
  int32_t *stackPt;       
  struct tcb *nextPt;  
};

typedef struct tcb tcbType;
tcbType tcbs[NUM_OF_THREADS];
tcbType *currentPt;

int32_t TCB_STACK[NUM_OF_THREADS][STACKSIZE];

void KernelStackInit(int i){
  tcbs[i].stackPt = &TCB_STACK[i][STACKSIZE-16]; 
  TCB_STACK[i][STACKSIZE-1] = 0x01000000;  
}



uint8_t KernelAddThreads(void(*task0)(void),void(*task1)(void),void(*task2)(void))
{ 
	__disable_irq();
  tcbs[0].nextPt = &tcbs[1]; 
  tcbs[1].nextPt = &tcbs[2]; 
  tcbs[2].nextPt = &tcbs[0]; 
  KernelStackInit(0);
  TCB_STACK[0][STACKSIZE-2] = (int32_t)(task0); 
  
  KernelStackInit(1);
  TCB_STACK[1][STACKSIZE-2] = (int32_t)(task1); 
  
  KernelStackInit(2);
  TCB_STACK[2][STACKSIZE-2] = (int32_t)(task2); 
  currentPt = &tcbs[0];
  __enable_irq();
	
  return 1;              
}
void KernelInit(void)
{
     __disable_irq();
     MILLIS_PRESCALER=(BUS_FREQ/1000);

}
void KernelLaunch(uint32_t quanta)
{
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

void ThreadYield(void)
{
    SysTick->VAL = 0;
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

uint32_t period_tick;

void SemInit(int32_t *semaphore, int32_t value)
{
	*semaphore = value;
}

void SemPost(int32_t *semaphore)
{
	__disable_irq();
	*semaphore += 0x01;
	__enable_irq();
}

void SemWait(volatile int32_t *semaphore)
{
    __disable_irq();
    while (*semaphore <= 0)
    {   
        __enable_irq();
        ThreadYield();
        __disable_irq();
    }
    *semaphore -= 1;
    __enable_irq();
}



