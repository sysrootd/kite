#include <stdio.h>
#include "kernel.h"

#include "stm32f4xx.h"
#include "gpio.h"  /* needed for debug toggle in SysTick_Handler */

extern uint32_t SystemCoreClock;


//-----------------------------------------------------------------------------------
uint8_t current_task = 1;
uint32_t g_tick_count = 0;
TCB_t user_tasks[MAX_TASKS];
//-----------------------------------------------------------------------------------

void idle_task(void)
{
	while(1);
}


void init_systick_timer(uint32_t tick_hz)
{

    uint32_t ticks = (SystemCoreClock / tick_hz) - 1U;

    SysTick_Config(ticks);
}


__attribute__((naked)) void init_scheduler_stack(uint32_t sched_top_of_stack)
{
     __set_MSP(sched_top_of_stack);
     __asm volatile("BX LR");

}



void init_tasks_stack(void)
{

	user_tasks[0].current_state = TASK_READY_STATE;
	user_tasks[1].current_state = TASK_READY_STATE;
	user_tasks[2].current_state = TASK_READY_STATE;

	user_tasks[0].psp_value = IDLE_STACK_START;
	user_tasks[1].psp_value = T1_STACK_START;
	user_tasks[2].psp_value = T2_STACK_START;

	user_tasks[0].task_handler = idle_task;
	user_tasks[1].task_handler = task1_handler;
	user_tasks[2].task_handler = task2_handler;


	uint32_t *pPSP;

	for(int i = 0 ; i < MAX_TASKS ;i++)
	{
		pPSP = (uint32_t*) user_tasks[i].psp_value;

		pPSP--;
		*pPSP = DUMMY_XPSR;//0x01000000

		pPSP--; //PC
		*pPSP = (uint32_t) user_tasks[i].task_handler;


		pPSP--; //LR
		*pPSP = 0xFFFFFFFD;

		for(int j = 0 ; j < 13 ; j++)
		{
			pPSP--;
		    *pPSP = 0;

		}

		user_tasks[i].psp_value = (uint32_t)pPSP;


	}

}

void processor_faults_init(void)
{
	SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
	SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
	SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk;

}


uint32_t get_psp_value(void)
{

	return user_tasks[current_task].psp_value;
}


void save_psp_value(uint32_t current_psp_value)
{
	user_tasks[current_task].psp_value = current_psp_value;
}


void update_next_task(void)
{
	int state = TASK_BLOCKED_STATE;

	for(int i= 0 ; i < (MAX_TASKS) ; i++)
	{
		current_task++;
	    current_task %= MAX_TASKS;
		state = user_tasks[current_task].current_state;
		if( (state == TASK_READY_STATE) && (current_task != 0) )
			break;
	}

	if(state != TASK_READY_STATE)
		current_task = 0;
}




__attribute__((naked)) void switch_sp_to_psp(void)
{
    /* initialise PSP from the current task's saved pointer */
    __asm volatile ("PUSH {LR}");              /* save return address */
    __asm volatile ("BL get_psp_value");
    __asm volatile ("MSR PSP,R0");              /* set PSP */
    __asm volatile ("POP {LR}");               /* restore LR */

    /* switch to PSP and optionally run unprivileged (bit1 = 1).  the
       previous version wrote 0x02 which left SPSEL=0, so MSP remained
       active; tasks were still stacking on MSP and clobbered each other's
       state. */
    __asm volatile ("MOV R0,#0x03");            /* SPSEL=1, nPRIV=1 */
    __asm volatile ("MSR CONTROL,R0");
    __asm volatile ("ISB");                        /* ensure update takes effect */
    __asm volatile ("BX LR");
}


/* schedule a context switch from an unprivileged task.  writing to
   SCB->ICSR is a privileged operation and will fault if executed from
   unprivileged code, so we perform it in the SVC handler instead. */
__attribute__((naked)) void schedule(void)
{
    __asm volatile("svc #0\n"
                   "bx lr\n");
}

/* SVC handler invoked when a task calls schedule().  run privileged. */
void SVC_Handler(void)
{
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}




void task_delay(uint32_t tick_count)
{
    /* do not delay idle task; it stays ready forever */
    if (current_task == 0U || tick_count == 0U) {
        /* zero‑delay still yields once to other tasks */
        if (current_task != 0U) {
            schedule();
        }
        return;
    }

    /* update block count atomically, but keep interrupts enabled so the
       scheduler may preempt immediately after we mark the task blocked. */
    INTERRUPT_DISABLE();
    user_tasks[current_task].block_count = g_tick_count + tick_count;
    user_tasks[current_task].current_state = TASK_BLOCKED_STATE;
    INTERRUPT_ENABLE();

    /* request a context switch now that the task is blocked */
    schedule();
}


__attribute__((naked)) void PendSV_Handler(void)
{

	/*Save the context of current task */

	//1. Get current running task's PSP value
	__asm volatile("MRS R0,PSP");
	//2. Using that PSP value store SF2( R4 to R11)
	__asm volatile("STMDB R0!,{R4-R11}");

	__asm volatile("PUSH {LR}");

	//3. Save the current value of PSP
    __asm volatile("BL save_psp_value");



	/*Retrieve the context of next task */

	//1. Decide next task to run
    __asm volatile("BL update_next_task");

	//2. get its past PSP value
	__asm volatile ("BL get_psp_value");

	//3. Using that PSP value retrieve SF2(R4 to R11)
	__asm volatile ("LDMIA R0!,{R4-R11}");

	//4. update PSP and exit
	__asm volatile("MSR PSP,R0");

	__asm volatile("POP {LR}");

	__asm volatile("BX LR");



}


void update_global_tick_count(void)
{
	g_tick_count++;
}

void unblock_tasks(void)
{
    /* wake up any blocked task whose timeout has passed.  use signed
       comparison so that wraparound of g_tick_count is handled correctly. */
    for (int i = 1; i < MAX_TASKS; i++) {
        if (user_tasks[i].current_state == TASK_BLOCKED_STATE) {
            if ((int32_t)(g_tick_count - user_tasks[i].block_count) >= 0) {
                user_tasks[i].current_state = TASK_READY_STATE;
            }
        }
    }
}


void  SysTick_Handler(void)
{
    /* debug toggle to allow measurement of tick rate (PB12 must be
       initialised in main). */
    gpio_toggle(GPIOB, 12);

    update_global_tick_count();

    unblock_tasks();

    /* pend the pendsv exception for context switching */
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

//2. implement the fault handlers
void HardFault_Handler(void)
{
	// printf("Exception : Hardfault\n");
	while(1);
}


void MemManage_Handler(void)
{
	// printf("Exception : MemManage\n");
	while(1);
}

volatile uint32_t busfault_addr;
volatile uint32_t busfault_cfsr;

void BusFault_Handler(void)
{

    while (1);
}
