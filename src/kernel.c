#include <stdlib.h>

#include "kernel.h"
#include "stm32f4xx.h"


/*----------------------------global section------------------------------*/
uint32_t global_tick;

TCB_t *current_running_node;
TCB_t *head_node;
TCB_t *link_node;

uint32_t *new_task_psp = STACK_START;
uint32_t *next_task_psp = STACK_START;
uint32_t msp_start;


/*-----------------------idle_task----------------------------------------*/
void idle_task(void)
{
    while(1)
    {
        __asm volatile ("WFI");
    }
}
/*------------------------------------------------------------------------*/

void core_faults_init(void)
{
    SCB->SHCSR |= (1U << 16);  // Enable MemManage fault
    SCB->SHCSR |= (1U << 17);  // Enable BusFault
    SCB->SHCSR |= (1U << 18);  // Enable UsageFault
}


__attribute__((naked)) void scheduler_init(void)
{
	msp_start = (uint32_t)next_task_psp;
	link_node->next_tcb_node = head_node;

    __asm volatile(
        "PUSH {LR}                    \n" // Save LR

        "BL   core_faults_init        \n" // Enable faults

        "POP  {LR}                    \n" // Restore LR

        "LDR  R0, =msp_start          \n" // Load address of msp_start
        "LDR  R0, [R0]                \n" // R0 = next_task_psp value

        "MSR  MSP, R0                 \n" // Set MSP = next_task_psp
    	"ISB                          \n" // Instruction sync barrier

        "PUSH {LR}                    \n" // Save LR
        "BL   tasks_stack_init        \n" // Initialize task stacks
        "POP  {LR}                    \n" // Restore LR

        "BX   LR                      \n" // Return
    );
}

void scheduler_start(void)
{
    systick_init();
    __asm volatile ("svc 0");
}


void systick_init(void)
{
    uint32_t count_value = (HSI_CLOCK / TICK_HZ) - 1;

    SYSTICK->LOAD = count_value;      // Load reload value

    SCB->SHP[10] = 0xFF;              // PendSV (exception 14) lowest priority
    SCB->SHP[11] = 0x00;              // SysTick (exception 15) higher priority

    SYSTICK->CTRL = (1 << 2) | (1 << 1) | (1 << 0);  // Enable SysTick
}

__attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile(
        "BL __get_psp        \n"  // R0 = current task PSP
        "MSR PSP, R0         \n"

        "LDMIA R0!, {R4-R11} \n"  // Restore R4-R11 (same as PendSV restore path)
        "MSR PSP, R0         \n"

        "LDR LR, =0xFFFFFFFD \n"  // Set EXC_RETURN value

        "BX LR               \n"  // Exception return
    );
}

__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile (
        // Save the context of current task

        "MRS R0, PSP           \n" // 1. Get current running task's PSP value
        "STMDB R0!, {R4-R11}   \n" // 2. Store R4-R11 on current task stack

        "PUSH {LR}             \n" // Preserve LR

        "BL __set_psp          \n" // 3. Save updated PSP value

        // Retrieve the context of next task

        "BL cooperative_sched  \n" // 1. Decide next task to run

        "BL __get_psp          \n" // 2. Get next task's PSP value

        "LDMIA R0!, {R4-R11}   \n" // 3. Restore R4-R11 of next task

        "MSR PSP, R0           \n" // 4. Update PSP

        "POP {LR}              \n" // Restore LR

        "BX LR                 \n" // Return from exception
    );
}

void SysTick_Handler(void)
{
	global_tick++;
	task_wake();
    SCB->ICSR |= (1UL << 28);
}



void schedule(void)
{
    SCB->ICSR |= (1UL << 28);
}

void tasks_stack_init(void)
{
    TCB_t *iter = head_node;
    uint32_t *pPSP;

    if (head_node == NULL) return;

    do {
        // Ensure 8-byte stack alignment
        if (((uint32_t)iter->psp_value & 0x7) != 0) {
            iter->psp_value = (uint32_t*)((uint32_t)iter->psp_value & ~0x7);
        }

        pPSP = iter->psp_value;

		pPSP--;
		*pPSP = 0x01000000;

		pPSP--; //PC
		*pPSP = (uint32_t)iter->task_handler;


		pPSP--; //LR
		*pPSP = 0xFFFFFFFD;

		for(int j = 0 ; j < 13 ; j++)
		{
			pPSP--;
		    *pPSP = 0;

		}

        iter->psp_value = pPSP;
        iter = iter->next_tcb_node;

    } while (iter != head_node);
}


uint32_t *find_stack_area(uint32_t stack_size_in_words)
{
    //8-byte alignment
    if (stack_size_in_words & 0x1u) {
    	stack_size_in_words++;
    }

    new_task_psp = next_task_psp;
    next_task_psp -= stack_size_in_words;

    return new_task_psp;
}


TCB_t *alloc_new_tcb_node(void)
{
    TCB_t *new_tcb_node = (TCB_t *)malloc(sizeof(TCB_t));

    if (new_tcb_node == NULL) {
        return NULL;
    }

    new_tcb_node->next_tcb_node = NULL;
    return new_tcb_node;
}

void create_idle_task(void)
{
	TCB_t *idle_tcb_node = alloc_new_tcb_node();

    if (idle_tcb_node == NULL) {
        return;
    }

    uint32_t *task_stack_start = find_stack_area(IDLE_TASK_STACK_WORDS);

    idle_tcb_node->block_count = 0;
    idle_tcb_node->current_state = TASK_WAKE;
    idle_tcb_node->psp_value = task_stack_start;
    idle_tcb_node->task_handler = idle_task;

    current_running_node = idle_tcb_node;
    link_node = idle_tcb_node;
    head_node = idle_tcb_node;


}

void create_task(uint8_t task_priority, void (*task_handle)(void), uint32_t stack_size)
{
	if(new_task_psp == next_task_psp) {
		create_idle_task();
	}

	TCB_t *tcb_node = alloc_new_tcb_node();

    if (tcb_node == NULL) {
        return;
    }

    uint32_t *task_stack_start = find_stack_area(stack_size);

    tcb_node->block_count = 0;
    tcb_node->current_state = TASK_WAKE;
    tcb_node->psp_value = task_stack_start;
    tcb_node->task_handler = task_handle;

    link_node->next_tcb_node = tcb_node;
    link_node = tcb_node;


}

uint32_t __get_psp(void)
{

	return (uint32_t)current_running_node->psp_value;

}

void __set_psp(uint32_t current_psp_value)
{

	current_running_node->psp_value = (uint32_t *)current_psp_value;

}

/*--------------------------------Scheduler Algorithm---------------------------*/
/*------------------------------Cooperative Non Prempt-----------------------------------*/

void cooperative_sched(void)
{

    TCB_t *candidate = current_running_node->next_tcb_node;
    uint32_t loop_counter = 0;
    const uint32_t MAX_LOOPS = 100;  // Prevent infinite loop

    // Find next ready task
    while (candidate->current_state != TASK_WAKE) {
        candidate = candidate->next_tcb_node;
        loop_counter++;

        if (candidate == current_running_node ||
            loop_counter >= MAX_LOOPS) {
            // No ready task found, return to idle task
            candidate = head_node;  // Point to idle task
            break;
        }
    }

    current_running_node = candidate;
}

/*------------------------------------------------------------------------------------*/

void task_delay(uint32_t tick_count)
{
	ENTER_CRITICAL();

    if (current_running_node != NULL) {
        current_running_node->block_count = global_tick + tick_count;
        current_running_node->current_state = TASK_SLEEP;
        schedule();
    }

    EXIT_CRITICAL();
}

void task_wake(void)
{
    TCB_t *iter = head_node;

    do {
        if (iter->current_state == TASK_SLEEP) {
            if (iter->block_count <= global_tick) {
                iter->current_state = TASK_WAKE;
            }
        }
        iter = iter->next_tcb_node;
    } while (iter != head_node && iter != NULL);
}


