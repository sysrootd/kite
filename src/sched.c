#include <stdlib.h>

#include "sched.h"
#include "stm32f4xx.h"
#include "mem.h"
#include "uart.h"

uint32_t global_tick;

TCB_t *current_running_node;
TCB_t *head_node;
TCB_t *link_node;

uint32_t *new_task_psp = STACK_START;
uint32_t *next_task_psp = STACK_START;
uint32_t msp_start;

static inline void request_context_switch(void)
{
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

void idle_task(void)
{
    while (1) {
        __asm volatile("WFI");
    }
}

void core_faults_init(void)
{
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
    SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk;
}

__attribute__((naked)) void scheduler_init(void)
{
    msp_start = (uint32_t)next_task_psp;
    link_node->next_tcb_node = head_node;

    __asm volatile(
        "PUSH {LR}                    \n"
        "BL   core_faults_init        \n"
        "POP  {LR}                    \n"
        "LDR  R0, =msp_start          \n"
        "LDR  R0, [R0]                \n"
        "MSR  MSP, R0                 \n"
        "ISB                          \n"
        "PUSH {LR}                    \n"
        "BL   task_stack_init         \n"
        "POP  {LR}                    \n"
        "BX   LR                      \n"
    );
}

void scheduler_start(void)
{
    systick_init();
    __asm volatile ("svc 0");
}

void systick_init(void)
{
    SysTick_Config(HSI_CLK / TICK_HZ);

    NVIC_SetPriority(PendSV_IRQn, 0xFF);
    NVIC_SetPriority(SysTick_IRQn, 0x00);
}

__attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile(
        "BL __get_psp        \n"
        "MSR PSP, R0         \n"
        "LDMIA R0!, {R4-R11} \n"
        "MSR PSP, R0         \n"
        "LDR LR, =0xFFFFFFFD \n"
        "BX LR               \n"
    );
}

__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile (
        "MRS R0, PSP           \n"
        "STMDB R0!, {R4-R11}   \n"
        "PUSH {LR}             \n"
        "BL __set_psp          \n"
        "BL cooperative_sched  \n"
        "BL __get_psp          \n"
        "LDMIA R0!, {R4-R11}   \n"
        "MSR PSP, R0           \n"
        "POP {LR}              \n"
        "BX LR                 \n"
    );
}

void SysTick_Handler(void)
{
    global_tick++;
    task_wake();
    request_context_switch();
}

void schedule(void)
{
    request_context_switch();
}

void task_stack_init(void)
{
    TCB_t *iter = head_node;
    
    if (head_node == NULL) return;
    
    do {
        //ensure 8-byte alignment
        uint32_t *stack_top = iter->psp_value;
        uint32_t aligned_psp = ((uint32_t)stack_top) & ~0x7;
        
        //initialize stack frame 
        uint32_t *stack = (uint32_t *)aligned_psp;
        
        *(--stack) = 0x01000000;  //xPSR-Thumb bit set
        *(--stack) = (uint32_t)iter->task_handler;  //PC
        *(--stack) = 0xFFFFFFFD;  //LR-EXC_RETURN for thread mode with PSP
        *(--stack) = 0x0000000C;  //R12
        *(--stack) = 0x00000003;  //R3
        *(--stack) = 0x00000002;  //R2
        *(--stack) = 0x00000001;  //R1
        *(--stack) = 0x00000000;  //R0
        
        //save R4-R11
        for (int i = 0; i < 8; i++) {
            *(--stack) = 0;
        }
        
        iter->psp_value = stack;
        iter = iter->next_tcb_node;

    } while (iter != head_node);
}

uint32_t *find_stack_area(uint32_t stack_size_in_words)
{
    stack_size_in_words = (stack_size_in_words + 1) & ~1;

    new_task_psp = next_task_psp;
    next_task_psp -= stack_size_in_words;

    return new_task_psp;
}

TCB_t *alloc_new_tcb_node(void)
{
    TCB_t *new_tcb_node = (TCB_t *)TCB_pool(sizeof(TCB_t));

    if (new_tcb_node == NULL) {
        return NULL;
    }

    new_tcb_node->next_tcb_node = NULL;
    return new_tcb_node;
}

void idle_task_init(void)
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
    idle_tcb_node->waiting_on = NULL;

    current_running_node = idle_tcb_node;
    link_node = idle_tcb_node;
    head_node = idle_tcb_node;
}

void task_init(uint8_t task_priority, void (*task_handle)(void), uint32_t stack_size_in_words)
{
    if(new_task_psp == next_task_psp) {
        idle_task_init();
    }

    TCB_t *tcb_node = alloc_new_tcb_node();

    if (tcb_node == NULL) {
        return;
    }

    uint32_t *task_stack_start = find_stack_area(stack_size_in_words);

    tcb_node->block_count = 0;
    tcb_node->current_state = TASK_WAKE;
    tcb_node->psp_value = task_stack_start;
    tcb_node->task_handler = task_handle;
    tcb_node->waiting_on = NULL;

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

void cooperative_sched(void)
{
    TCB_t *candidate = current_running_node->next_tcb_node;

    do
    {
        if(candidate->current_state == TASK_WAKE)
        {
            current_running_node = candidate;
            return;
        }

        candidate = candidate->next_tcb_node;

    } while(candidate != current_running_node);

    current_running_node = head_node;
}

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

void semaphore_init(semaphore_t *sem, int32_t initial_count)
{
    sem->count = initial_count;
}

void semaphore_wait(semaphore_t *sem)
{
    ENTER_CRITICAL();

    sem->count--;

    if (sem->count < 0)
    {
        current_running_node->waiting_on = sem;
        current_running_node->current_state = TASK_BLOCKED;
        schedule();
    }

    EXIT_CRITICAL();
}

void semaphore_post(semaphore_t *sem)
{
    ENTER_CRITICAL();

    sem->count++;

    if (sem->count <= 0)
    {
        TCB_t *iter = head_node;

        do
        {
            if (iter->current_state == TASK_BLOCKED &&
                iter->waiting_on == sem)
            {
                iter->waiting_on = NULL;
                iter->current_state = TASK_WAKE;
                request_context_switch();
                break;
            }

            iter = iter->next_tcb_node;

        } while (iter != head_node);
    }

    EXIT_CRITICAL();
}

void mutex_init(mutex_t *m)
{
    m->locked = 0;
    m->owner  = NULL;
}

void mutex_lock(mutex_t *m)
{
    ENTER_CRITICAL();

    if (m->locked == 0)
    {
        m->locked = 1;
        m->owner  = current_running_node;
    }
    else
    {
        current_running_node->waiting_on = m;
        current_running_node->current_state = TASK_BLOCKED;
        schedule();
    }

    EXIT_CRITICAL();
}

void mutex_unlock(mutex_t *m)
{
    ENTER_CRITICAL();

    if (m->owner == current_running_node)
    {
        TCB_t *iter = head_node;
        TCB_t *next_owner = NULL;

        do
        {
            if (iter->current_state == TASK_BLOCKED &&
                iter->waiting_on == m)
            {
                next_owner = iter;
                break;
            }

            iter = iter->next_tcb_node;
        } while (iter != head_node);

        if (next_owner)
        {
            next_owner->waiting_on = NULL;
            next_owner->current_state = TASK_WAKE;
            m->owner = next_owner;
            request_context_switch();
        }
        else
        {
            m->locked = 0;
            m->owner  = NULL;
        }
    }

    EXIT_CRITICAL();
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "TST lr, #4       \n"
        "ITE EQ           \n"
        "MRSEQ r0, MSP    \n"
        "MRSNE r0, PSP    \n"
        "B hardfault      \n"
    );
}

void hardfault(uint32_t *stack)
{
    uint32_t r0  = stack[0];
    uint32_t r1  = stack[1];
    uint32_t r2  = stack[2];
    uint32_t r3  = stack[3];
    uint32_t r12 = stack[4];
    uint32_t lr  = stack[5];
    uint32_t pc  = stack[6];
    uint32_t psr = stack[7];

    uart_printf(USART2,"HardFault\r\n");
    uart_printf(USART2,"R0  %x\r\n",r0);
    uart_printf(USART2,"R1  %x\r\n",r1);
    uart_printf(USART2,"R2  %x\r\n",r2);
    uart_printf(USART2,"R3  %x\r\n",r3);
    uart_printf(USART2,"R12 %x\r\n",r12);
    uart_printf(USART2,"LR  %x\r\n",lr);
    uart_printf(USART2,"PC  %x\r\n",pc);
    uart_printf(USART2,"PSR %x\r\n",psr);

    while(1);
}

void MemManage_Handler(void)
{
    uart_printf(USART2,"MemManage Fault\r\n");
    uart_printf(USART2,"MMFSR %x\r\n",SCB->CFSR & 0xFF);
    uart_printf(USART2,"MMFAR %x\r\n",SCB->MMFAR);

    while(1);
}

void BusFault_Handler(void)
{
    uart_printf(USART2,"Bus Fault\r\n");
    uart_printf(USART2,"BFSR %x\r\n",(SCB->CFSR >> 8) & 0xFF);
    uart_printf(USART2,"BFAR %x\r\n",SCB->BFAR);

    while(1);
}

void UsageFault_Handler(void)
{
    uart_printf(USART2,"Usage Fault\r\n");
    uart_printf(USART2,"UFSR %x\r\n",(SCB->CFSR >> 16) & 0xFFFF);

    while(1);
}