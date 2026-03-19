#include <stdlib.h>
#include "stm32f4xx.h"
#include "sched.h"
#include "mem.h"
#include "uart.h"

volatile uint32_t global_systick = 0;

TCB_t *current_running_node = NULL;
TCB_t *head_node = NULL;
TCB_t *link_node = NULL;

static uint32_t *new_task_psp = STACK_START;
static uint32_t *next_task_psp = STACK_START;
static uint32_t msp_start;

static inline void request_context_switch(void)
{
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

void core_faults_init(void)
{
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_USGFAULTENA_Msk;
}

void find_highest_priority_task(void)
{
    TCB_t *iter = head_node;
   
    if(iter == NULL) {
        return;
    }
  
    unit8_t high = 0;
    unit8_t low = iter->base_priority;
   
    do{
        if(low > high) {
            high = low;
            current_running_node = iter;
            iter = iter->next_tcb_node;
        }
    }
    while(iter != head_node);
}
            
    
__attribute__((naked)) void scheduler_init(void)
{
    msp_start = (uint32_t)next_task_psp;
    link_node->next_tcb_node = head_node;

    __asm volatile(
        "PUSH {LR}                    \n"
        "BL   find_highest_priority_task \n"
        "POP  {LR}                    \n"
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

__attribute__((naked)) void scheduler_start(void)
{
    systick_init();
    __asm volatile ("svc 0");
}

uint32_t get_systick_counter(void)
{
    return global_systick;
}

void systick_init(void)
{
    SysTick_Config(SYSTEM_CLK / TICK_HZ);

    NVIC_SetPriority(PendSV_IRQn,  0xFF);
    NVIC_SetPriority(SysTick_IRQn, 0x00);
}

void SysTick_Handler(void)
{
    global_systick++;
    task_wake();
    request_context_switch();
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
        "BL fair_priority_sched\n"
        "BL __get_psp          \n"
        "LDMIA R0!, {R4-R11}   \n"
        "MSR PSP, R0           \n"
        "POP {LR}              \n"
        "BX LR                 \n"
    );
}

uint32_t __get_psp(void)
{
    return (uint32_t)current_running_node->psp_value;
}

void __set_psp(uint32_t current_psp_value)
{
    current_running_node->psp_value = (uint32_t *)current_psp_value;
}

void task_stack_init(void)
{
    if (head_node == NULL) {
        return;
    }

    TCB_t *iter = head_node;
    do {
        uint32_t *stack_top = iter->psp_value;
        uint32_t aligned_psp = ((uint32_t)stack_top) & ~0x7U;
        uint32_t *stack = (uint32_t *)aligned_psp;

        *(--stack) = 0x01000000;
        *(--stack) = (uint32_t)iter->task_handler;
        *(--stack) = 0xFFFFFFFD;
        *(--stack) = 0x0000000C;
        *(--stack) = 0x00000003;
        *(--stack) = 0x00000002;
        *(--stack) = 0x00000001;
        *(--stack) = 0x00000000;

        for (int i = 0; i < 8; i++) {
            *(--stack) = 0;
        }

        iter->psp_value = stack;
        iter = iter->next_tcb_node;
    } while (iter != head_node);
}

uint32_t *find_stack_area(uint32_t stack_words)
{
    stack_words = (stack_words + 1U) & ~1U;

    new_task_psp = next_task_psp;
    next_task_psp -= stack_words;
    return new_task_psp;
}

TCB_t *alloc_new_tcb_node(void)
{
    TCB_t *node = (TCB_t *)TCB_pool(sizeof(TCB_t));
    if (node) {
        node->next_tcb_node = NULL;
    }
    return node;
}

void idle_task_init(void)
{
    TCB_t *idle = alloc_new_tcb_node();
    if (idle == NULL) {
        return;
    }

    uint32_t *stack = find_stack_area(IDLE_TASK_STACK_SIZE);

    idle->block_count   = 0;
    idle->current_state = TASK_WAKE;
    idle->psp_value     = stack;
    idle->task_handler  = idle_task;

    idle->base_priority      = 0;
    idle->effective_priority = 0;

    idle->held_mutex         = NULL;
    idle->waiting_on    = NULL;

    link_node = idle;
    head_node = idle;
}

void task_init(uint8_t priority, void (*handler)(void), uint32_t stack_words)
{
    if (new_task_psp == next_task_psp) {
        idle_task_init();
    }

    TCB_t *tcb = alloc_new_tcb_node();
    if (tcb == NULL) {
        return;
    }

    uint32_t *stack = find_stack_area(stack_words);

    tcb->block_count   = 0;
    tcb->current_state = TASK_WAKE;
    tcb->psp_value     = stack;
    
    tcb->base_priority      = priority;
    tcb->effective_priority = priority;
    tcb->held_mutex         = NULL;
    
    tcb->task_handler  = handler;
    tcb->waiting_on    = NULL;

    link_node->next_tcb_node = tcb;
    link_node = tcb;

    if (head_node->next_tcb_node == link_node) {
        current_running_node = tcb;
    }
}

void cooperative_sched(void)
{
    TCB_t *start = current_running_node;
    TCB_t *candidate = current_running_node->next_tcb_node;

    do {
        if (candidate->current_state == TASK_WAKE) {
            current_running_node = candidate;
            return;
        }
        candidate = candidate->next_tcb_node;
    } while (candidate != start);

    if (start->current_state == TASK_WAKE) {
        current_running_node = start;
        return;
    }

    current_running_node = head_node;
}

void fair_priority_sched(void)
{
    TCB_t *start = current_running_node;
    TCB_t *candidate = current_running_node->next_tcb_node;
    TCB_t *iter;
    uint8_t highest = 0;
    uint8_t found = 0;

    iter = head_node;
    do {
        if (iter->current_state == TASK_WAKE) {
            if (!found || iter->effective_priority > highest) {
                highest = iter->effective_priority;
                found = 1;
            }
        }
        iter = iter->next_tcb_node;
    } while (iter != head_node);

    if (!found) {
        current_running_node = head_node;
        return;
    }

    do {
        if ((candidate->current_state == TASK_WAKE) &&
            (candidate->effective_priority == highest)) {
            current_running_node = candidate;
            return;
        }
        candidate = candidate->next_tcb_node;
    } while (candidate != start);

    if ((start->current_state == TASK_WAKE) &&
        (start->effective_priority == highest)) {
        current_running_node = start;
        return;
    }

    current_running_node = head_node;
}

void task_yield(void)
{
    schedule();
}

void task_delay(uint32_t ticks)
{
    ENTER_CRITICAL();

    if (current_running_node != NULL && ticks > 0) {
        current_running_node->block_count = global_systick + ticks - 1;
        current_running_node->current_state = TASK_SLEEP;
    }

    EXIT_CRITICAL();
    schedule();
}

void task_sleep_until(uint32_t *last_wake, uint32_t period)
{
    ENTER_CRITICAL();

    if (current_running_node != NULL) {
        *last_wake += period;
        current_running_node->block_count = *last_wake;
        current_running_node->current_state = TASK_SLEEP;
    }

    EXIT_CRITICAL();
    schedule();
}

void task_wake(void)
{
    if (head_node == NULL) {
        return;
    }

    TCB_t *iter = head_node;
    uint8_t woke_task = 0;

    do {
        if (iter->current_state == TASK_SLEEP) {
            if (iter->block_count <= global_systick) {
                iter->current_state = TASK_WAKE;
                woke_task = 1;
            }
        }
        iter = iter->next_tcb_node;
    } while (iter != head_node);

    if (woke_task) {
        schedule();
    }
}

void schedule(void)
{
    request_context_switch();
}

void idle_task(void)
{
    while (1) {
        __asm volatile("wfi");
    }
}

void semaphore_init(semaphore_t *sem, int32_t initial_count)
{
    sem->count = initial_count;
}

void semaphore_wait(semaphore_t *sem)
{
    ENTER_CRITICAL();

    sem->count--;

    if (sem->count < 0) {
        current_running_node->waiting_on = sem;
        current_running_node->current_state = TASK_BLOCKED;
        EXIT_CRITICAL();
        schedule();
        return;
    }

    EXIT_CRITICAL();
}

void semaphore_post(semaphore_t *sem)
{
    ENTER_CRITICAL();

    sem->count++;

    if (sem->count <= 0) {
        TCB_t *iter = head_node;
        do {
            if (iter->current_state == TASK_BLOCKED &&
                iter->waiting_on == sem) {
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
    m->highest_waiting_prio = 0;
}

static void update_task_priority(TCB_t *task, uint8_t new_prio)
{
    if (task->effective_priority == new_prio) return;
    task->effective_priority = new_prio;
    request_context_switch();
}

static uint8_t highest_waiter_priority(mutex_t *m)
{
    uint8_t max_prio = 0;
    TCB_t *iter = head_node;
    if (!iter) return 0;
    do {
        if (iter->current_state == TASK_BLOCKED && iter->waiting_on == m) {
            if (iter->effective_priority > max_prio) {
                max_prio = iter->effective_priority;
            }
        }
        iter = iter->next_tcb_node;
    } while (iter != head_node);
    return max_prio;
}

void mutex_lock(mutex_t *m)
{
    ENTER_CRITICAL();

    if (m->locked == 0) {
        m->locked = 1;
        m->owner = current_running_node;
        current_running_node->held_mutex = m;
        m->highest_waiting_prio = 0;
    } else {
        current_running_node->waiting_on = m;
        current_running_node->current_state = TASK_BLOCKED;

        uint8_t my_prio = current_running_node->effective_priority;
        if (my_prio > m->highest_waiting_prio) {
            m->highest_waiting_prio = my_prio;
        }

        if (m->owner->effective_priority < m->highest_waiting_prio) {
            update_task_priority(m->owner, m->highest_waiting_prio);
        }

        EXIT_CRITICAL();
        schedule();
        return;
    }

    EXIT_CRITICAL();
}

void mutex_unlock(mutex_t *m)
{
    ENTER_CRITICAL();

    if (m->owner == current_running_node) {
        current_running_node->held_mutex = NULL;

        if (current_running_node->effective_priority != current_running_node->base_priority) {
            update_task_priority(current_running_node, current_running_node->base_priority);
        }

        TCB_t *iter = head_node;
        TCB_t *next_owner = NULL;
        uint8_t highest_prio = 0;

        do {
            if (iter->current_state == TASK_BLOCKED && iter->waiting_on == m) {
                if (iter->effective_priority > highest_prio) {
                    highest_prio = iter->effective_priority;
                    next_owner = iter;
                }
            }
            iter = iter->next_tcb_node;
        } while (iter != head_node);

        if (next_owner) {
            next_owner->waiting_on = NULL;
            next_owner->current_state = TASK_WAKE;
            next_owner->held_mutex = m;
            m->owner = next_owner;

            m->highest_waiting_prio = highest_waiter_priority(m);

            if (m->highest_waiting_prio > next_owner->effective_priority) {
                update_task_priority(next_owner, m->highest_waiting_prio);
            }

            request_context_switch();
        } else {
            m->locked = 0;
            m->owner = NULL;
            m->highest_waiting_prio = 0;
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

    uart_printf(USART2, "\n\nHardFault\n");
    uart_printf(USART2, "R0  %08x\n", r0);
    uart_printf(USART2, "R1  %08x\n", r1);
    uart_printf(USART2, "R2  %08x\n", r2);
    uart_printf(USART2, "R3  %08x\n", r3);
    uart_printf(USART2, "R12 %08x\n", r12);
    uart_printf(USART2, "LR  %08x\n", lr);
    uart_printf(USART2, "PC  %08x\n", pc);
    uart_printf(USART2, "PSR %08x\n", psr);

    while (1);
}

void MemManage_Handler(void)
{
    uart_printf(USART2, "MemManage Fault\n");
    uart_printf(USART2, "MMFSR %02x\n", SCB->CFSR & 0xFF);
    uart_printf(USART2, "MMFAR %08x\n", SCB->MMFAR);
    while (1);
}

void BusFault_Handler(void)
{
    uart_printf(USART2, "Bus Fault\n");
    uart_printf(USART2, "BFSR %02x\n", (SCB->CFSR >> 8) & 0xFF);
    uart_printf(USART2, "BFAR %08x\n", SCB->BFAR);
    while (1);
}

void UsageFault_Handler(void)
{
    uart_printf(USART2, "Usage Fault\n");
    uart_printf(USART2, "UFSR %04x\n", (SCB->CFSR >> 16) & 0xFFFF);
    while (1);
}