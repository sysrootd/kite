/**
 * @file sched.c
 * @brief RTOS scheduler implementation for STM32F4
 *
 * Implements a simple round‑robin / priority scheduler with support for
 * semaphores, mutexes, and task delays.**/

#include <stdlib.h>
#include "stm32f4xx.h"
#include "sched.h"
#include "mem.h"          /* TCB_pool() is defined here */
#include "uart.h"         /* For debug prints in fault handlers */

/*============================================================================
 *                      Private Variables
 *============================================================================*/
volatile uint32_t global_systick = 0;        /**< SysTick counter */

TCB_t *current_running_node = NULL;           /**< Currently executing task */
TCB_t *head_node = NULL;                       /**< Head of circular task list (idle task) */
TCB_t *link_node = NULL;                       /**< Last node added (used to build list) */

static uint32_t *new_task_psp = STACK_START;  /**< Stack top for a new task being allocated */
static uint32_t *next_task_psp = STACK_START; /**< Next free stack location (grows downwards) */
static uint32_t msp_start;                     /**< Saved MSP value after scheduler init */

/*============================================================================
 *                      Private Helpers
 *============================================================================*/
/**
 * @brief Trigger a PendSV exception to perform context switch.
 */
static inline void request_context_switch(void)
{
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

/**
 * @brief Enable Memory, Bus, and Usage fault handlers.
 */
void core_faults_init(void)
{
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_USGFAULTENA_Msk;
}

/*============================================================================
 *                      Scheduler Initialisation
 *============================================================================*/
/**
 * @brief Initialise scheduler: set MSP, make task list circular, init stacks.
 * @note Naked function – no prologue/epilogue.
 */
__attribute__((naked)) void scheduler_init(void)
{
    msp_start = (uint32_t)next_task_psp;          /* New MSP top */
    link_node->next_tcb_node = head_node;         /* Close circular list */

    __asm volatile(
        "PUSH {LR}                    \n"          /* Save return address */
        "BL   core_faults_init        \n"          /* Enable fault handlers */
        "POP  {LR}                    \n"          /* Restore return address */
        "LDR  R0, =msp_start          \n"          /* Load &msp_start */
        "LDR  R0, [R0]                \n"          /* Load msp_start value */
        "MSR  MSP, R0                 \n"          /* Set MSP to stack top */
        "ISB                          \n"          /* Flush pipeline */
        "PUSH {LR}                    \n"          /* Save return address */
        "BL   task_stack_init         \n"          /* Initialise all task stacks */
        "POP  {LR}                    \n"          /* Restore return address */
        "BX   LR                      \n"          /* Return */
    );
}

/**
 * @brief Start multitasking by triggering SVC exception.
 */
__attribute__((naked)) void scheduler_start(void)
{
    systick_init();                                 /* Start SysTick timer */
    __asm volatile ("svc 0");                       /* Trigger SVC to start first task */
}

/*============================================================================
 *                      System Tick Accessor
 *============================================================================*/
uint32_t get_systick_counter(void)
{
    return global_systick;
}

/*============================================================================
 *                      SysTick Configuration and Handler
 *============================================================================*/
/**
 * @brief Configure SysTick timer and set interrupt priorities.
 */
void systick_init(void)
{
    SysTick_Config(SYSTEM_CLK / TICK_HZ);            /* 1 ms tick if SYSTEM_CLK=168MHz, TICK_HZ=1000 */

    NVIC_SetPriority(PendSV_IRQn,  0xFF);            /* PendSV: lowest priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00);            /* SysTick: highest priority */
}

/**
 * @brief SysTick handler – increments tick and wakes sleeping tasks.
 */
void SysTick_Handler(void)
{
    global_systick++;
    task_wake();                                      /* Wake tasks whose delay expired */
    request_context_switch();                         /* Request a context switch */
}

/*============================================================================
 *                      SVC and PendSV Handlers
 *============================================================================*/
/**
 * @brief SVC handler – starts the first task.
 * @note Assumes current_running_node points to the first task.
 */
__attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile(
        "BL __get_psp        \n"                    /* Get first task's PSP from TCB */
        "MSR PSP, R0         \n"                    /* Set PSP to that stack */
        "LDMIA R0!, {R4-R11} \n"                    /* Restore R4-R11 from stack */
        "MSR PSP, R0         \n"                    /* Update PSP after restore */
        "LDR LR, =0xFFFFFFFD \n"                    /* EXC_RETURN for thread mode + PSP */
        "BX LR               \n"                    /* Return to task */
    );
}

/**
 * @brief PendSV handler – performs actual context switch.
 */
__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile (
        "MRS R0, PSP           \n"                  /* Get current PSP */
        "STMDB R0!, {R4-R11}   \n"                  /* Save R4-R11 on stack */
        "PUSH {LR}             \n"                  /* Save EXC_RETURN */
        "BL __set_psp          \n"                  /* Save updated PSP to TCB */
        "BL fair_priority_sched\n"                  /* Pick next task to run */
        "BL __get_psp          \n"                  /* Get next task's PSP */
        "LDMIA R0!, {R4-R11}   \n"                  /* Restore next task's R4-R11 */
        "MSR PSP, R0           \n"                  /* Set PSP to next task stack */
        "POP {LR}              \n"                  /* Restore EXC_RETURN */
        "BX LR                 \n"                  /* Return to next task */
    );
}

/*============================================================================
 *                      PSP Accessors
 *============================================================================*/
/**
 * @brief Get PSP value of current task from TCB.
 * @return PSP value (stack pointer) to be loaded.
 */
uint32_t __get_psp(void)
{
    /* current_running_node is updated by scheduler before this call */
    return (uint32_t)current_running_node->psp_value;
}

/**
 * @brief Save current PSP into current task's TCB.
 * @param current_psp_value  PSP value to store.
 */
void __set_psp(uint32_t current_psp_value)
{
    current_running_node->psp_value = (uint32_t *)current_psp_value;
}

/*============================================================================
 *                      Task Stack Initialisation
 *============================================================================*/
/**
 * @brief Initialise stack frames for all tasks.
 * @details Fills each task's stack with initial register values (xPSR, PC, LR, R0-R3, R4-R11).
 */
void task_stack_init(void)
{
    if (head_node == NULL) {
        return;                     /* No tasks? Should not happen */
    }

    TCB_t *iter = head_node;
    do {
        /* Ensure 8‑byte alignment for stack pointer (AAPCS requirement) */
        uint32_t *stack_top = iter->psp_value;
        uint32_t aligned_psp = ((uint32_t)stack_top) & ~0x7U;
        uint32_t *stack = (uint32_t *)aligned_psp;

        /* Exception stack frame (automatically pushed on exception entry) */
        *(--stack) = 0x01000000;                /* xPSR with Thumb bit set */
        *(--stack) = (uint32_t)iter->task_handler; /* PC (entry point) */
        *(--stack) = 0xFFFFFFFD;                /* LR (EXC_RETURN) */
        *(--stack) = 0x0000000C;                /* R12 */
        *(--stack) = 0x00000003;                /* R3 */
        *(--stack) = 0x00000002;                /* R2 */
        *(--stack) = 0x00000001;                /* R1 */
        *(--stack) = 0x00000000;                /* R0 */

        /* Additional registers (R4-R11) – saved by PendSV */
        for (int i = 0; i < 8; i++) {
            *(--stack) = 0;
        }

        iter->psp_value = stack;                /* Updated PSP points to last pushed word */
        iter = iter->next_tcb_node;
    } while (iter != head_node);
}

/*============================================================================
 *                      Stack and TCB Allocation
 *============================================================================*/
/**
 * @brief Allocate a stack area for a new task.
 * @param stack_words  Stack size in 32‑bit words.
 * @return Pointer to the top of the allocated stack (highest address).
 * @note Stack grows downwards; the returned pointer is the initial stack top.
 */
uint32_t *find_stack_area(uint32_t stack_words)
{
    /* Ensure even number of words for 8‑byte alignment */
    stack_words = (stack_words + 1U) & ~1U;

    new_task_psp = next_task_psp;                /* Current top for new task */
    next_task_psp -= stack_words;                /* Move next pointer downwards */
    return new_task_psp;
}

/**
 * @brief Allocate a new TCB node from the pool.
 * @return Pointer to new TCB, or NULL on failure.
 */
TCB_t *alloc_new_tcb_node(void)
{
    TCB_t *node = (TCB_t *)TCB_pool(sizeof(TCB_t));
    if (node) {
        node->next_tcb_node = NULL;
    }
    return node;
}

/*============================================================================
 *                      Task Creation
 *============================================================================*/
/**
 * @brief Initialise the idle task (must be called before any other task).
 */
void idle_task_init(void)
{
    TCB_t *idle = alloc_new_tcb_node();
    if (idle == NULL) {
        return;                     /* Allocation failed – should not happen */
    }

    uint32_t *stack = find_stack_area(IDLE_TASK_STACK_SIZE);

    idle->block_count   = 0;
    idle->current_state = TASK_WAKE;          /* Idle always ready */
    idle->psp_value     = stack;
    idle->task_handler  = idle_task;

    idle->base_priority      = 0;
    idle->effective_priority = 0;

    idle->held_mutex         = NULL;
    idle->waiting_on    = NULL;

    link_node = idle;                          /* First node in list */
    head_node = idle;                          /* Head of circular list */
}

/**
 * @brief Create a new application task.
 * @param priority      Task priority (higher number = higher priority)
 * @param handler       Pointer to task function
 * @param stack_words   Stack size in 32‑bit words
 */
void task_init(uint8_t priority, void (*handler)(void), uint32_t stack_words)
{
    /* Ensure idle task exists first */
    if (new_task_psp == next_task_psp) {        /* No tasks allocated yet */
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
    
    // Set priority fields
    tcb->base_priority      = priority;
    tcb->effective_priority = priority;
    tcb->held_mutex         = NULL;          // Initially holds no mutex
    
    tcb->task_handler  = handler;
    tcb->waiting_on    = NULL;

    // Add to circular list (unchanged)
    link_node->next_tcb_node = tcb;
    link_node = tcb;

    if (head_node->next_tcb_node == link_node) {
        current_running_node = tcb;
    }
}

/*============================================================================
 *                      Scheduling Algorithms
 *============================================================================*/
/**
 * @brief Cooperative round‑robin scheduler.
 * @details Selects the next ready task in circular order. Does not use priorities.
 */
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

    /* If no other ready task, check the current one */
    if (start->current_state == TASK_WAKE) {
        current_running_node = start;
        return;
    }

    /* Fallback to idle task */
    current_running_node = head_node;
}

/**
 * @brief Priority‑based scheduler (fair among equal priorities).
 * @details Selects the highest‑priority ready task; round‑robin among tasks with the same priority.
 *          Uses effective_priority (which may be boosted due to priority inheritance).
 */
void fair_priority_sched(void)
{
    TCB_t *start = current_running_node;
    TCB_t *candidate = current_running_node->next_tcb_node;
    TCB_t *iter;
    uint8_t highest = 0;
    uint8_t found = 0;

    /* First pass: find the highest effective priority among all ready tasks */
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
        /* No ready tasks – run idle */
        current_running_node = head_node;
        return;
    }

    /* Second pass: select the next ready task with that highest priority
     * (starting after the current task to ensure fairness) */
    do {
        if ((candidate->current_state == TASK_WAKE) &&
            (candidate->effective_priority == highest)) {
            current_running_node = candidate;
            return;
        }
        candidate = candidate->next_tcb_node;
    } while (candidate != start);

    /* If we wrapped and didn't find any (should not happen), check current task */
    if ((start->current_state == TASK_WAKE) &&
        (start->effective_priority == highest)) {
        current_running_node = start;
        return;
    }

    /* Ultimate fallback – idle task */
    current_running_node = head_node;
}

/*============================================================================
 *                      Task Control API
 *============================================================================*/
/**
 * @brief Manually yield the CPU.
 */
void task_yield(void)
{
    schedule();
}

/**
 * @brief Delay the current task for a specified number of ticks.
 * @param ticks  Number of ticks to sleep (0 means no delay).
 */
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

/**
 * @brief Delay until a specified absolute tick value (for periodic tasks).
 * @param last_wake  Pointer to variable holding the last wake time (updated by function).
 * @param period     Period in ticks.
 */
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

/**
 * @brief Wake any tasks whose sleep/block timeout has expired.
 * @note Called from SysTick_Handler.
 */
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
        schedule();                     /* Request context switch to run newly ready tasks */
    }
}

/**
 * @brief Request a context switch (trigger PendSV).
 */
void schedule(void)
{
    request_context_switch();
}

/*============================================================================
 *                      Idle Task
 *============================================================================*/
void idle_task(void)
{
    while (1) {
        __asm volatile("wfi");          /* Wait for interrupt – saves power */
    }
}

/*============================================================================
 *                      Semaphores
 *============================================================================*/
void semaphore_init(semaphore_t *sem, int32_t initial_count)
{
    sem->count = initial_count;
}

void semaphore_wait(semaphore_t *sem)
{
    ENTER_CRITICAL();

    sem->count--;

    if (sem->count < 0) {
        /* Resource not available – block current task */
        current_running_node->waiting_on = sem;
        current_running_node->current_state = TASK_BLOCKED;
        EXIT_CRITICAL();                /* Release lock before scheduling */
        schedule();                      /* This function does not return until task is re‑scheduled */
        return;                          /* When we return, we have the semaphore */
    }

    EXIT_CRITICAL();
}

void semaphore_post(semaphore_t *sem)
{
    ENTER_CRITICAL();

    sem->count++;

    if (sem->count <= 0) {
        /* At least one task waiting – unblock the first one */
        TCB_t *iter = head_node;
        do {
            if (iter->current_state == TASK_BLOCKED &&
                iter->waiting_on == sem) {
                iter->waiting_on = NULL;
                iter->current_state = TASK_WAKE;
                request_context_switch();   /* Let scheduler run the unblocked task */
                break;
            }
            iter = iter->next_tcb_node;
        } while (iter != head_node);
    }

    EXIT_CRITICAL();
}

/*============================================================================
 *                      Mutexes
 *============================================================================*/
void mutex_init(mutex_t *m)
{
    m->locked = 0;
    m->owner  = NULL;
    m->highest_waiting_prio = 0;

}

// Update a task's effective priority and reschedule if needed
static void update_task_priority(TCB_t *task, uint8_t new_prio)
{
    if (task->effective_priority == new_prio) return;
    task->effective_priority = new_prio;
    // If the task is current or ready, we might need to re‑evaluate scheduling.
    // For simplicity, we can just request a context switch.
    request_context_switch();
}

// Find the highest priority among tasks waiting on a mutex
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
        // Free – take it
        m->locked = 1;
        m->owner = current_running_node;
        current_running_node->held_mutex = m;
        // No waiters yet, so highest_waiting_prio = 0
        m->highest_waiting_prio = 0;
    } else {
        // Already locked – block current task
        current_running_node->waiting_on = m;
        current_running_node->current_state = TASK_BLOCKED;

        // Update mutex's highest waiting priority
        uint8_t my_prio = current_running_node->effective_priority;
        if (my_prio > m->highest_waiting_prio) {
            m->highest_waiting_prio = my_prio;
        }

        // Priority inheritance: boost owner if needed
        if (m->owner->effective_priority < m->highest_waiting_prio) {
            // Save original base priority (if not already boosted)
            // We assume base_priority never changes, so we can just boost effective.
            update_task_priority(m->owner, m->highest_waiting_prio);
        }

        EXIT_CRITICAL();
        schedule();                 // will not return until task is re‑scheduled
        return;
    }

    EXIT_CRITICAL();
}

void mutex_unlock(mutex_t *m)
{
    ENTER_CRITICAL();

    // Only owner can unlock
    if (m->owner == current_running_node) {
        // Remove mutex from owner's held list
        current_running_node->held_mutex = NULL;

        // Restore owner's priority to its base (if it was boosted)
        // But careful: the owner might hold other mutexes – we'll ignore that for now.
        if (current_running_node->effective_priority != current_running_node->base_priority) {
            update_task_priority(current_running_node, current_running_node->base_priority);
        }

        // Find next waiting task (first one in list order, but we could pick highest priority)
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
            // Hand mutex to next owner
            next_owner->waiting_on = NULL;
            next_owner->current_state = TASK_WAKE;
            next_owner->held_mutex = m;
            m->owner = next_owner;

            // Update highest waiting priority for mutex (by scanning remaining waiters)
            m->highest_waiting_prio = highest_waiter_priority(m);

            // Priority inheritance for new owner: boost if there are waiters with higher priority
            if (m->highest_waiting_prio > next_owner->effective_priority) {
                update_task_priority(next_owner, m->highest_waiting_prio);
            }

            request_context_switch();
        } else {
            // No waiters – release mutex
            m->locked = 0;
            m->owner = NULL;
            m->highest_waiting_prio = 0;
        }
    }

    EXIT_CRITICAL();
}

/*============================================================================
 *                      Fault Handlers
 *============================================================================*/
/**
 * @brief HardFault_Handler – assembly stub to extract stack pointer.
 */
__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "TST lr, #4       \n"               /* Check which stack was used */
        "ITE EQ           \n"               /* If‑Then‑Else */
        "MRSEQ r0, MSP    \n"               /* Use MSP if equal */
        "MRSNE r0, PSP    \n"               /* Use PSP if not equal */
        "B hardfault      \n"               /* Branch to C handler */
    );
}

/**
 * @brief C handler for HardFault – prints register dump.
 * @param stack  Pointer to the stack frame (MSP or PSP).
 */
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

/**
 * @brief Memory management fault handler.
 */
void MemManage_Handler(void)
{
    uart_printf(USART2, "MemManage Fault\n");
    uart_printf(USART2, "MMFSR %02x\n", SCB->CFSR & 0xFF);
    uart_printf(USART2, "MMFAR %08x\n", SCB->MMFAR);
    while (1);
}

/**
 * @brief Bus fault handler.
 */
void BusFault_Handler(void)
{
    uart_printf(USART2, "Bus Fault\n");
    uart_printf(USART2, "BFSR %02x\n", (SCB->CFSR >> 8) & 0xFF);
    uart_printf(USART2, "BFAR %08x\n", SCB->BFAR);
    while (1);
}

/**
 * @brief Usage fault handler.
 */
void UsageFault_Handler(void)
{
    uart_printf(USART2, "Usage Fault\n");
    uart_printf(USART2, "UFSR %04x\n", (SCB->CFSR >> 16) & 0xFFFF);
    while (1);
}