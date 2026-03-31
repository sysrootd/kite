#include <stddef.h>

#include "stm32f4xx.h"
#include "sched.h"
#include "mem.h"

// System tick counter, incremented every SysTick interrupt
volatile uint32_t global_systick = 0;

// The currently running task
TCB_t *current_running_node = NULL;
// Head of the circular linked list of tasks
TCB_t *head_node = NULL;
// Last node in the list (used to maintain circular link)
TCB_t *link_node = NULL;

// Stack allocation pointers: next available stack area
static uint32_t *new_task_psp = STACK_START;
static uint32_t *next_task_psp = STACK_START;
static uint32_t msp_start;   // Saved main stack pointer at scheduler init

// Nesting count for critical sections
static volatile uint32_t critical_nesting = 0;

// Enter critical section: block interrupts up to KERNEL_INTERRUPT_PRIORITY
void sched_enter_critical(void)
{
    __set_BASEPRI(KERNEL_INTERRUPT_MASK);
    __DSB();
    __ISB();
    critical_nesting++;
}

// Exit critical section: restore interrupts when nesting reaches zero
void sched_exit_critical(void)
{
    if (critical_nesting > 0)
    {
        critical_nesting--;

        if (critical_nesting == 0)
        {
            __set_BASEPRI(0);
            __DSB();
            __ISB();
        }
    }
}

// Request a context switch by setting PendSV bit
static inline void request_context_switch(void)
{
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

// Enable memory, bus, and usage faults in the System Handler Control Register
void core_faults_init(void)
{
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_USGFAULTENA_Msk;
}

// Find the highest priority ready task and set current_running_node
void find_high_priority_task(void)
{
    TCB_t *iter = head_node;

    if (iter == NULL) {
        current_running_node = NULL;
        return;
    }

    current_running_node = iter;
    uint8_t high = iter->base_priority;

    iter = iter->next_tcb_node;

    while (iter != head_node) {
        if (iter->base_priority > high) {
            high = iter->base_priority;
            current_running_node = iter;
        }

        iter = iter->next_tcb_node;
    }
}

// Called from scheduler_init: complete the circular list, find initial task, enable faults
void init_helper(void)
{
    msp_start = (uint32_t)next_task_psp;
    link_node->next_tcb_node = head_node;   // close circular list
    find_high_priority_task();
    core_faults_init();
}

// Naked function to set up main stack and call task_stack_init
__attribute__((naked)) void scheduler_init(void)
{
    __asm volatile(
        "PUSH {LR}              \n"   // save LR (return address)
        "BL   init_helper       \n"   // prepare scheduler data
        "POP  {LR}              \n"   // restore LR
        "LDR  R0, =msp_start    \n"
        "LDR  R0, [R0]          \n"
        "MSR  MSP, R0           \n"   // set main stack pointer
        "ISB                    \n"
        "PUSH {LR}              \n"
        "BL   task_stack_init   \n"   // initialise stacks for all tasks
        "POP  {LR}              \n"
        "BX   LR                \n"   // return
    );
}

// Start multitasking: enable SysTick and trigger SVC to switch to first task
void scheduler_start(void)
{
    systick_init();
    __asm volatile ("svc 0");
}

// Return the current system tick count
uint32_t get_systick_counter(void)
{
    return global_systick;
}

// Initialise SysTick timer and set interrupt priorities
void systick_init(void)
{
    SysTick_Config(SYSTEM_CLK / TICK_HZ);

    NVIC_SetPriority(PendSV_IRQn,  0xFF);   // lowest priority
    NVIC_SetPriority(SysTick_IRQn, 0x00);   // highest priority
}

// SysTick interrupt: increment tick, wake tasks, request context switch
void SysTick_Handler(void)
{
    global_systick++;
    task_wake();
    request_context_switch();
}

// SVC handler: load first task's stack and switch to it
__attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile(
        "BL __get_psp        \n"   // get PSP of current task (which is initial)
        "MSR PSP, R0         \n"   // set PSP
        "LDMIA R0!, {R4-R11} \n"   // pop R4-R11 from stack
        "MSR PSP, R0         \n"   // update PSP
        "LDR LR, =0xFFFFFFFD \n"   // return to thread mode with PSP
        "BX LR               \n"
    );
}

// PendSV handler: save current context, run scheduler, restore next context
__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile (
        "MRS R0, PSP           \n"   // get current PSP
        "STMDB R0!, {R4-R11}   \n"   // save R4-R11 onto current stack
        "PUSH {LR}             \n"
        "BL __set_psp          \n"   // store updated PSP in current TCB
        "BL fair_priority_sched\n"   // choose next task
        "BL __get_psp          \n"   // get PSP of new task
        "LDMIA R0!, {R4-R11}   \n"   // restore R4-R11 from new stack
        "MSR PSP, R0           \n"   // set PSP
        "POP {LR}              \n"
        "BX LR                 \n"
    );
}

// Helper to get PSP from current TCB
uint32_t __get_psp(void)
{
    return (uint32_t)current_running_node->psp_value;
}

// Helper to store PSP in current TCB
void __set_psp(uint32_t current_psp_value)
{
    current_running_node->psp_value = (uint32_t *)current_psp_value;
}

// Initialise the stack frames for all tasks
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

        // Simulate exception stack frame (Cortex-M)
        *(--stack) = 0x01000000;         // xPSR
        *(--stack) = (uint32_t)iter->task_handler; // PC
        *(--stack) = 0xFFFFFFFD;         // LR (return to thread mode)
        *(--stack) = 0x0000000C;         // R12
        *(--stack) = 0x00000003;         // R3
        *(--stack) = 0x00000002;         // R2
        *(--stack) = 0x00000001;         // R1
        *(--stack) = 0x00000000;         // R0

        // Save R4-R11 (zero for now)
        for (int i = 0; i < 8; i++) {
            *(--stack) = 0;
        }

        iter->psp_value = stack;
        iter = iter->next_tcb_node;
    } while (iter != head_node);
}

// Allocate a stack area from the top of SRAM, moving downward
uint32_t *find_stack_area(uint32_t stack_words)
{
    stack_words = (stack_words + 1U) & ~1U;   // round up to even number

    new_task_psp = next_task_psp;
    next_task_psp -= stack_words;
    return new_task_psp;
}

// Allocate a TCB from the static pool (defined in mem.c)
TCB_t *alloc_new_tcb_node(void)
{
    TCB_t *node = (TCB_t *)TCB_pool(sizeof(TCB_t));
    if (node) {
        node->next_tcb_node = NULL;
    }
    return node;
}

// Create the idle task with lowest priority (0)
void create_idle_task(void)
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

// Create a new user task with given priority and stack size
void create_task(uint8_t priority, void (*handler)(void), uint32_t stack_words)
{
    // If this is the first task, create idle task first (as a placeholder)
    if (new_task_psp == next_task_psp) {
        create_idle_task();
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

    // Insert at the end of the circular list
    link_node->next_tcb_node = tcb;
    link_node = tcb;
}

// Simple round-robin with priority: choose the next ready task with highest effective priority
void fair_priority_sched(void)
{
    TCB_t *start = current_running_node;
    TCB_t *candidate = current_running_node->next_tcb_node;
    TCB_t *iter;
    uint8_t highest = 0;
    uint8_t found = 0;

    // First find the highest priority among all ready tasks
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
        // No ready tasks? should never happen, but fallback to head
        current_running_node = head_node;
        return;
    }

    // Search for a ready task with that highest priority, starting after current
    do {
        if ((candidate->current_state == TASK_WAKE) &&
            (candidate->effective_priority == highest)) {
            current_running_node = candidate;
            return;
        }
        candidate = candidate->next_tcb_node;
    } while (candidate != start);

    // If none found after current, check current itself (should be found)
    if ((start->current_state == TASK_WAKE) &&
        (start->effective_priority == highest)) {
        current_running_node = start;
        return;
    }

    // Fallback (shouldn't happen)
    current_running_node = head_node;
}

// Yield the CPU – simply request a context switch
void task_yield(void)
{
    schedule();
}

// Delay the current task for a given number of ticks
void task_delay(uint32_t ticks)
{
    ENTER_CRITICAL();

    if (current_running_node != NULL && ticks > 0) {
        // Set wakeup time
        current_running_node->block_count = global_systick + ticks - 1;
        current_running_node->current_state = TASK_SLEEP;
    }

    EXIT_CRITICAL();
    schedule();
}

// Periodic sleep: calculate next wakeup time and go to sleep
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

// Called from SysTick: wake tasks whose delay has expired
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
        schedule();  // request a context switch if any task woke up
    }
}

// Request a context switch via PendSV
void schedule(void)
{
    request_context_switch();
}

// Idle task: just wait for interrupt
void idle_task(void)
{
    while (1) {
        __asm volatile("wfi");
    }
}

// Initialise a semaphore
void semaphore_init(semaphore_t *sem, int32_t initial_count)
{
    sem->count = initial_count;
}

// Wait (P) on a semaphore
void semaphore_wait(semaphore_t *sem)
{
    ENTER_CRITICAL();

    sem->count--;

    if (sem->count < 0) {
        // Block current task on this semaphore
        current_running_node->waiting_on = sem;
        current_running_node->current_state = TASK_BLOCKED;
        EXIT_CRITICAL();
        schedule();   // context switch
        return;
    }

    EXIT_CRITICAL();
}

// Post (V) on a semaphore
void semaphore_post(semaphore_t *sem)
{
    ENTER_CRITICAL();

    sem->count++;

    if (sem->count <= 0) {
        // There is at least one waiting task – wake the first one found
        TCB_t *iter = head_node;
        do {
            if (iter->current_state == TASK_BLOCKED &&
                iter->waiting_on == sem) {
                iter->waiting_on = NULL;
                iter->current_state = TASK_WAKE;
                request_context_switch();   // immediately request switch
                break;
            }
            iter = iter->next_tcb_node;
        } while (iter != head_node);
    }

    EXIT_CRITICAL();
}

// Initialise a mutex (no priority inheritance initially)
void mutex_init(mutex_t *m)
{
    m->locked = 0;
    m->owner  = NULL;
    m->highest_waiting_prio = 0;
}

// Helper: update a task's effective priority and request a context switch if changed
static void update_task_priority(TCB_t *task, uint8_t new_prio)
{
    if (task->effective_priority == new_prio) return;
    task->effective_priority = new_prio;
    request_context_switch();
}

// Helper: find the highest priority among tasks waiting on this mutex
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

// Lock a mutex (with simple priority inheritance)
void mutex_lock(mutex_t *m)
{
    ENTER_CRITICAL();

    if (m->locked == 0) {
        // Mutex is free, take it
        m->locked = 1;
        m->owner = current_running_node;
        current_running_node->held_mutex = m;
        m->highest_waiting_prio = 0;
    } else {
        // Already locked: block current task
        current_running_node->waiting_on = m;
        current_running_node->current_state = TASK_BLOCKED;

        uint8_t my_prio = current_running_node->effective_priority;
        if (my_prio > m->highest_waiting_prio) {
            m->highest_waiting_prio = my_prio;
        }

        // Priority inheritance: boost owner if needed
        if (m->owner->effective_priority < m->highest_waiting_prio) {
            update_task_priority(m->owner, m->highest_waiting_prio);
        }

        EXIT_CRITICAL();
        schedule();
        return;
    }

    EXIT_CRITICAL();
}

// Unlock a mutex (restore priority and hand over to highest waiter)
void mutex_unlock(mutex_t *m)
{
    ENTER_CRITICAL();

    if (m->owner == current_running_node) {
        current_running_node->held_mutex = NULL;

        // Restore original priority if it was boosted
        if (current_running_node->effective_priority != current_running_node->base_priority) {
            update_task_priority(current_running_node, current_running_node->base_priority);
        }

        // Find the highest priority waiter to become the new owner
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
            // Transfer ownership to the highest waiter
            next_owner->waiting_on = NULL;
            next_owner->current_state = TASK_WAKE;
            next_owner->held_mutex = m;
            m->owner = next_owner;

            // Recalculate highest waiting priority among remaining waiters
            m->highest_waiting_prio = highest_waiter_priority(m);

            // Possibly boost the new owner if there are still waiters with higher priority
            if (m->highest_waiting_prio > next_owner->effective_priority) {
                update_task_priority(next_owner, m->highest_waiting_prio);
            }

            request_context_switch();
        } else {
            // No waiters, mutex becomes free
            m->locked = 0;
            m->owner = NULL;
            m->highest_waiting_prio = 0;
        }
    }

    EXIT_CRITICAL();
}

// HardFault handler – naked, extracts stack pointer and calls C handler
__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "TST lr, #4       \n"   // check which stack was used
        "ITE EQ           \n"
        "MRSEQ r0, MSP    \n"
        "MRSNE r0, PSP    \n"
        "B hardfault      \n"
    );
}

// C handler for HardFault – can be used to print debug info
void hardfault(uint32_t *stack)
{
    // Uncomment to inspect stack content
    // uint32_t r0  = stack[0];
    // uint32_t r1  = stack[1];
    // uint32_t r2  = stack[2];
    // uint32_t r3  = stack[3];
    // uint32_t r12 = stack[4];
    // uint32_t lr  = stack[5];
    // uint32_t pc  = stack[6];
    // uint32_t psr = stack[7];

    while (1);
}

// Memory management fault handler
void MemManage_Handler(void)
{
    while (1);
}

// Bus fault handler
void BusFault_Handler(void)
{
    while (1);
}

// Usage fault handler
void UsageFault_Handler(void)
{
    while (1);
}