#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx.h"
#include "sched.h"
#include "mem.h"

volatile uint32_t global_systick = 0;

TCB_t *current_running_node = NULL;
TCB_t *head_node             = NULL;
TCB_t *link_node             = NULL;

static uint32_t *new_task_psp  = STACK_START;
static uint32_t *next_task_psp = STACK_START;
static uint32_t  msp_start;

static volatile uint32_t tick_count       = 0;
static          uint32_t time_slice_ticks = SCHED_TIME_SLICE;

static volatile uint32_t critical_nesting = 0;

__attribute__((naked, used)) void scheduler_init(void);

static void __attribute__((used)) scheduler_start(void);
static void    systick_init(void);
void           SysTick_Handler(void);
static uint8_t task_wake(void);
static void    idle_task(void);

static void core_faults_init(void);
static void find_high_priority_task(void);
static void __attribute__((used)) init_helper(void);

static uint32_t __attribute__((used)) __get_psp(void);
static void     __attribute__((used)) __set_psp(uint32_t current_psp_value);
static void     __attribute__((used)) task_stack_init(void);

static uint32_t *find_stack_area(uint32_t stack_words);
static TCB_t    *alloc_new_tcb_node(void);
static void      create_idle_task(void);
static void      __attribute__((used)) fair_priority_sched(void);

static uint8_t update_task_priority(TCB_t *task, uint8_t new_prio);
static uint8_t highest_waiter_priority(mutex_t *m);
static void    propagate_priority(mutex_t *m, uint8_t prio);

static void __attribute__((used)) SVC_Handler_C(uint32_t *stack_frame);
__attribute__((naked)) static void svc_start_first_task(void);

static void svc_task_delay(uint32_t ticks);
static void svc_task_sleep_until(uint32_t *last_wake, uint32_t period);
static void svc_semaphore_wait(semaphore_t *sem);
static void svc_semaphore_post(semaphore_t *sem);
static void svc_mutex_lock(mutex_t *m);
static void svc_mutex_unlock(mutex_t *m);

void kite_start(void)
{
    scheduler_init();
}

void sched_enter_critical(void)
{
    __set_BASEPRI(KERNEL_INTERRUPT_MASK);
    __DSB();
    __ISB();
    critical_nesting++;
}

void sched_exit_critical(void)
{
    if (critical_nesting > 0U)
    {
        critical_nesting--;

        if (critical_nesting == 0U)
        {
            __set_BASEPRI(0U);
            __DSB();
            __ISB();
        }
    }
}

static inline void request_context_switch(void)
{
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

static void core_faults_init(void)
{
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk  |
                  SCB_SHCSR_USGFAULTENA_Msk;
}

static void find_high_priority_task(void)
{
    TCB_t *iter = head_node;

    if (iter == NULL)
    {
        current_running_node = NULL;
        return;
    }

    current_running_node = NULL;
    uint8_t high = 0U;

    do
    {
        if ((iter->current_state == TASK_WAKE) &&
            ((current_running_node == NULL) || (iter->effective_priority > high)))
        {
            high = iter->effective_priority;
            current_running_node = iter;
        }
        iter = iter->next_tcb_node;
    } while (iter != head_node);

    if (current_running_node == NULL)
    {
        current_running_node = head_node;
    }
}

static void __attribute__((used)) init_helper(void)
{
    msp_start = (uint32_t)next_task_psp;
    link_node->next_tcb_node = head_node;
    find_high_priority_task();
    core_faults_init();
}

__attribute__((naked, used)) void scheduler_init(void)
{
    __asm volatile(
        "PUSH {LR}              \n"
        "BL   init_helper       \n"
        "POP  {R3, LR}          \n"
        "LDR  R0, =msp_start    \n"
        "LDR  R0, [R0]          \n"
        "MSR  MSP, R0           \n"
        "ISB                    \n"
        "BL   task_stack_init   \n"
        "B    scheduler_start   \n"
    );
}

static void scheduler_start(void)
{
    systick_init();
    __asm volatile("svc 0");
}

inline uint32_t get_systick_counter(void)
{
    return global_systick;
}

static void systick_init(void)
{
    SysTick_Config(SystemCoreClock / TICK_HZ);

    NVIC_SetPriority(PendSV_IRQn,  0xFF);
    NVIC_SetPriority(SysTick_IRQn, 0x00);
    NVIC_SetPriority(SVCall_IRQn,  0x01);
}

void set_time_slice_ticks(uint32_t ticks)
{
    if (ticks < 1U)
    {
        ticks = 1U;
    }

    ENTER_CRITICAL();
    time_slice_ticks = ticks;
    tick_count       = 0U;
    EXIT_CRITICAL();
}

void SysTick_Handler(void)
{
    global_systick++;
    tick_count++;

    uint8_t need_switch = task_wake();

    if (tick_count >= time_slice_ticks)
    {
        tick_count  = 0U;
        need_switch = 1U;
    }

    if (need_switch)
    {
        request_context_switch();
    }
}

__attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile(
        "TST LR, #4        \n"
        "ITE EQ            \n"
        "MRSEQ R0, MSP     \n"
        "MRSNE R0, PSP     \n"
        "B SVC_Handler_C   \n"
    );
}

static void __attribute__((used)) SVC_Handler_C(uint32_t *stack_frame)
{
    uint8_t *pc         = (uint8_t *)stack_frame[6];
    uint8_t  svc_number = pc[-2];

    switch (svc_number)
    {
        case SVC_START_FIRST_TASK:
            svc_start_first_task();
            break;
        case SVC_YIELD:
            request_context_switch();
            break;
        case SVC_DELAY:
            svc_task_delay(stack_frame[0]);
            break;
        case SVC_SLEEP_UNTIL:
            svc_task_sleep_until((uint32_t *)stack_frame[0], stack_frame[1]);
            break;
        case SVC_SEM_WAIT:
            svc_semaphore_wait((semaphore_t *)stack_frame[0]);
            break;
        case SVC_SEM_POST:
            svc_semaphore_post((semaphore_t *)stack_frame[0]);
            break;
        case SVC_MUTEX_LOCK:
            svc_mutex_lock((mutex_t *)stack_frame[0]);
            break;
        case SVC_MUTEX_UNLOCK:
            svc_mutex_unlock((mutex_t *)stack_frame[0]);
            break;
        default:
            while (1) {}
    }
}

__attribute__((naked)) static void svc_start_first_task(void)
{
    __asm volatile(
        "LDR  R0, =current_running_node    \n"
        "LDR  R0, [R0]                     \n"
        "LDR  R0, [R0, %[off]]             \n"
        "LDMIA R0!, {R4-R11}               \n"
        "MSR  PSP, R0                      \n"
        "MOV  R0, #3                       \n"
        "MSR  CONTROL, R0                  \n"
        "ISB                               \n"
        "LDR  LR, =0xFFFFFFFD              \n"
        "BX   LR                           \n"
        :
        : [off] "I" (PSP_VALUE_OFFSET)
    );
}

__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile(
        "MRS R0, PSP            \n"
        "STMDB R0!, {R4-R11}    \n"
        "PUSH {R3, LR}          \n"
        "BL __set_psp           \n"
        "BL fair_priority_sched \n"
        "BL __get_psp           \n"
        "POP {R3, LR}           \n"
        "LDMIA R0!, {R4-R11}    \n"
        "MSR PSP, R0            \n"
        "BX LR                  \n"
    );
}

static uint32_t __attribute__((used)) __get_psp(void)
{
    return (uint32_t)current_running_node->psp_value;
}

static void __attribute__((used)) __set_psp(uint32_t current_psp_value)
{
    current_running_node->psp_value = (uint32_t *)current_psp_value;
}

static void __attribute__((used)) task_stack_init(void)
{
    if (head_node == NULL)
    {
        return;
    }

    TCB_t *iter = head_node;
    do
    {
        uint32_t *stack_top = iter->psp_value;
        uint32_t  aligned   = ((uint32_t)stack_top) & ~0x7U;
        uint32_t *stack     = (uint32_t *)aligned;

        *(--stack) = 0x01000000U;
        *(--stack) = (uint32_t)iter->task_handler;
        *(--stack) = 0xFFFFFFFDU;
        *(--stack) = 0x0000000CU;
        *(--stack) = 0x00000003U;
        *(--stack) = 0x00000002U;
        *(--stack) = 0x00000001U;
        *(--stack) = 0x00000000U;

        for (int i = 0; i < 8; i++)
        {
            *(--stack) = 0U;
        }

        iter->psp_value = stack;
        iter = iter->next_tcb_node;
    } while (iter != head_node);
}

static uint32_t *find_stack_area(uint32_t stack_words)
{
    stack_words   = (stack_words + 1U) & ~1U;
    new_task_psp  = next_task_psp;
    next_task_psp -= stack_words;
    return new_task_psp;
}

static TCB_t *alloc_new_tcb_node(void)
{
    TCB_t *node = TCB_pool();
    if (node)
    {
        node->next_tcb_node = NULL;
    }
    return node;
}

static void create_idle_task(void)
{
    TCB_t *idle = alloc_new_tcb_node();
    if (idle == NULL)
    {
        return;
    }

    uint32_t *stack = find_stack_area(IDLE_TASK_STACK_SIZE);

    idle->block_count        = 0U;
    idle->current_state      = TASK_WAKE;
    idle->psp_value          = stack;
    idle->task_handler       = idle_task;
    idle->base_priority      = 0U;
    idle->effective_priority = 0U;
    idle->waiting_on         = NULL;

    for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
    {
        idle->held_mutex[i] = NULL;
    }

    link_node = idle;
    head_node = idle;
}

uint8_t create_task(uint8_t priority, void (*handler)(void), uint32_t stack_words)
{
    if (handler == NULL)
    {
        return 0U;
    }

    if (new_task_psp == next_task_psp)
    {
        create_idle_task();
    }

    TCB_t *tcb = alloc_new_tcb_node();
    if (tcb == NULL)
    {
        return 0U;
    }

    uint32_t *stack = find_stack_area(stack_words);

    tcb->block_count        = 0U;
    tcb->current_state      = TASK_WAKE;
    tcb->psp_value          = stack;
    tcb->base_priority      = priority;
    tcb->effective_priority = priority;
    tcb->task_handler       = handler;
    tcb->waiting_on         = NULL;

    for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
    {
        tcb->held_mutex[i] = NULL;
    }

    link_node->next_tcb_node = tcb;
    link_node = tcb;

    return 1U;
}

static void __attribute__((used)) fair_priority_sched(void)
{
    TCB_t  *iter;
    TCB_t  *start;
    TCB_t  *best    = NULL;
    uint8_t highest = 0U;

    if (head_node == NULL)
    {
        current_running_node = NULL;
        return;
    }

    if ((current_running_node != NULL) &&
        (current_running_node->next_tcb_node != NULL))
    {
        start = current_running_node->next_tcb_node;
    }
    else
    {
        start = head_node;
    }

    iter = start;
    do
    {
        if (iter->current_state == TASK_WAKE)
        {
            if ((best == NULL) || (iter->effective_priority > highest))
            {
                best    = iter;
                highest = iter->effective_priority;
            }
        }
        iter = iter->next_tcb_node;
    } while (iter != start);

    if (best != NULL)
    {
        current_running_node = best;
    }
    else
    {
        if ((current_running_node == NULL) ||
            (current_running_node->current_state != TASK_WAKE))
        {
            current_running_node = head_node;
        }
    }
}

void task_yield(void)
{
    __asm volatile("svc 1" ::: "memory");
}

void task_delay(uint32_t ticks)
{
    register uint32_t r0 __asm("r0") = ticks;
    __asm volatile("svc 2" : : "r"(r0) : "memory");
}

void task_sleep_until(uint32_t *last_wake, uint32_t period)
{
    register uint32_t *r0 __asm("r0") = last_wake;
    register uint32_t  r1 __asm("r1") = period;
    __asm volatile("svc 3" : : "r"(r0), "r"(r1) : "memory");
}

static uint8_t task_wake(void)
{
    if (head_node == NULL)
    {
        return 0U;
    }

    TCB_t  *iter      = head_node;
    uint8_t woke_task = 0U;

    do
    {
        if (iter->current_state == TASK_SLEEP)
        {
            if ((int32_t)(global_systick - iter->block_count) >= 0)
            {
                iter->current_state = TASK_WAKE;
                woke_task           = 1U;
            }
        }
        iter = iter->next_tcb_node;
    } while (iter != head_node);

    return woke_task;
}

static void idle_task(void)
{
    while (1)
    {
        __asm volatile("wfi");
    }
}

void semaphore_init(semaphore_t *sem, int32_t initial_count)
{
    sem->count = initial_count;
}

void semaphore_wait(semaphore_t *sem)
{
    register semaphore_t *r0 __asm("r0") = sem;
    __asm volatile("svc 4" : : "r"(r0) : "memory");
}

void semaphore_post(semaphore_t *sem)
{
    register semaphore_t *r0 __asm("r0") = sem;
    __asm volatile("svc 5" : : "r"(r0) : "memory");
}

void mutex_init(mutex_t *m)
{
    m->locked               = 0U;
    m->owner                = NULL;
    m->highest_waiting_prio = 0U;
}

static uint8_t update_task_priority(TCB_t *task, uint8_t new_prio)
{
    if (task->effective_priority == new_prio)
    {
        return 0U;
    }
    task->effective_priority = new_prio;
    return 1U;
}

static uint8_t highest_waiter_priority(mutex_t *m)
{
    uint8_t  max_prio = 0U;
    TCB_t   *iter     = head_node;

    if (iter == NULL)
    {
        return 0U;
    }

    do
    {
        if ((iter->current_state == TASK_BLOCKED) && (iter->waiting_on == m))
        {
            if (iter->effective_priority > max_prio)
            {
                max_prio = iter->effective_priority;
            }
        }
        iter = iter->next_tcb_node;
    } while (iter != head_node);

    return max_prio;
}

static void propagate_priority(mutex_t *m, uint8_t prio)
{
    mutex_t *current_m = m;

    while (current_m != NULL)
    {
        TCB_t *owner = current_m->owner;

        if (owner == NULL)
        {
            break;
        }

        if (prio > current_m->highest_waiting_prio)
        {
            current_m->highest_waiting_prio = prio;
        }

        if (owner->effective_priority >= prio)
        {
            break;
        }

        update_task_priority(owner, prio);

        if (owner->current_state != TASK_BLOCKED)
        {
            break;
        }

        current_m = (mutex_t *)owner->waiting_on;
    }
}

void mutex_lock(mutex_t *m)
{
    register mutex_t *r0 __asm("r0") = m;
    __asm volatile("svc 6" : : "r"(r0) : "memory");
}

void mutex_unlock(mutex_t *m)
{
    register mutex_t *r0 __asm("r0") = m;
    __asm volatile("svc 7" : : "r"(r0) : "memory");
}

static void svc_task_delay(uint32_t ticks)
{
    if ((current_running_node == NULL) || (ticks == 0U))
    {
        request_context_switch();
        return;
    }

    ENTER_CRITICAL();
    current_running_node->block_count   = global_systick + ticks;
    current_running_node->current_state = TASK_SLEEP;
    EXIT_CRITICAL();

    request_context_switch();
}

static void svc_task_sleep_until(uint32_t *last_wake, uint32_t period)
{
    if ((current_running_node == NULL) || (last_wake == NULL))
    {
        return;
    }

    ENTER_CRITICAL();

    uint32_t next_wake = *last_wake + period;
    *last_wake = next_wake;

    if ((int32_t)(next_wake - global_systick) > 0)
    {
        current_running_node->block_count   = next_wake;
        current_running_node->current_state = TASK_SLEEP;
    }

    EXIT_CRITICAL();

    request_context_switch();
}

static void svc_semaphore_wait(semaphore_t *sem)
{
    if (sem == NULL)
    {
        return;
    }

    ENTER_CRITICAL();

    sem->count--;

    if (sem->count < 0)
    {
        current_running_node->waiting_on    = sem;
        current_running_node->current_state = TASK_BLOCKED;
        EXIT_CRITICAL();
        request_context_switch();
        return;
    }

    EXIT_CRITICAL();
}

static void svc_semaphore_post(semaphore_t *sem)
{
    if (sem == NULL)
    {
        return;
    }

    ENTER_CRITICAL();

    sem->count++;

    if (sem->count <= 0)
    {
        TCB_t  *iter      = head_node;
        TCB_t  *best      = NULL;
        uint8_t best_prio = 0U;

        if (iter != NULL)
        {
            do
            {
                if ((iter->current_state == TASK_BLOCKED) &&
                    (iter->waiting_on == sem))
                {
                    if ((best == NULL) ||
                        (iter->effective_priority > best_prio))
                    {
                        best      = iter;
                        best_prio = iter->effective_priority;
                    }
                }
                iter = iter->next_tcb_node;
            } while (iter != head_node);
        }

        if (best != NULL)
        {
            best->waiting_on    = NULL;
            best->current_state = TASK_WAKE;
            EXIT_CRITICAL();
            request_context_switch();
            return;
        }
    }

    EXIT_CRITICAL();
}

static void svc_mutex_unlock(mutex_t *m)
{
    uint8_t need_switch = 0U;

    if ((m == NULL) || (m->owner != current_running_node))
    {
        return;
    }

    ENTER_CRITICAL();

    TCB_t  *iter       = head_node;
    TCB_t  *next_owner = NULL;
    uint8_t highest    = 0U;

    if (iter != NULL)
    {
        do
        {
            if ((iter->current_state == TASK_BLOCKED) &&
                (iter->waiting_on == m))
            {
                if ((next_owner == NULL) ||
                    (iter->effective_priority > highest))
                {
                    highest    = iter->effective_priority;
                    next_owner = iter;
                }
            }
            iter = iter->next_tcb_node;
        } while (iter != head_node);
    }

    for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
    {
        if (current_running_node->held_mutex[i] == m)
        {
            current_running_node->held_mutex[i] = NULL;
            break;
        }
    }

    if (next_owner != NULL)
    {
        next_owner->waiting_on    = NULL;
        next_owner->current_state = TASK_WAKE;

        uint8_t slot = HELD_MUTEX_MAX;
        for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
        {
            if (next_owner->held_mutex[i] == NULL)
            {
                slot = i;
                break;
            }
        }

        if (slot < HELD_MUTEX_MAX)
        {
            next_owner->held_mutex[slot] = m;
        }
        else
        {
            while (1) {}
        }

        m->owner                = next_owner;
        m->locked               = 1U;
        m->highest_waiting_prio = highest_waiter_priority(m);

        if (m->highest_waiting_prio > next_owner->effective_priority)
        {
            update_task_priority(next_owner, m->highest_waiting_prio);
        }

        need_switch = 1U;
    }
    else
    {
        m->locked               = 0U;
        m->owner                = NULL;
        m->highest_waiting_prio = 0U;
    }

    if (current_running_node->effective_priority !=
        current_running_node->base_priority)
    {
        uint8_t max_inherited = current_running_node->base_priority;

        for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
        {
            mutex_t *held = current_running_node->held_mutex[i];
            if ((held != NULL) &&
                (held->highest_waiting_prio > max_inherited))
            {
                max_inherited = held->highest_waiting_prio;
            }
        }

        need_switch |= update_task_priority(current_running_node, max_inherited);
    }

    EXIT_CRITICAL();

    if (need_switch != 0U)
    {
        request_context_switch();
    }
}

static void svc_mutex_lock(mutex_t *m)
{
    uint8_t need_switch = 0U;

    if (m == NULL)
    {
        return;
    }

    ENTER_CRITICAL();

    if (m->locked == 0U)
    {
        uint8_t slot = HELD_MUTEX_MAX;

        for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
        {
            if (current_running_node->held_mutex[i] == NULL)
            {
                slot = i;
                break;
            }
        }

        if (slot < HELD_MUTEX_MAX)
        {
            m->locked                              = 1U;
            m->owner                               = current_running_node;
            current_running_node->held_mutex[slot] = m;
            m->highest_waiting_prio                = 0U;
        }
        else
        {
            while (1) {}
        }
    }
    else
    {
        current_running_node->waiting_on    = m;
        current_running_node->current_state = TASK_BLOCKED;

        uint8_t my_prio = current_running_node->effective_priority;
        if (my_prio > m->highest_waiting_prio)
        {
            m->highest_waiting_prio = my_prio;
        }

        propagate_priority(m, my_prio);

        need_switch = 1U;
    }

    EXIT_CRITICAL();

    if (need_switch != 0U)
    {
        request_context_switch();
    }
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
    (void)stack;
    while (1);
}

void MemManage_Handler(void)  { while (1); }
void BusFault_Handler(void)   { while (1); }
void UsageFault_Handler(void) { while (1); }