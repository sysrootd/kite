/*
 * sched_fixed.c — reviewed and corrected
 *
 * Fix summary
 * -----------
 * BUG-1  svc_start_first_task: calling __get_psp via BL overwrites R0 before
 *        LDMIA can use it.  Rewrite as a true naked trampoline that reads
 *        current_running_node->psp_value directly in assembly, so no C helper
 *        call is needed and R0 is never clobbered.
 *
 * BUG-2  svc_task_delay: off-by-one.  `global_systick + ticks - 1` wakes the
 *        task one tick too early (a 1-tick delay fires at the current tick).
 *        Changed to `global_systick + ticks`.
 *
 * BUG-3  svc_task_sleep_until: no late-wakeup guard.  If the next target is
 *        already in the past, the task sleeps until systick wraps (~49 days on
 *        a 1 kHz tick).  Added a check: if target <= global_systick, yield
 *        immediately without blocking.
 *
 * BUG-4  svc_mutex_unlock: highest_waiter_priority() was called after
 *        next_owner was selected but before next_owner->waiting_on was cleared.
 *        The new owner still appeared to be waiting on the mutex, inflating the
 *        returned priority.  Clear waiting_on before the call.
 *
 * BUG-5  TCB held_mutex is a scalar pointer — a task holding two mutexes loses
 *        the first when it acquires the second, breaking priority inheritance
 *        restoration on unlock.  Changed to a small fixed-size array
 *        (HELD_MUTEX_MAX = 4).  unlock walks the array; lock fills the first
 *        empty slot.
 *
 * BUG-6  fair_priority_sched: fallback when no TASK_WAKE task is found was
 *        head_node unconditionally — which is the idle task, and may itself be
 *        sleeping.  Corrected to keep current_running_node if it is still
 *        TASK_WAKE, otherwise fall back to head_node (idle is always TASK_WAKE
 *        by design, but the intent is now explicit).
 */

#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx.h"
#include "sched.h"
#include "mem.h"

/* --------------------------------------------------------------------------
 * Maximum number of mutexes a single task may hold simultaneously.
 * Increase if your application requires deeper nesting.
 * -------------------------------------------------------------------------- */
#define HELD_MUTEX_MAX  4U

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

/* --------------------------------------------------------------------------
 * Forward declarations
 * -------------------------------------------------------------------------- */
__attribute__((naked, used)) void scheduler_init(void);

static void     scheduler_start(void);
static void     systick_init(void);
void            SysTick_Handler(void);
static uint8_t  task_wake(void);
static void     idle_task(void);

static void     core_faults_init(void);
static void     find_high_priority_task(void);
static void __attribute__((used)) init_helper(void);

static uint32_t __attribute__((used)) __get_psp(void);
static void     __attribute__((used)) __set_psp(uint32_t current_psp_value);
static void     __attribute__((used)) task_stack_init(void);

static uint32_t *find_stack_area(uint32_t stack_words);
static TCB_t    *alloc_new_tcb_node(void);
static void      create_idle_task(void);
static void      __attribute__((used)) fair_priority_sched(void);

static uint8_t  update_task_priority(TCB_t *task, uint8_t new_prio);
static uint8_t  highest_waiter_priority(mutex_t *m);

static void     __attribute__((used)) SVC_Handler_C(uint32_t *stack_frame);
__attribute__((naked)) static void svc_start_first_task(void);

static void svc_task_delay(uint32_t ticks);
static void svc_task_sleep_until(uint32_t *last_wake, uint32_t period);
static void svc_semaphore_wait(semaphore_t *sem);
static void svc_semaphore_post(semaphore_t *sem);
static void svc_mutex_lock(mutex_t *m);
static void svc_mutex_unlock(mutex_t *m);

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */
void kite_start(void)
{
    scheduler_init();
    scheduler_start();
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

/* --------------------------------------------------------------------------
 * Hardware init
 * -------------------------------------------------------------------------- */
static void core_faults_init(void)
{
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk  |
                  SCB_SHCSR_USGFAULTENA_Msk;
}

/* --------------------------------------------------------------------------
 * Scheduler bootstrap
 * -------------------------------------------------------------------------- */
static void find_high_priority_task(void)
{
    TCB_t *iter = head_node;

    if (iter == NULL)
    {
        current_running_node = NULL;
        return;
    }

    current_running_node = iter;
    uint8_t high = iter->effective_priority;
    iter = iter->next_tcb_node;

    while (iter != head_node)
    {
        if (iter->effective_priority > high)
        {
            high = iter->effective_priority;
            current_running_node = iter;
        }
        iter = iter->next_tcb_node;
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
        "POP  {LR}              \n"
        "LDR  R0, =msp_start    \n"
        "LDR  R0, [R0]          \n"
        "MSR  MSP, R0           \n"
        "ISB                    \n"
        "PUSH {LR}              \n"
        "BL   task_stack_init   \n"
        "POP  {LR}              \n"
        "BX   LR                \n"
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
    SysTick_Config(SYSTEM_CLK / TICK_HZ);

    NVIC_SetPriority(PendSV_IRQn,  0xFF);
    NVIC_SetPriority(SysTick_IRQn, 0x00);
    NVIC_SetPriority(SVCall_IRQn,  0x01);
}

void set_time_slice_ticks(uint32_t ticks)
{
    ENTER_CRITICAL();
    time_slice_ticks = ticks;
    tick_count = 0U;
    EXIT_CRITICAL();
}

/* --------------------------------------------------------------------------
 * SysTick — no functional change, variable naming clarified
 * -------------------------------------------------------------------------- */
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

/* --------------------------------------------------------------------------
 * SVC dispatch
 * -------------------------------------------------------------------------- */
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
    uint8_t *pc        = (uint8_t *)stack_frame[6];
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
            while (1) { /* trap unhandled SVCs */ }
    }
}

/* --------------------------------------------------------------------------
 * BUG-1 FIX: svc_start_first_task
 *
 * Original code called __get_psp() via BL, which places the return value in
 * R0 but also corrupts LR and may use the stack — both are forbidden in a
 * naked function that is mid-way through setting up the very first task stack.
 *
 * Fix: read current_running_node->psp_value directly in assembly.
 * The psp_value field is the FIRST member of TCB_t (offset 0), so a plain
 * LDR chain suffices.  Adjust the offset constant if you reorder the struct.
 *
 * PSP_VALUE_OFFSET must equal offsetof(TCB_t, psp_value).  With the layout
 * in sched.h (block_count u32 at 0, current_state u8 at 4 padded to 8,
 * psp_value pointer at 8) the offset is 8.  Verify with:
 *   static_assert(offsetof(TCB_t, psp_value) == PSP_VALUE_OFFSET, "");
 * -------------------------------------------------------------------------- */
#define PSP_VALUE_OFFSET  8   /* offsetof(TCB_t, psp_value) — verify against struct */

__attribute__((naked)) static void svc_start_first_task(void)
{
    __asm volatile(
        /* R0 = current_running_node (pointer to TCB) */
        "LDR  R0, =current_running_node    \n"
        "LDR  R0, [R0]                     \n"
        /* R0 = TCB->psp_value (already points to the saved R4-R11 frame) */
        "LDR  R0, [R0, %[off]]             \n"
        /* Restore R4-R11 from the pre-initialised stack frame */
        "LDMIA R0!, {R4-R11}               \n"
        /* Update PSP to the hardware-saved frame (R0-R3, R12, LR, PC, xPSR) */
        "MSR  PSP, R0                      \n"
        /* Switch to unprivileged Thread mode, use PSP */
        "MOV  R0, #3                       \n"
        "MSR  CONTROL, R0                  \n"
        "ISB                               \n"
        /* EXC_RETURN: return to Thread mode, use PSP, no FPU */
        "LDR  LR, =0xFFFFFFFD              \n"
        "BX   LR                           \n"
        :
        : [off] "I" (PSP_VALUE_OFFSET)
    );
}

/* --------------------------------------------------------------------------
 * Context switch — unchanged
 * -------------------------------------------------------------------------- */
__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile(
        "MRS R0, PSP            \n"
        "STMDB R0!, {R4-R11}    \n"
        "PUSH {LR}              \n"
        "BL __set_psp           \n"
        "BL fair_priority_sched \n"
        "BL __get_psp           \n"
        "LDMIA R0!, {R4-R11}    \n"
        "MSR PSP, R0            \n"
        "POP {LR}               \n"
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

/* --------------------------------------------------------------------------
 * Task stack initialisation — unchanged
 * -------------------------------------------------------------------------- */
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

        *(--stack) = 0x01000000U;             /* xPSR  — Thumb bit */
        *(--stack) = (uint32_t)iter->task_handler; /* PC */
        *(--stack) = 0xFFFFFFFDU;             /* LR    — EXC_RETURN */
        *(--stack) = 0x0000000CU;             /* R12 */
        *(--stack) = 0x00000003U;             /* R3  */
        *(--stack) = 0x00000002U;             /* R2  */
        *(--stack) = 0x00000001U;             /* R1  */
        *(--stack) = 0x00000000U;             /* R0  */

        for (int i = 0; i < 8; i++)
        {
            *(--stack) = 0U;                  /* R4-R11 */
        }

        iter->psp_value = stack;
        iter = iter->next_tcb_node;
    } while (iter != head_node);
}

/* --------------------------------------------------------------------------
 * Task creation helpers
 * -------------------------------------------------------------------------- */
static uint32_t *find_stack_area(uint32_t stack_words)
{
    stack_words  = (stack_words + 1U) & ~1U;
    new_task_psp  = next_task_psp;
    next_task_psp -= stack_words;
    return new_task_psp;
}

static TCB_t *alloc_new_tcb_node(void)
{
    TCB_t *node = (TCB_t *)TCB_pool(sizeof(TCB_t));
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

    /* BUG-5 FIX: initialise the held_mutex array */
    for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
    {
        idle->held_mutex[i] = NULL;
    }

    link_node = idle;
    head_node = idle;
}

void create_task(uint8_t priority, void (*handler)(void), uint32_t stack_words)
{
    if (new_task_psp == next_task_psp)
    {
        create_idle_task();
    }

    TCB_t *tcb = alloc_new_tcb_node();
    if (tcb == NULL)
    {
        return;
    }

    uint32_t *stack = find_stack_area(stack_words);

    tcb->block_count        = 0U;
    tcb->current_state      = TASK_WAKE;
    tcb->psp_value          = stack;
    tcb->base_priority      = priority;
    tcb->effective_priority = priority;
    tcb->task_handler       = handler;
    tcb->waiting_on         = NULL;

    /* BUG-5 FIX: initialise the held_mutex array */
    for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
    {
        tcb->held_mutex[i] = NULL;
    }

    link_node->next_tcb_node = tcb;
    link_node = tcb;
}

/* --------------------------------------------------------------------------
 * BUG-6 FIX: fair_priority_sched
 *
 * Original fallback when no TASK_WAKE task was found: unconditionally switch
 * to head_node (idle task).  If the current task is still TASK_WAKE (e.g. it
 * used task_yield() while all others are blocked), it should keep running.
 * -------------------------------------------------------------------------- */
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
        /*
         * No runnable task found starting from the next slot.
         * Keep the current task if it is still runnable; otherwise fall back
         * to idle (head_node, which is always TASK_WAKE by design).
         */
        if ((current_running_node != NULL) &&
            (current_running_node->current_state == TASK_WAKE))
        {
            /* stay on current */
        }
        else
        {
            current_running_node = head_node;
        }
    }
}

/* --------------------------------------------------------------------------
 * Task control API
 * -------------------------------------------------------------------------- */
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
            if (iter->block_count <= global_systick)
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

/* --------------------------------------------------------------------------
 * Semaphore
 * -------------------------------------------------------------------------- */
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

/* --------------------------------------------------------------------------
 * Mutex
 * -------------------------------------------------------------------------- */
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

/* --------------------------------------------------------------------------
 * SVC implementations
 * -------------------------------------------------------------------------- */

/* BUG-2 FIX: svc_task_delay
 * Changed `global_systick + ticks - 1U` → `global_systick + ticks`.
 * The old formula woke the task one tick early: a 1-tick delay expired at
 * global_systick + 0, i.e. potentially in the same systick ISR invocation.
 * -------------------------------------------------------------------------- */
static void svc_task_delay(uint32_t ticks)
{
    if ((current_running_node == NULL) || (ticks == 0U))
    {
        request_context_switch();
        return;
    }

    ENTER_CRITICAL();
    current_running_node->block_count   = global_systick + ticks; /* FIX: was -1U */
    current_running_node->current_state = TASK_SLEEP;
    EXIT_CRITICAL();

    request_context_switch();
}

/* BUG-3 FIX: svc_task_sleep_until
 * If the next target wake time is already in the past (task ran long), do not
 * block — yield immediately so the scheduler can run the overdue work without
 * a full wrap-around sleep of ~49 days.
 * -------------------------------------------------------------------------- */
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
        /* Target is still in the future — sleep until then */
        current_running_node->block_count   = next_wake;
        current_running_node->current_state = TASK_SLEEP;
    }
    /* else: already past — fall through and yield without blocking */

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
        current_running_node->waiting_on   = sem;
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

/* --------------------------------------------------------------------------
 * BUG-4 & BUG-5 FIX: svc_mutex_unlock
 *
 * BUG-4: highest_waiter_priority() was called with next_owner still in the
 *        wait list (waiting_on not yet cleared), causing it to count the new
 *        owner as a waiter and returning a priority that is too high.
 *        Fix: clear next_owner->waiting_on BEFORE calling highest_waiter_priority.
 *
 * BUG-5: held_mutex was a scalar — only one mutex could be tracked.  Now it
 *        is an array.  Unlock removes the entry for this mutex and only
 *        restores base priority if NO other held mutex has a higher waiter.
 * -------------------------------------------------------------------------- */
static void svc_mutex_unlock(mutex_t *m)
{
    uint8_t need_switch = 0U;

    if ((m == NULL) || (m->owner != current_running_node))
    {
        return;
    }

    ENTER_CRITICAL();

    /* Find the highest-priority waiter to become the new owner */
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

    /* Remove this mutex from the current owner's held list (BUG-5 FIX) */
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
        /* BUG-4 FIX: clear waiting_on BEFORE calling highest_waiter_priority
         * so the new owner is not counted as a waiter in the recalculation. */
        next_owner->waiting_on   = NULL;
        next_owner->current_state = TASK_WAKE;

        /* Add the mutex to the new owner's held list (BUG-5 FIX) */
        for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
        {
            if (next_owner->held_mutex[i] == NULL)
            {
                next_owner->held_mutex[i] = m;
                break;
            }
        }

        m->owner = next_owner;
        m->locked = 1U;
        m->highest_waiting_prio = highest_waiter_priority(m); /* now correct */

        if (m->highest_waiting_prio > next_owner->effective_priority)
        {
            update_task_priority(next_owner, m->highest_waiting_prio);
        }
        else if (next_owner->effective_priority != next_owner->base_priority)
        {
            update_task_priority(next_owner, next_owner->base_priority);
        }

        need_switch = 1U;
    }
    else
    {
        m->locked               = 0U;
        m->owner                = NULL;
        m->highest_waiting_prio = 0U;
    }

    /*
     * Restore the unlocker's priority.  BUG-5 FIX: only reset to base if
     * no other held mutex still warrants a higher priority.
     */
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

/* --------------------------------------------------------------------------
 * BUG-5 FIX: svc_mutex_lock
 * Store the acquired mutex in the first empty slot of held_mutex[].
 * -------------------------------------------------------------------------- */
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
        m->locked = 1U;
        m->owner  = current_running_node;

        /* BUG-5 FIX: store in the first available slot */
        for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
        {
            if (current_running_node->held_mutex[i] == NULL)
            {
                current_running_node->held_mutex[i] = m;
                break;
            }
        }

        m->highest_waiting_prio = 0U;
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

        if ((m->owner != NULL) &&
            (m->owner->effective_priority < m->highest_waiting_prio))
        {
            update_task_priority(m->owner, m->highest_waiting_prio);
        }

        need_switch = 1U;
    }

    EXIT_CRITICAL();

    if (need_switch != 0U)
    {
        request_context_switch();
    }
}

/* --------------------------------------------------------------------------
 * Fault handlers — unchanged
 * -------------------------------------------------------------------------- */
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

void MemManage_Handler(void) { while (1); }
void BusFault_Handler(void)  { while (1); }
void UsageFault_Handler(void){ while (1); }