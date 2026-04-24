/**
 * @file    sched.c
 * @brief   Kite RTOS — preemptive priority scheduler implementation.
 *
 * @details
 * Implements a fixed-priority, preemptive, round-robin scheduler for
 * ARM Cortex-M microcontrollers.  Key subsystems:
 *
 *  - **Ready queue**       — O(1) bitmap-accelerated, per-priority circular
 *                            doubly-linked lists.
 *  - **Sleep list**        — deadline-sorted singly-linked list; tasks are
 *                            woken by SysTick at their expiry tick.
 *  - **Wait queue**        — priority-ordered singly-linked list shared by
 *                            semaphores and mutexes.
 *  - **Semaphore**         — counting semaphore with priority-ordered wait
 *                            queue.
 *  - **Mutex**             — binary mutex with full Priority Inheritance (PI)
 *                            to eliminate unbounded priority inversion.
 *  - **MPU stack guard**   — one no-access MPU region per task catches stack
 *                            overflows before they corrupt adjacent memory.
 *  - **Critical section**  — BASEPRI masking (nestable); never disables
 *                            NMI or HardFault.
 *
 * Context switching is performed entirely in PendSV (lowest priority ISR).
 * All blocking kernel calls are routed through SVC to guarantee that the
 * call is made from Thread mode with a valid PSP, even when invoked from
 * within an ISR-like context.
 *
 * @note    Requires CMSIS headers supplied by the MCU vendor (mcu.h) and a
 *          project-level configuration header (config.h / sched.h).
 *          The heap allocator entry-point @c TCB_pool() is provided by
 *          mem.h / mem.c.
 *
 * @version 1.0.0
 * @date    2025
 */

#include <stddef.h>
#include <stdint.h>

#include "mcu.h"
#include "sched.h"
#include "mem.h"

/* =========================================================================
 * Global & module-level state
 * ========================================================================= */

/** @brief Monotonically increasing system tick counter (ms resolution). */
volatile uint32_t global_systick = 0;

/** @brief Pointer to the TCB of the currently executing task. */
TCB_t *current_running_node  = NULL;

/** @brief Head of the intrusive singly-linked list of all TCBs. */
TCB_t *head_node             = NULL;

/** @brief Tail pointer used during task registration to append new TCBs. */
TCB_t *link_node             = NULL;

/** @brief Next free stack allocation pointer (grows downward from SRAM top). */
static uint32_t *new_task_psp  = STACK_START;

/** @brief Watermark for the next task's stack top. */
static uint32_t *next_task_psp = STACK_START;

/** @brief MSP value captured before the kernel switches to PSP-based tasks. */
static uint32_t  msp_start;

/** @brief Sub-tick counter used for round-robin time-slicing. */
static volatile uint32_t tick_count       = 0;

/** @brief Configurable time-slice length in system ticks. */
static          uint32_t time_slice_ticks = SCHED_TIME_SLICE;

/** @brief Per-priority ready queues (circular doubly-linked lists). */
static TCB_t   *ready_queue[PRIO_LEVELS];

/** @brief Bitmap of occupied priority levels; bit N set ⟹ ready_queue[N] non-empty. */
static uint32_t ready_bitmap = 0U;

/** @brief Head of the deadline-sorted sleep list. */
static TCB_t *sleep_list_head = NULL;

/**
 * @brief Per-task held-mutex table.
 *
 * @c held_mutex_table[i][j] holds a pointer to the j-th mutex currently
 * owned by task i.  The corresponding bit in @c TCB_t::held_mutex_bitmap
 * indicates whether slot j is occupied.
 */
static mutex_t *held_mutex_table[MAX_TASKS][HELD_MUTEX_MAX];

/** @brief Total number of user tasks registered (excludes idle). */
static uint8_t  task_count = 0U;

/** @brief Forward declaration used by the held-mutex helpers. */
static uint8_t  tcb_index_of(TCB_t *t);

/**
 * @brief Entry in the deferred task-initialisation table.
 *
 * Stack frames are pushed during @c task_stack_init(), not at
 * @c create_task() time, because the MSP is still in use by C startup
 * code at registration time.
 */
typedef struct {
    TCB_t        *tcb;       /**< Pointer to the owning TCB.     */
    void        (*handler)(void); /**< Task entry-point function. */
} task_init_entry_t;

/** @brief Table of all registered tasks awaiting stack initialisation. */
static task_init_entry_t task_init_table[MAX_TASKS];

/** @brief Number of entries currently stored in @c task_init_table. */
static uint8_t           task_init_count = 0U;

/* =========================================================================
 * Forward declarations — Critical Section
 * ========================================================================= */
static inline void request_context_switch(void);

/* =========================================================================
 * Forward declarations — Scheduler Core & Startup
 * ========================================================================= */
__attribute__((naked, used)) void scheduler_init(void);
static void __attribute__((used)) scheduler_start(void);
static void    systick_init(void);
void           SysTick_Handler(void);
static uint8_t task_wake(void);
static void    find_high_priority_task(void);
static void __attribute__((used)) init_helper(void);

/* =========================================================================
 * Forward declarations — MPU & Memory Protection
 * ========================================================================= */
static void core_faults_init(void);
static void mpu_init(void);
static void mpu_update_stack_guard(TCB_t *t);

/* =========================================================================
 * Forward declarations — Task Creation & Stack
 * ========================================================================= */
static uint32_t *find_stack_area(uint32_t stack_words, uint32_t **guard_base_out);
static TCB_t    *alloc_new_tcb_node(void);
static void      create_idle_task(void);
static void __attribute__((used)) task_stack_init(void);
static void __attribute__((used, noreturn)) kernel_panic(void);
static void idle_task(void);

/* =========================================================================
 * Forward declarations — Context Switch
 * ========================================================================= */
static uint32_t __attribute__((used)) __get_psp(void);
static void     __attribute__((used)) __set_psp(uint32_t current_psp_value);
static void __attribute__((used)) SVC_Handler_C(uint32_t *stack_frame);
__attribute__((naked)) static void svc_start_first_task(void);

/* =========================================================================
 * Forward declarations — Mutex & Priority Inheritance
 * ========================================================================= */
static uint8_t update_task_priority(TCB_t *task, uint8_t new_prio);
static uint8_t highest_waiter_priority(mutex_t *m);
static void    propagate_priority(mutex_t *m, uint8_t prio);

/* =========================================================================
 * Forward declarations — SVC Dispatch Targets
 * ========================================================================= */
static void svc_task_delay(uint32_t ticks);
static void svc_task_sleep_until(uint32_t *last_wake, uint32_t period);
static void svc_semaphore_wait(semaphore_t *sem);
static void svc_semaphore_post(semaphore_t *sem);
static void svc_mutex_lock(mutex_t *m);
static void svc_mutex_unlock(mutex_t *m);

/* =========================================================================
 * SECTION 1 — CRITICAL SECTION
 *
 * Nestable interrupt masking via BASEPRI.  All kernel data structures must
 * be accessed inside a critical section to prevent races with SysTick and
 * PendSV.  NMI and HardFault are never masked.
 * ========================================================================= */

/** @brief Nesting depth of the critical section; 0 means interrupts are unmasked. */
static volatile uint32_t critical_nesting = 0;

/** @brief BASEPRI value saved on the first @c sched_enter_critical() call. */
static          uint32_t saved_basepri    = 0;

/**
 * @brief Enter a nestable critical section by raising BASEPRI.
 *
 * @details
 * Reads the current BASEPRI value, then immediately sets it to
 * @c KERNEL_INTERRUPT_MASK so that all interrupts at or below the kernel
 * priority are deferred.  The original BASEPRI is saved only on the
 * outermost entry, ensuring that nested calls do not overwrite the saved
 * value.  A DSB + ISB pair guarantees the write is committed before any
 * subsequent memory access.
 *
 * @note    This function is safe to call from Thread mode and from ISRs
 *          whose priority is higher than the kernel mask.
 */
void sched_enter_critical(void)
{
    uint32_t current_basepri;

    __asm volatile
    (
        "mrs %0, basepri     \n"
        "msr basepri, %1     \n"
        "dsb                 \n"
        "isb                 \n"
        : "=r" (current_basepri)
        : "r"  (KERNEL_INTERRUPT_MASK)
        : "memory"
    );

    if (critical_nesting == 0U)
    {
        saved_basepri = current_basepri;
    }

    critical_nesting++;
}

/**
 * @brief Exit a nestable critical section and restore BASEPRI.
 *
 * @details
 * Decrements the nesting counter.  When the counter reaches zero the
 * previously saved BASEPRI is restored, re-enabling deferred interrupts.
 * A DSB + ISB pair ensures the barrier is visible before the function
 * returns.  Calls that underflow (counter already 0) are silently ignored.
 */
void sched_exit_critical(void)
{
    if (critical_nesting == 0U)
    {
        return;
    }

    critical_nesting--;

    if (critical_nesting == 0U)
    {
        __asm volatile
        (
            "msr basepri, %0   \n"
            "dsb               \n"
            "isb               \n"
            :
            : "r" (saved_basepri)
            : "memory"
        );
    }
}

/**
 * @brief Pend a PendSV exception to trigger a context switch.
 *
 * @details
 * Sets the PENDSVSET bit in the Interrupt Control and State Register.
 * Because PendSV is configured at the lowest hardware priority, the actual
 * context switch occurs only after all higher-priority ISRs have returned,
 * making this call safe from any interrupt context.
 */
static inline void request_context_switch(void)
{
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

/* =========================================================================
 * SECTION 2 — HELD-MUTEX TABLE
 *
 * Internal bookkeeping that tracks which mutexes are currently owned by
 * each task.  A compact bitmap (held_mutex_bitmap in the TCB) marks
 * occupied slots so that scans terminate early.
 * ========================================================================= */

/**
 * @brief Return the index of task @p t in @c task_init_table.
 *
 * @details
 * Linear scan of the task-initialisation table.  This function is called
 * infrequently (only on mutex lock/unlock paths) so O(n) is acceptable.
 * Issues an undefined-instruction trap (@c udf) if @p t is not found,
 * which triggers HardFault — a deliberate kernel panic for corrupted state.
 *
 * @param[in]  t  Pointer to the TCB whose index is required.
 * @return     Index of @p t in @c task_init_table.
 */
static uint8_t tcb_index_of(TCB_t *t)
{
    for (uint8_t i = 0U; i < task_init_count; i++)
    {
        if (task_init_table[i].tcb == t)
        {
            return i;
        }
    }
    
    __asm volatile("udf #0");
    return 0U;
}

/**
 * @brief Return a pointer to the held-mutex row for task @p t.
 *
 * @param[in]  t  Owning task.
 * @return     Pointer to the first element of @p t's row in
 *             @c held_mutex_table.
 */
static mutex_t **held_row(TCB_t *t)
{
    return held_mutex_table[tcb_index_of(t)];
}

/**
 * @brief Find the lowest free slot in @p t's held-mutex bitmap.
 *
 * @param[in]  t  Owning task.
 * @return     Slot index in [0, HELD_MUTEX_MAX) if a free slot exists,
 *             or @c HELD_MUTEX_MAX if the table is full.
 */
static uint8_t held_alloc_slot(TCB_t *t)
{
    uint16_t bm = t->held_mutex_bitmap;
    for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
    {
        if ((bm & (1U << i)) == 0U)
        {
            return i;
        }
    }
    return HELD_MUTEX_MAX;   
}

/**
 * @brief Record mutex @p m in slot @p slot of task @p t's held table.
 *
 * @param[in]  t     Owning task.
 * @param[in]  slot  Slot index returned by @c held_alloc_slot().
 * @param[in]  m     Mutex to record.
 */
static void held_set(TCB_t *t, uint8_t slot, mutex_t *m)
{
    held_row(t)[slot]    = m;
    t->held_mutex_bitmap |= (uint16_t)(1U << slot);
}

/**
 * @brief Remove mutex @p m from task @p t's held table.
 *
 * @details
 * Searches all occupied slots for @p m and clears the first match.
 * A no-op if @p m is not found (defensive; should not occur in correct code).
 *
 * @param[in]  t  Owning task.
 * @param[in]  m  Mutex to remove.
 */
static void held_clear_mutex(TCB_t *t, mutex_t *m)
{
    mutex_t **row = held_row(t);
    for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
    {
        if (row[i] == m)
        {
            row[i]                = NULL;
            t->held_mutex_bitmap &= (uint16_t)~(1U << i);
            return;
        }
    }
}

/**
 * @brief Return the mutex stored in slot @p slot of task @p t, or NULL.
 *
 * @param[in]  t     Owning task.
 * @param[in]  slot  Slot index to query.
 * @return     Pointer to the held mutex, or @c NULL if the slot is empty.
 */
static mutex_t *held_get(TCB_t *t, uint8_t slot)
{
    if ((t->held_mutex_bitmap & (1U << slot)) == 0U)
    {
        return NULL;
    }
    return held_row(t)[slot];
}

/* =========================================================================
 * SECTION 3 — READY QUEUE
 *
 * O(1) scheduler data structure: a 32-bit bitmap marks non-empty priority
 * levels; each level holds a circular doubly-linked list of TASK_WAKE TCBs.
 * The highest ready task is found with a single CLZ instruction.
 * ========================================================================= */

/**
 * @brief Insert task @p t into the ready queue at its effective priority.
 *
 * @details
 * If the level's list is empty the task becomes a self-loop (single-element
 * circular list) and the priority bit in @c ready_bitmap is set.  Otherwise
 * the task is appended before the current head (tail insertion) to preserve
 * FIFO ordering within a priority level (round-robin).
 *
 * @param[in]  t  TCB to enqueue; must be in @c TASK_WAKE state.
 */
static inline void rq_add(TCB_t *t)
{
    uint8_t p = TCB_EFF_PRIO(t);

    if (ready_queue[p] == NULL)
    {
        t->rq_next     = t;
        t->rq_prev     = t;
        ready_queue[p] = t;
        ready_bitmap  |= (1U << p);
    }
    else
    {
        TCB_t *head   = ready_queue[p];
        TCB_t *tail   = head->rq_prev;
        tail->rq_next = t;
        t->rq_prev    = tail;
        t->rq_next    = head;
        head->rq_prev = t;
    }
}

/**
 * @brief Remove task @p t from the ready queue.
 *
 * @details
 * If @p t is the sole occupant of its priority level the level's head
 * pointer is cleared and the corresponding bit in @c ready_bitmap is
 * cleared.  Otherwise the doubly-linked pointers of the adjacent nodes
 * are updated, and the head pointer is advanced if @p t was the head.
 * The task's own link pointers are nulled to aid debugging.
 *
 * @param[in]  t  TCB to dequeue.
 */
static inline void rq_remove(TCB_t *t)
{
    uint8_t p = TCB_EFF_PRIO(t);

    if (t->rq_next == t)
    {
        ready_queue[p] = NULL;
        ready_bitmap  &= ~(1U << p);
    }
    else
    {
        t->rq_prev->rq_next = t->rq_next;
        t->rq_next->rq_prev = t->rq_prev;

        if (ready_queue[p] == t)
        {
            ready_queue[p] = t->rq_next;
        }
    }

    t->rq_next = NULL;
    t->rq_prev = NULL;
}

/**
 * @brief Return the head of the highest non-empty priority level.
 *
 * @details
 * Uses @c __builtin_clz on the 32-bit @c ready_bitmap to locate the
 * most-significant set bit in a single instruction, yielding O(1)
 * scheduler selection.
 *
 * @return  Pointer to the highest-priority ready TCB, or @c NULL if no
 *          task is ready (should not occur while the idle task is running).
 */
static inline TCB_t *rq_highest(void)
{
    if (ready_bitmap == 0U)
    {
        return NULL;
    }
    uint8_t p = (uint8_t)(31U - __builtin_clz(ready_bitmap));
    return ready_queue[p];
}

/* =========================================================================
 * SECTION 4 — SLEEP LIST
 *
 * A deadline-sorted singly-linked list of sleeping tasks.  SysTick walks
 * the head of this list each tick and wakes all tasks whose deadline has
 * expired.  Insertion is O(n) but the list is expected to be short.
 * ========================================================================= */

/**
 * @brief Insert task @p t into the sleep list in deadline order.
 *
 * @details
 * Sets @p t's state to @c TASK_SLEEP and inserts it into the singly-linked
 * @c sleep_list_head list so that the list remains sorted by ascending
 * @c block_count (deadline in system ticks).  Signed 32-bit arithmetic is
 * used for the comparison so that tick-counter wrap-around is handled
 * correctly for deadlines within ±2³¹ ticks of the current time.
 *
 * @param[in]  t  TCB to insert; @c block_count must already be set to the
 *                wakeup tick.
 */
static void sl_insert(TCB_t *t)
{
    TCB_SET_STATE(t, TASK_SLEEP);
    t->sl_next = NULL;

    if (sleep_list_head == NULL ||
        (int32_t)(t->block_count - sleep_list_head->block_count) <= 0)
    {
        t->sl_next      = sleep_list_head;
        sleep_list_head = t;
        return;
    }

    TCB_t *curr = sleep_list_head;
    while ((curr->sl_next != NULL) &&
           ((int32_t)(t->block_count - curr->sl_next->block_count) > 0))
    {
        curr = curr->sl_next;
    }
    t->sl_next    = curr->sl_next;
    curr->sl_next = t;
}

/* =========================================================================
 * SECTION 5 — WAIT QUEUE
 *
 * A priority-sorted singly-linked list shared by semaphores and mutexes.
 * The highest-priority waiter is always at the head so that O(1) wakeup
 * of the best candidate is possible.  The bitmap mirrors which priority
 * levels are represented, enabling fast highest-priority queries.
 * ========================================================================= */

/**
 * @brief Insert task @p t into the wait queue in priority order.
 *
 * @details
 * Tasks are inserted in descending priority order (highest first).
 * The @p wq_bitmap is updated to reflect the newly occupied priority level.
 * If @p t has strictly higher priority than the current head it becomes the
 * new head; otherwise the list is scanned for the correct insertion point.
 *
 * @param[in,out]  wq_head    Pointer to the wait-queue head pointer.
 * @param[in,out]  wq_bitmap  Pointer to the wait-queue priority bitmap.
 * @param[in]      t          TCB to enqueue; must be in @c TASK_BLOCKED state.
 */
static void wq_enqueue(TCB_t **wq_head, uint32_t *wq_bitmap, TCB_t *t)
{
    uint8_t p   = TCB_EFF_PRIO(t);
    *wq_bitmap |= (1U << p);
    t->wq_next  = NULL;

    if ((*wq_head == NULL) || (p > TCB_EFF_PRIO(*wq_head)))
    {
        t->wq_next = *wq_head;
        *wq_head   = t;
        return;
    }

    TCB_t *curr = *wq_head;
    while ((curr->wq_next != NULL) &&
           (TCB_EFF_PRIO(curr->wq_next) >= p))
    {
        curr = curr->wq_next;
    }
    t->wq_next    = curr->wq_next;
    curr->wq_next = t;
}

/**
 * @brief Remove and return the highest-priority task from the wait queue.
 *
 * @details
 * Removes the head of the wait queue (the highest-priority waiter).
 * If no other task at the same priority level remains the corresponding
 * bit in @p wq_bitmap is cleared.  The returned TCB's @c wq_next pointer
 * is nulled.
 *
 * @param[in,out]  wq_head    Pointer to the wait-queue head pointer.
 * @param[in,out]  wq_bitmap  Pointer to the wait-queue priority bitmap.
 * @return         Pointer to the dequeued TCB, or @c NULL if the queue is
 *                 empty.
 */
static TCB_t *wq_dequeue_highest(TCB_t **wq_head, uint32_t *wq_bitmap)
{
    TCB_t *t = *wq_head;
    if (t == NULL)
    {
        return NULL;
    }

    uint8_t p  = TCB_EFF_PRIO(t);
    *wq_head   = t->wq_next;
    t->wq_next = NULL;

    if ((*wq_head == NULL) || (TCB_EFF_PRIO(*wq_head) != p))
    {
        *wq_bitmap &= ~(1U << p);
    }

    return t;
}

/**
 * @brief Re-insert a blocked task into its mutex wait queue at a new priority.
 *
 * @details
 * Called during Priority Inheritance propagation when a blocked task's
 * effective priority changes.  The task is first unlinked from the mutex
 * wait queue, the old priority's bitmap bit is cleared if no other task
 * at that level remains, the task's effective priority is updated, and
 * then the task is re-inserted at the correct position.  Finally,
 * @c mutex_t::highest_waiting_prio is refreshed.
 *
 * If @p task is not waiting on any mutex the effective priority is simply
 * updated in the TCB without any queue manipulation.
 *
 * @param[in]  task      Blocked TCB whose priority is changing.
 * @param[in]  new_prio  New effective priority to apply.
 */
static void wq_reorder_for_blocked_task(TCB_t *task, uint8_t new_prio)
{
    mutex_t *m = (mutex_t *)task->waiting_on;
    if (m == NULL)
    {
        TCB_SET_EFF_PRIO(task, new_prio);
        return;
    }

    uint8_t old_p = TCB_EFF_PRIO(task);

    /* Unlink task from its current position in the wait queue. */
    if (m->wq_head == task)
    {
        m->wq_head = task->wq_next;
    }
    else
    {
        TCB_t *prev = m->wq_head;
        while ((prev != NULL) && (prev->wq_next != task))
        {
            prev = prev->wq_next;
        }
        if (prev != NULL)
        {
            prev->wq_next = task->wq_next;
        }
    }
    task->wq_next = NULL;

    /* Clear the old priority bit if no other waiter holds that level. */
    {
        uint8_t still_old = 0U;
        TCB_t  *curr      = m->wq_head;
        while (curr != NULL)
        {
            if (TCB_EFF_PRIO(curr) == old_p)
            {
                still_old = 1U;
                break;
            }
            curr = curr->wq_next;
        }
        if (still_old == 0U)
        {
            m->wq_bitmap &= ~(1U << old_p);
        }
    }

    /* Re-insert at new priority. */
    TCB_SET_EFF_PRIO(task, new_prio);
    m->wq_bitmap |= (1U << new_prio);

    if ((m->wq_head == NULL) || (new_prio > TCB_EFF_PRIO(m->wq_head)))
    {
        task->wq_next = m->wq_head;
        m->wq_head    = task;
    }
    else
    {
        TCB_t *curr = m->wq_head;
        while ((curr->wq_next != NULL) &&
               (TCB_EFF_PRIO(curr->wq_next) >= new_prio))
        {
            curr = curr->wq_next;
        }
        task->wq_next = curr->wq_next;
        curr->wq_next = task;
    }

    m->highest_waiting_prio = highest_waiter_priority(m);
}

/* =========================================================================
 * SECTION 6 — MPU & MEMORY PROTECTION
 *
 * One MPU region (STACK_GUARD_MPU_REGION) is reserved as a no-access guard
 * page at the bottom of each task's stack.  On every context switch the
 * region base address is updated to point at the new task's guard page,
 * providing per-task overflow detection at zero additional memory cost.
 * ========================================================================= */

/**
 * @brief Initialise the MPU stack-guard region to a disabled state.
 *
 * @details
 * Selects the guard region via RNR, then clears RBAR and RASR so the
 * region is disabled.  DSB + ISB ensure the writes are visible to the
 * processor pipeline before the first context switch enables the region.
 *
 * @note    Called once during @c core_faults_init().
 */
static void mpu_init(void)
{
    MPU->RNR  = STACK_GUARD_MPU_REGION;
    MPU->RBAR = 0U;
    MPU->RASR = 0U;

    __DSB();
    __ISB();
}

/**
 * @brief Point the MPU stack-guard region at task @p t's guard page.
 *
 * @details
 * Writes the base address of @p t's guard page into RBAR and configures
 * RASR as:
 *  - Region size: 32 bytes (SIZE field = 4, i.e., 2^(4+1) = 32 B)
 *  - Access permissions: no access (AP = 0b000)
 *  - Execute-never: set
 *
 * If @p t is @c NULL or its @c stack_guard_base is @c NULL the region is
 * disabled (RASR = 0), which occurs transiently when the idle task runs and
 * has no guard configured.
 *
 * Called from @c scheduler() on every context switch.
 *
 * @param[in]  t  TCB of the task about to be resumed, or @c NULL.
 */
static void mpu_update_stack_guard(TCB_t *t)
{
    MPU->RNR = STACK_GUARD_MPU_REGION;

    if ((t == NULL) || (t->stack_guard_base == NULL))
    {
        MPU->RASR = 0U;
        return;
    }

    MPU->RBAR = ((uint32_t)t->stack_guard_base)
                | MPU_RBAR_VALID_Msk
                | (STACK_GUARD_MPU_REGION & MPU_RBAR_REGION_Msk);

    MPU->RASR = MPU_RASR_ENABLE_Msk
                | (4U << MPU_RASR_SIZE_Pos)
                | (0U << MPU_RASR_AP_Pos)
                | MPU_RASR_XN_Msk;
}

/**
 * @brief Enable configurable fault handlers and initialise the MPU.
 *
 * @details
 * Enables MemManage, BusFault, and UsageFault in the System Handler Control
 * and State Register so that these faults are routed to their dedicated
 * handlers rather than escalating to HardFault.  Then calls @c mpu_init()
 * to clear the guard region.
 *
 * @note    Must be called before the first context switch so that the MPU
 *          is active when user tasks start executing.
 */
static void core_faults_init(void)
{
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk  |
                  SCB_SHCSR_USGFAULTENA_Msk;

    mpu_init();
}

/* =========================================================================
 * SECTION 7 — TASK CREATION & STACK INITIALISATION
 *
 * Tasks are registered in two phases:
 *   1. create_task()     — allocates a TCB and stack area; appends to the
 *                          ready queue.  Stack frames are NOT pushed yet
 *                          because the MSP is still live.
 *   2. task_stack_init() — called from scheduler_init() after the MSP has
 *                          been rewound; pushes the initial Cortex-M
 *                          exception frame onto every task's PSP stack.
 * ========================================================================= */

/**
 * @brief Unrecoverable kernel error handler.
 *
 * @details
 * Spins forever.  Marked @c noreturn so the compiler can omit epilogue
 * code from callers.  Used as the LR sentinel pushed onto each task's
 * initial stack frame so that returning from a task entry function is
 * caught deterministically rather than causing undefined behaviour.
 */
static void __attribute__((used, noreturn)) kernel_panic(void)
{

    while (1) {}
}

/**
 * @brief Idle task — runs when no user task is ready.
 *
 * @details
 * When @c ENABLE_STOP_MODE is defined the idle task places the MCU into
 * low-power Stop mode (LPDS bit set, PDDS cleared) and waits for any
 * interrupt to resume execution, minimising quiescent current.  Without
 * the define a plain @c WFI is issued instead, halting the CPU clock
 * until the next interrupt while keeping all peripherals active.
 */
static void idle_task(void)
{
    while (1)
    {
#ifdef ENABLE_STOP_MODE
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_CWUF;
        PWR->CR |= PWR_CR_LPDS;
        PWR->CR &= ~PWR_CR_PDDS;

        __asm volatile("wfi");

        PWR->CR &= ~PWR_CR_LPDS;
#else
        __asm volatile("wfi");
#endif
    }
}

/**
 * @brief Carve out a stack area for a new task from the top of SRAM.
 *
 * @details
 * The allocation pointer @c next_task_psp grows downward.  The requested
 * size is rounded up to the nearest 8-word boundary to maintain 8-byte
 * stack alignment as required by the ARM ABI.  A guard-page base address
 * (32-byte aligned, just below the allocated region) is optionally
 * returned via @p guard_base_out for use with the MPU stack-guard region.
 *
 * @warning No bounds check against heap or BSS is performed; the caller is
 *          responsible for ensuring sufficient SRAM remains.
 *
 * @param[in]   stack_words    Requested stack depth in 32-bit words.
 * @param[out]  guard_base_out If non-NULL, receives the 32-byte-aligned
 *                             guard-page base address.
 * @return      Pointer to the top (highest address) of the allocated stack
 *              region; this becomes the initial PSP for the task.
 */
static uint32_t *find_stack_area(uint32_t stack_words, uint32_t **guard_base_out)
{
    stack_words = (stack_words + 7U) & ~7U;

    new_task_psp  = next_task_psp;
    next_task_psp -= stack_words;

    uint32_t guard_addr = ((uint32_t)next_task_psp) & ~31U;

    if (guard_base_out != NULL)
    {
        *guard_base_out = (uint32_t *)guard_addr;
    }

    return new_task_psp;
}

/**
 * @brief Allocate and zero-initialise a new TCB from the kernel pool.
 *
 * @details
 * Obtains a @c TCB_t from the statically-allocated pool managed by
 * @c mem.c and clears all link pointers and the held-mutex bitmap.
 * Returns @c NULL if the pool is exhausted.
 *
 * @return  Pointer to the initialised TCB, or @c NULL on allocation failure.
 */
static TCB_t *alloc_new_tcb_node(void)
{
    TCB_t *node = TCB_pool();
    if (node != NULL)
    {
        node->next_tcb_node     = NULL;
        node->rq_next           = NULL;
        node->rq_prev           = NULL;
        node->sl_next           = NULL;
        node->wq_next           = NULL;
        node->held_mutex_bitmap = 0U;
        node->stack_guard_base  = NULL;
    }
    return node;
}

/**
 * @brief Create and register the built-in idle task.
 *
 * @details
 * Allocates a TCB and a dedicated @c IDLE_TASK_STACK_SIZE-word stack for
 * the idle task.  The idle task is assigned priority 0 (the lowest) and
 * is placed at the head and tail of the global TCB list.  It is the first
 * task added to both @c task_init_table and the ready queue.
 *
 * This function must be called before any user task is created and before
 * @c scheduler_init() is invoked.
 *
 * @note    A @c NULL return from @c alloc_new_tcb_node() causes a silent
 *          return; the kernel will subsequently fault when no task is ready.
 */
static void __attribute((used)) create_idle_task(void)
{
    TCB_t *idle = alloc_new_tcb_node();
    if (idle == NULL)
    {
        return;
    }

    uint32_t *stack_guard = NULL;
    uint32_t *stack = find_stack_area(IDLE_TASK_STACK_SIZE, &stack_guard);

    idle->block_count      = 0U;
    idle->psp_value        = stack;
    idle->waiting_on       = NULL;
    idle->stack_guard_base = stack_guard;

    TCB_INIT_STATE_PRIO(idle, TASK_WAKE, 0U, 0U);

#ifdef SCHED_DEBUG
    idle->name = "idle";
#endif

    if (task_init_count < MAX_TASKS)
    {
        task_init_table[task_init_count].tcb     = idle;
        task_init_table[task_init_count].handler = idle_task;
        task_init_count++;
    }

    link_node = idle;
    head_node = idle;

    rq_add(idle);
}

/**
 * @brief Push initial Cortex-M exception frames onto every task's stack.
 *
 * @details
 * Called from @c scheduler_init() after the MSP has been rewound to free
 * the C stack frame space originally used by @c kite_start() and the
 * scheduler init chain.  For each registered task the function writes a
 * synthetic exception return frame:
 *
 *  | Offset | Register | Value                          |
 *  |--------|----------|--------------------------------|
 *  | +0     | R4–R11   | 0x00000000 (caller-saved, ×8) |
 *  | +32    | R0       | 0x00000000                     |
 *  | +36    | R1       | 0x00000001                     |
 *  | +40    | R2       | 0x00000002                     |
 *  | +44    | R3       | 0x0000000C                     |
 *  | +48    | R12      | (kernel_panic sentinel as LR)  |
 *  | +52    | LR       | task entry-point               |
 *  | +56    | PC       | task entry-point               |
 *  | +60    | xPSR     | 0x01000000 (Thumb bit set)     |
 *
 * The stack pointer stored in the TCB is updated to point at the bottom
 * of this frame so that the first PendSV exception return restores it
 * correctly.
 */
static void __attribute__((used)) task_stack_init(void)
{
    for (uint8_t idx = 0U; idx < task_init_count; idx++)
    {
        TCB_t        *iter    = task_init_table[idx].tcb;
        void        (*handler)(void) = task_init_table[idx].handler;

        uint32_t *stack_top = iter->psp_value;
        uint32_t  aligned   = ((uint32_t)stack_top) & ~0x7U;
        uint32_t *stack     = (uint32_t *)aligned;

        *(--stack) = 0x01000000U;
        *(--stack) = (uint32_t)handler;
        *(--stack) = (uint32_t)kernel_panic;
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
    }
}

/**
 * @brief Register a new user task with the scheduler.
 *
 * @details
 * Allocates a TCB via @c alloc_new_tcb_node() and a stack region via
 * @c find_stack_area(), initialises the TCB fields, appends it to the
 * global TCB list and the ready queue, and records the task in
 * @c task_init_table for deferred stack-frame initialisation.
 *
 * The task does not begin executing until @c kite_start() triggers the
 * first context switch.
 *
 * @param[in]  priority     Static base priority [0, PRIO_LEVELS-1].
 *                          Higher numeric value = higher priority.
 * @param[in]  handler      Task entry-point function; must never return.
 * @param[in]  stack_words  Stack depth in 32-bit words.
 * @param[in]  name         Human-readable task name (stored only when
 *                          @c SCHED_DEBUG is defined).
 *
 * @retval  1  Task created successfully.
 * @retval  0  Creation failed (NULL handler, task-table full, or pool
 *             exhausted).
 */
uint8_t create_task(uint8_t priority, void (*handler)(void),
                    uint32_t stack_words, const char *name)
{
    if (handler == NULL)
    {
        return 0U;
    }

    if (task_init_count >= MAX_TASKS)
    {
        return 0U;
    }

    TCB_t *tcb = alloc_new_tcb_node();
    if (tcb == NULL)
    {
        return 0U;
    }

    uint32_t *stack_guard = NULL;
    uint32_t *stack = find_stack_area(stack_words, &stack_guard);

    tcb->block_count      = 0U;
    tcb->psp_value        = stack;
    tcb->waiting_on       = NULL;
    tcb->stack_guard_base = stack_guard;

    TCB_INIT_STATE_PRIO(tcb, TASK_WAKE, priority, priority);

#ifdef SCHED_DEBUG
    tcb->name = name;
#else
    (void)name;
#endif

    task_init_table[task_init_count].tcb     = tcb;
    task_init_table[task_init_count].handler = handler;
    task_init_count++;
    task_count = task_init_count;

    link_node->next_tcb_node = tcb;
    link_node                = tcb;

    rq_add(tcb);

    return 1U;
}

/* =========================================================================
 * SECTION 8 — SCHEDULER CORE & STARTUP
 *
 * The startup sequence is:
 *   kite_start()      — creates idle task, calls scheduler_init().
 *   scheduler_init()  — saves MSP, rewinds it, calls task_stack_init()
 *                        then jumps to scheduler_start().
 *   scheduler_start() — starts SysTick, arms MPU, issues SVC 0 to
 *                        transfer control to the first task.
 * ========================================================================= */

/**
 * @brief Kernel entry point — initialise and start the scheduler.
 *
 * @details
 * Creates the idle task then invokes @c scheduler_init() which performs
 * the low-level hardware setup and issues the first context switch.
 * This function does not return; control is transferred to the
 * highest-priority ready task via an SVC exception return.
 *
 * @note    All user tasks must be created with @c create_task() before
 *          calling @c kite_start().
 */
void kite_start(void)
{
    create_idle_task();
    scheduler_init();
}

/**
 * @brief Select the highest-priority ready task as the current task.
 *
 * @details
 * Thin wrapper around @c rq_highest() used during the startup sequence
 * before the PendSV ISR takes over scheduling responsibility.
 */
static void find_high_priority_task(void)
{
    current_running_node = rq_highest();
}

/**
 * @brief C-level helper called from @c scheduler_init() with the old MSP.
 *
 * @details
 * Captures the stack pointer value at the point of entry (which is the
 * top of the C-call-chain stack used by @c kite_start()) into
 * @c msp_start so that @c scheduler_init() can rewind the MSP after
 * returning.  Then selects the first task to run and enables the
 * configurable fault handlers and MPU.
 *
 * @note    This function is called from a @c naked function; do not rely
 *          on an ABI-conformant prologue or epilogue.
 */
static void __attribute__((used)) init_helper(void)
{
    msp_start = (uint32_t)next_task_psp;
    find_high_priority_task();
    core_faults_init();
}

/**
 * @brief Low-level scheduler initialisation routine (naked).
 *
 * @details
 * Executes entirely in assembly to allow precise MSP manipulation:
 *
 *  1. Pushes LR (to preserve the return address from @c kite_start).
 *  2. Calls @c init_helper() to capture the current MSP and select the
 *     first task.
 *  3. Rewinds the MSP to @c msp_start, discarding the C startup stack.
 *  4. Calls @c task_stack_init() to push initial exception frames.
 *  5. Branches to @c scheduler_start() (no return).
 *
 * The MSP rewind reclaims stack space that would otherwise remain
 * permanently allocated under a naïve C call chain.
 */
__attribute__((naked, used)) void scheduler_init(void)
{
    __asm volatile(
        "PUSH {R3, LR}          \n"
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

/**
 * @brief Final stage of scheduler startup.
 *
 * @details
 * Starts the SysTick peripheral, arms the MPU stack-guard region for the
 * first task, then issues @c SVC 0 (@c SVC_START_FIRST_TASK) to perform
 * the initial context restore.  This function does not return.
 */
static void scheduler_start(void)
{
    systick_init();
    mpu_update_stack_guard(current_running_node);
    __asm volatile("svc 0");
}

/**
 * @brief Return the current system tick count.
 *
 * @details
 * Provides a lightweight, non-critical read of @c global_systick.  The
 * value may be transiently inconsistent on 16-bit platforms where 32-bit
 * reads are non-atomic, but on Cortex-M the read is a single LDR and is
 * therefore inherently atomic.
 *
 * @return  Number of SysTick interrupts since kernel start (1 ms per tick
 *          when @c TICK_HZ = 1000).
 */
inline uint32_t get_systick_counter(void)
{
    return global_systick;
}

/**
 * @brief Initialise the SysTick peripheral and configure interrupt priorities.
 *
 * @details
 * Programs SysTick to interrupt at @c TICK_HZ using the core clock via
 * @c SysTick_Config().  Then sets interrupt priorities:
 *  - PendSV  → lowest (0xFF) so context switches never pre-empt ISRs.
 *  - SysTick → @c KERNEL_INTERRUPT_PRIORITY.
 *  - SVCall  → @c KERNEL_INTERRUPT_PRIORITY (same level as SysTick so
 *              SVC and SysTick cannot pre-empt each other).
 */
static void systick_init(void)
{
    SysTick_Config(SystemCoreClock / TICK_HZ);

    NVIC_SetPriority(PendSV_IRQn,  0xFF);
    NVIC_SetPriority(SysTick_IRQn, KERNEL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(SVCall_IRQn,  KERNEL_INTERRUPT_PRIORITY);
}

/**
 * @brief Change the round-robin time-slice length at runtime.
 *
 * @details
 * Updates @c time_slice_ticks inside a critical section and resets the
 * sub-tick counter so the new slice takes effect from the next tick.
 * A value of 0 is clamped to 1 to prevent division-by-zero or immediate
 * continuous switching.
 *
 * @param[in]  ticks  Desired time-slice length in system ticks (≥ 1).
 */
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

/* =========================================================================
 * SECTION 9 — TICK & SLEEP WAKEUP
 *
 * The SysTick ISR drives both the sleep-list wakeup logic and the
 * round-robin time-slice counter.  task_wake() scans the head of the
 * sleep list and moves expired entries back to the ready queue.
 * ========================================================================= */

/**
 * @brief Wake all tasks whose sleep deadline has elapsed.
 *
 * @details
 * Walks @c sleep_list_head (which is sorted by ascending @c block_count)
 * and removes every task whose @c block_count ≤ @c global_systick, moving
 * it back to the ready queue with state @c TASK_WAKE.  Stops as soon as a
 * task with a future deadline is encountered.  Signed arithmetic handles
 * tick-counter wrap-around correctly.
 *
 * @return  1 if at least one task was woken (caller should request a
 *          context switch), 0 otherwise.
 */
static uint8_t task_wake(void)
{
    uint8_t woke = 0U;

    while ((sleep_list_head != NULL) &&
           ((int32_t)(global_systick - sleep_list_head->block_count) >= 0))
    {
        TCB_t *t        = sleep_list_head;
        sleep_list_head = t->sl_next;
        t->sl_next      = NULL;
        TCB_SET_STATE(t, TASK_WAKE);
        rq_add(t);
        woke = 1U;
    }

    return woke;
}

/**
 * @brief SysTick interrupt service routine.
 *
 * @details
 * Increments both @c global_systick and the round-robin @c tick_count.
 * Calls @c task_wake() to promote any expired sleeping tasks to the ready
 * queue.  Resets @c tick_count and requests a context switch when the
 * configured time slice has elapsed, implementing cooperative round-robin
 * among tasks at the same priority level.
 */
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

/* =========================================================================
 * SECTION 10 — CONTEXT SWITCH (INTERRUPT LEVEL)
 *
 * The SVC handler decodes the SVC number from the opcode byte preceding
 * the saved PC and dispatches to the appropriate kernel function.
 * PendSV performs the actual register save/restore using stacked exception
 * frames; the scheduler() function selects the next task to run.
 * ========================================================================= */

/**
 * @brief SVC exception entry (naked trampoline).
 *
 * @details
 * Determines whether the call was made from Thread mode (PSP) or Handler
 * mode (MSP) by testing bit 2 of EXC_RETURN (LR), then passes the correct
 * stack frame pointer to @c SVC_Handler_C() in R0.
 */
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

/**
 * @brief C-level SVC dispatcher.
 *
 * @details
 * Extracts the SVC number from the immediate byte of the @c svc instruction
 * (located two bytes before the stacked PC) and dispatches to the
 * appropriate kernel service.  An unrecognised SVC number results in an
 * infinite loop (kernel panic).
 *
 * @param[in]  stack_frame  Pointer to the exception stack frame.  R0–R3 of
 *                          the faulting context are accessible as
 *                          @c stack_frame[0]–[3]; the stacked PC is at
 *                          @c stack_frame[6].
 */
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

/**
 * @brief Restore and launch the first task (SVC 0 handler, naked).
 *
 * @details
 * Reads @c current_running_node, loads the task's PSP from @c psp_value,
 * pops the saved callee-saved registers (R4–R11), sets the PSP, then
 * switches CONTROL to use PSP and unprivileged Thread mode (CONTROL = 3),
 * and performs an exception return with LR = 0xFFFFFFFD (return to Thread
 * mode using PSP, no FP extension).
 *
 * @note    This function restores the entire context of the first task and
 *          therefore does not return in the normal C sense.
 */
__attribute__((naked)) static void svc_start_first_task(void)
{
    __asm volatile(
        "LDR  R0, =current_running_node    \n"
        "LDR  R0, [R0]                     \n"
        
        "LDR  R0, [R0, #0]                 \n"
        "LDMIA R0!, {R4-R11}               \n"
        "MSR  PSP, R0                      \n"
        "MOV  R0, #3                       \n"
        "MSR  CONTROL, R0                  \n"
        "ISB                               \n"
        "LDR  LR, =0xFFFFFFFD              \n"
        "BX   LR                           \n"
    );
}

/**
 * @brief PendSV interrupt service routine — context switch body (naked).
 *
 * @details
 * The PendSV handler performs the full Cortex-M context save/restore cycle:
 *
 *  **Save:**
 *   1. Read PSP into R0.
 *   2. Push R4–R11 below the hardware-stacked frame (@c STMDB).
 *   3. Call @c __set_psp() to record the updated PSP in the current TCB.
 *
 *  **Schedule:**
 *   4. Raise BASEPRI to the kernel mask so the scheduler body runs
 *      atomically.
 *   5. Call @c scheduler() to select the next task.
 *   6. Lower BASEPRI back to 0.
 *
 *  **Restore:**
 *   7. Call @c __get_psp() to read the new task's saved PSP.
 *   8. Pop R4–R11 from the new task's stack.
 *   9. Write PSP and perform exception return (BX LR).
 *
 * The hardware automatically stacks/unstacks R0–R3, R12, LR, PC, xPSR
 * around the exception entry/exit.
 */
__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile(
        "MRS  R0, PSP                       \n"
        "STMDB R0!, {R4-R11}                \n"
        "PUSH {R3, LR}                      \n"
        "BL   __set_psp                     \n"
        "MOV  R1, %[mask]                   \n"
        "MSR  BASEPRI, R1                   \n"
        "DSB                                \n"
        "ISB                                \n"
        "BL   scheduler                     \n"
        "MOV  R1, #0                        \n"
        "MSR  BASEPRI, R1                   \n"
        "BL   __get_psp                     \n"
        "POP  {R3, LR}                      \n"
        "LDMIA R0!, {R4-R11}                \n"
        "MSR  PSP, R0                       \n"
        "ISB                                \n"
        "BX   LR                            \n"
        : : [mask] "i" (KERNEL_INTERRUPT_MASK)
    );
}

/**
 * @brief Return the saved PSP of the current task.
 *
 * @details
 * Called from @c PendSV_Handler() assembly to load the next task's stack
 * pointer into R0 for the LDMIA instruction.
 *
 * @return  Current task's saved PSP as a @c uint32_t (passed in R0).
 */
static uint32_t __attribute__((used)) __get_psp(void)
{
    return (uint32_t)current_running_node->psp_value;
}

/**
 * @brief Save the outgoing task's updated PSP into its TCB.
 *
 * @details
 * Called from @c PendSV_Handler() assembly immediately after the callee-
 * saved registers have been pushed, so the value passed in R0 reflects the
 * true top of the outgoing task's stack.
 *
 * @param[in]  current_psp_value  Updated PSP value to store in the TCB.
 */
static void __attribute__((used)) __set_psp(uint32_t current_psp_value)
{
    current_running_node->psp_value = (uint32_t *)current_psp_value;
}

/**
 * @brief Select the next task to run and update the MPU guard.
 *
 * @details
 * If the outgoing task is still in @c TASK_WAKE state it is rotated to
 * the tail of its ready-queue level (yielding its time slice to peers at
 * the same priority).  @c rq_highest() then picks the globally highest-
 * priority ready task, which becomes @c current_running_node.  Finally, the
 * MPU stack-guard region is reprogrammed for the incoming task.
 *
 * Called from @c PendSV_Handler() with BASEPRI at the kernel mask.
 */
static void __attribute__((used)) scheduler(void)
{
    if ((current_running_node != NULL) &&
        (TCB_STATE(current_running_node) == TASK_WAKE))
    {
        rq_remove(current_running_node);
        rq_add(current_running_node);
    }

    current_running_node = rq_highest();

    mpu_update_stack_guard(current_running_node);
}

/* =========================================================================
 * SECTION 11 — TASK CONTROL PUBLIC API
 *
 * All blocking calls are routed through SVC so they are guaranteed to
 * execute atomically with respect to SysTick even if called from Thread
 * mode with interrupts enabled.  The SVC gateway functions place arguments
 * in the correct registers before the trap, matching the ABI expected by
 * SVC_Handler_C which reads them from the stacked frame.
 * ========================================================================= */

/**
 * @brief Voluntarily yield the CPU to the next ready task at any priority.
 *
 * @details
 * Issues @c SVC 1 (@c SVC_YIELD), which calls @c request_context_switch()
 * to pend PendSV.  The calling task remains in @c TASK_WAKE state and will
 * be rotated to the tail of its priority level's ready queue, allowing
 * same-priority peers to run before it is scheduled again.
 */
void task_yield(void)
{
    __asm volatile("svc 1" ::: "memory");
}

/**
 * @brief Delay the calling task by @p ticks system ticks.
 *
 * @details
 * Issues @c SVC 2 (@c SVC_DELAY) with @p ticks in R0.  The kernel removes
 * the task from the ready queue, computes an absolute wakeup deadline of
 * @c global_systick + @p ticks, and inserts it into the sleep list.  A
 * value of 0 results in an immediate context switch with no delay.
 *
 * @param[in]  ticks  Number of system ticks to sleep (0 = yield).
 */
void task_delay(uint32_t ticks)
{
    register uint32_t r0 __asm("r0") = ticks;
    __asm volatile("svc 2" : : "r"(r0) : "memory");
}

/**
 * @brief Sleep until an absolute periodic deadline.
 *
 * @details
 * Issues @c SVC 3 (@c SVC_SLEEP_UNTIL) with @p last_wake in R0 and
 * @p period in R1.  The kernel advances @c *last_wake by @p period and
 * sleeps until that absolute tick, compensating automatically for any
 * execution jitter accumulated in the previous period.  Useful for
 * implementing fixed-rate control loops without drift.
 *
 * @param[in,out]  last_wake  Pointer to the reference tick; updated to the
 *                            next wakeup deadline on each call.  Must be
 *                            initialised to @c get_systick_counter() before
 *                            the first call.
 * @param[in]      period     Period in system ticks.
 */
void task_sleep_until(uint32_t *last_wake, uint32_t period)
{
    register uint32_t *r0 __asm("r0") = last_wake;
    register uint32_t  r1 __asm("r1") = period;
    __asm volatile("svc 3" : : "r"(r0), "r"(r1) : "memory");
}

/**
 * @brief SVC implementation: delay the current task by @p ticks ticks.
 *
 * @details
 * Computes the absolute deadline, removes the task from the ready queue, and
 * inserts it into the sleep list via @c sl_insert().  A @c ticks value of
 * 0 or a @c NULL @c current_running_node causes an immediate context switch
 * without any sleep.
 *
 * @param[in]  ticks  Number of ticks to sleep.
 */
static void svc_task_delay(uint32_t ticks)
{
    if ((current_running_node == NULL) || (ticks == 0U))
    {
        request_context_switch();
        return;
    }

    ENTER_CRITICAL();

    current_running_node->block_count = global_systick + ticks;

    rq_remove(current_running_node);
    sl_insert(current_running_node);

    EXIT_CRITICAL();

    request_context_switch();
}

/**
 * @brief SVC implementation: sleep until an absolute periodic deadline.
 *
 * @details
 * Advances @c *last_wake by @p period to compute the next absolute wakeup
 * tick.  If the deadline is still in the future the task is moved to the
 * sleep list; if it has already passed the task remains in the ready queue
 * and a context switch is requested to allow other tasks at the same
 * priority to run.
 *
 * @param[in,out]  last_wake  Reference tick; updated to the next deadline.
 * @param[in]      period     Period length in system ticks.
 */
static void svc_task_sleep_until(uint32_t *last_wake, uint32_t period)
{
    if ((current_running_node == NULL) || (last_wake == NULL))
    {
        return;
    }

    ENTER_CRITICAL();

    uint32_t next_wake = *last_wake + period;
    *last_wake         = next_wake;

    if ((int32_t)(next_wake - global_systick) > 0)
    {
        current_running_node->block_count = next_wake;
        rq_remove(current_running_node);
        sl_insert(current_running_node);
    }

    EXIT_CRITICAL();

    request_context_switch();
}

/* =========================================================================
 * SECTION 12 — SEMAPHORE
 *
 * Counting semaphore with a priority-ordered wait queue.  The public API
 * (semaphore_init / semaphore_wait / semaphore_post) is the application
 * interface; the svc_* variants are invoked only by the SVC dispatcher.
 * ========================================================================= */

/**
 * @brief Initialise a semaphore to a given count.
 *
 * @details
 * Must be called before the semaphore is used by any task.  Safe to call
 * before @c kite_start() since it performs only a plain struct write.
 *
 * @param[out]  sem            Semaphore to initialise; must not be @c NULL.
 * @param[in]   initial_count  Starting count; negative values are valid and
 *                             mean the semaphore is immediately unavailable.
 */
void semaphore_init(semaphore_t *sem, int32_t initial_count)
{
    sem->count     = initial_count;
    sem->wq_head   = NULL;
    sem->wq_bitmap = 0U;
}

/**
 * @brief Decrement the semaphore, blocking if the count would go negative.
 *
 * @details
 * Gateway to @c SVC_SEM_WAIT (SVC 4).  Places @p sem in R0 before the
 * trap so @c SVC_Handler_C() can extract it from the stacked frame.
 *
 * @param[in]  sem  Semaphore to acquire; must not be @c NULL.
 */
void semaphore_wait(semaphore_t *sem)
{
    register semaphore_t *r0 __asm("r0") = sem;
    __asm volatile("svc 4" : : "r"(r0) : "memory");
}

/**
 * @brief Increment the semaphore, waking the highest-priority waiter if any.
 *
 * @details
 * Gateway to @c SVC_SEM_POST (SVC 5).  Places @p sem in R0 before the
 * trap so @c SVC_Handler_C() can extract it from the stacked frame.
 *
 * @param[in]  sem  Semaphore to release; must not be @c NULL.
 */
void semaphore_post(semaphore_t *sem)
{
    register semaphore_t *r0 __asm("r0") = sem;
    __asm volatile("svc 5" : : "r"(r0) : "memory");
}

/**
 * @brief SVC implementation: decrement semaphore or block the calling task.
 *
 * @details
 * Decrements @c sem->count inside a critical section.  If the count drops
 * below zero the task is moved to the semaphore's wait queue and a context
 * switch is requested.  If the count remains ≥ 0 the task continues without
 * blocking.
 *
 * @param[in]  sem  Target semaphore; silently ignored if @c NULL.
 */
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
        current_running_node->waiting_on = sem;
        TCB_SET_STATE(current_running_node, TASK_BLOCKED);

        rq_remove(current_running_node);
        wq_enqueue(&sem->wq_head, &sem->wq_bitmap, current_running_node);

        EXIT_CRITICAL();
        request_context_switch();
        return;
    }

    EXIT_CRITICAL();
}

/**
 * @brief SVC implementation: increment semaphore and wake the best waiter.
 *
 * @details
 * Increments @c sem->count inside a critical section.  If the count is
 * still ≤ 0 at least one task is blocked; the highest-priority waiter is
 * dequeued, moved to @c TASK_WAKE, added to the ready queue, and a context
 * switch is requested so that a higher-priority newly-woken task can run
 * immediately.
 *
 * @param[in]  sem  Target semaphore; silently ignored if @c NULL.
 */
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
        TCB_t *best = wq_dequeue_highest(&sem->wq_head, &sem->wq_bitmap);

        if (best != NULL)
        {
            best->waiting_on = NULL;
            TCB_SET_STATE(best, TASK_WAKE);
            rq_add(best);

            EXIT_CRITICAL();
            request_context_switch();
            return;
        }
    }

    EXIT_CRITICAL();
}

/* =========================================================================
 * SECTION 13 — MUTEX & PRIORITY INHERITANCE
 *
 * Binary mutex with full Priority Inheritance (PI).  When a high-priority
 * task blocks on a locked mutex the owner's effective priority is boosted
 * to at least that of the waiter.  The boost propagates transitively along
 * chains of blocking mutexes.  On unlock, the owner's priority is restored
 * to the maximum of its base priority and the highest_waiting_prio of any
 * remaining held mutexes.
 * ========================================================================= */

/**
 * @brief Initialise a mutex to the unlocked state.
 *
 * @details
 * Must be called before the mutex is used by any task.  Safe to call
 * before @c kite_start().
 *
 * @param[out]  m  Mutex to initialise; must not be @c NULL.
 */
void mutex_init(mutex_t *m)
{
    m->locked               = 0U;
    m->owner                = NULL;
    m->highest_waiting_prio = 0U;
    m->wq_head              = NULL;
    m->wq_bitmap            = 0U;
}

/**
 * @brief Acquire the mutex, blocking if it is already held.
 *
 * @details
 * Gateway to @c SVC_MUTEX_LOCK (SVC 6).  Places @p m in R0 before the
 * trap.  If the mutex is free the calling task becomes the owner
 * immediately.  If it is already locked the task blocks and Priority
 * Inheritance is applied to the owner chain.
 *
 * @warning Must not be called from an ISR context.
 *
 * @param[in]  m  Mutex to acquire; must not be @c NULL.
 */
void mutex_lock(mutex_t *m)
{
    register mutex_t *r0 __asm("r0") = m;
    __asm volatile("svc 6" : : "r"(r0) : "memory");
}

/**
 * @brief Release the mutex and wake the highest-priority waiter.
 *
 * @details
 * Gateway to @c SVC_MUTEX_UNLOCK (SVC 7).  Places @p m in R0 before the
 * trap.  Only the current owner may unlock.  On release, the owner's
 * priority is restored and the next waiter (if any) inherits ownership.
 *
 * @warning Must not be called from an ISR context.
 *
 * @param[in]  m  Mutex to release; must not be @c NULL, and the calling
 *                task must be the current owner.
 */
void mutex_unlock(mutex_t *m)
{
    register mutex_t *r0 __asm("r0") = m;
    __asm volatile("svc 7" : : "r"(r0) : "memory");
}

/**
 * @brief Return the effective priority of the highest-priority waiter.
 *
 * @details
 * Uses @c __builtin_clz on the wait-queue bitmap for O(1) lookup.
 *
 * @param[in]  m  Mutex whose wait queue is queried.
 * @return     Priority of the highest waiter, or 0 if no waiters exist.
 */
static uint8_t highest_waiter_priority(mutex_t *m)
{
    if (m->wq_bitmap == 0U)
    {
        return 0U;
    }
    return (uint8_t)(31U - __builtin_clz(m->wq_bitmap));
}

/**
 * @brief Change a task's effective priority and reposition it in its queue.
 *
 * @details
 * If the task is in @c TASK_WAKE state it is removed from and re-inserted
 * into the ready queue at @p new_prio.  If it is in @c TASK_BLOCKED state
 * @c wq_reorder_for_blocked_task() repositions it within its mutex's wait
 * queue.  For tasks in any other state (e.g., @c TASK_SLEEP) the priority
 * is updated in the TCB only; no queue reordering is performed.
 *
 * @param[in]  task      Task whose priority is to be updated.
 * @param[in]  new_prio  Target effective priority.
 * @retval     1  The priority was changed.
 * @retval     0  The priority was already equal to @p new_prio; no change.
 */
static uint8_t update_task_priority(TCB_t *task, uint8_t new_prio)
{
    if (TCB_EFF_PRIO(task) == new_prio)
    {
        return 0U;
    }

    if (TCB_STATE(task) == TASK_WAKE)
    {
        rq_remove(task);
        TCB_SET_EFF_PRIO(task, new_prio);
        rq_add(task);
    }
    else if (TCB_STATE(task) == TASK_BLOCKED)
    {
        wq_reorder_for_blocked_task(task, new_prio);
    }
    else
    {
        TCB_SET_EFF_PRIO(task, new_prio);
    }

    return 1U;
}

/**
 * @brief Propagate a priority boost transitively along a mutex wait chain.
 *
 * @details
 * Implements the transitive Priority Inheritance algorithm.  Starting from
 * mutex @p m, the function iterates along the chain of ownership:
 *
 *  - If @p prio exceeds the mutex's current @c highest_waiting_prio, that
 *    field is updated.
 *  - If the owner's effective priority is already ≥ @p prio, propagation
 *    stops (the chain is already boosted to at least this level).
 *  - Otherwise the owner's priority is raised to @p prio.
 *  - If the owner is itself blocked on another mutex the loop continues
 *    to that mutex, propagating the boost further up the chain.
 *  - Propagation also stops when an owner is not blocked (it is running or
 *    sleeping), as priority inheritance only applies to ready/blocked tasks.
 *
 * @param[in]  m     Mutex at the base of the ownership chain.
 * @param[in]  prio  Priority to propagate.
 */
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

        if (TCB_EFF_PRIO(owner) >= prio)
        {
            break;
        }

        update_task_priority(owner, prio);

        if (TCB_STATE(owner) != TASK_BLOCKED)
        {
            break;
        }

        current_m = (mutex_t *)owner->waiting_on;
    }
}

/**
 * @brief SVC implementation: acquire a mutex or block with PI.
 *
 * @details
 * If the mutex is unlocked a held-table slot is allocated for the calling
 * task, the mutex is marked locked, and the task continues without
 * blocking.  If the slot table is full (exceeds @c HELD_MUTEX_MAX) a
 * deliberate undefined instruction is executed to trigger HardFault.
 *
 * If the mutex is already locked the calling task is moved to
 * @c TASK_BLOCKED, recorded as waiting on @p m, enqueued in the mutex wait
 * queue, and @c propagate_priority() boosts the owner chain.  A context
 * switch is requested so the CPU is yielded.
 *
 * @param[in]  m  Mutex to acquire; silently ignored if @c NULL.
 */
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
        uint8_t slot = held_alloc_slot(current_running_node);

        if (slot < HELD_MUTEX_MAX)
        {
            m->locked               = 1U;
            m->owner                = current_running_node;
            m->highest_waiting_prio = 0U;
            held_set(current_running_node, slot, m);
        }
        else
        {
            __asm volatile("udf #0");
        }
    }
    else
    {
        current_running_node->waiting_on = m;
        TCB_SET_STATE(current_running_node, TASK_BLOCKED);

        uint8_t my_prio = TCB_EFF_PRIO(current_running_node);
        if (my_prio > m->highest_waiting_prio)
        {
            m->highest_waiting_prio = my_prio;
        }

        rq_remove(current_running_node);
        wq_enqueue(&m->wq_head, &m->wq_bitmap, current_running_node);

        propagate_priority(m, my_prio);

        need_switch = 1U;
    }

    EXIT_CRITICAL();

    if (need_switch != 0U)
    {
        request_context_switch();
    }
}

/**
 * @brief SVC implementation: release a mutex and restore owner priority.
 *
 * @details
 * Verifies the caller is the current owner (no-op otherwise).  Removes the
 * mutex from the owner's held table.  If a waiter exists, the highest-
 * priority waiter is dequeued and granted ownership atomically; its
 * effective priority may be further boosted if higher waiters remain.
 * If no waiters exist the mutex is simply marked unlocked.
 *
 * After transfer (or release), the original owner's effective priority is
 * recomputed as the maximum of its base priority and
 * @c highest_waiting_prio across all remaining held mutexes, undoing any
 * inherited boost that is no longer justified.
 *
 * A context switch is requested if the priority change or new owner could
 * require rescheduling.
 *
 * @param[in]  m  Mutex to release; silently ignored if @c NULL or if the
 *                caller is not the owner.
 */
static void svc_mutex_unlock(mutex_t *m)
{
    uint8_t need_switch = 0U;

    if ((m == NULL) || (m->owner != current_running_node))
    {
        return;
    }

    ENTER_CRITICAL();

    held_clear_mutex(current_running_node, m);

    TCB_t *next_owner = wq_dequeue_highest(&m->wq_head, &m->wq_bitmap);

    if (next_owner != NULL)
    {
        uint8_t slot = held_alloc_slot(next_owner);

        if (slot < HELD_MUTEX_MAX)
        {
            held_set(next_owner, slot, m);
        }
        else
        {
            __asm volatile("udf #0");
        }

        next_owner->waiting_on = NULL;
        TCB_SET_STATE(next_owner, TASK_WAKE);
        rq_add(next_owner);

        m->owner                = next_owner;
        m->locked               = 1U;
        m->highest_waiting_prio = highest_waiter_priority(m);

        if (m->highest_waiting_prio > TCB_EFF_PRIO(next_owner))
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

    /* Restore the unlocker's priority to the maximum still justified. */
    if (TCB_EFF_PRIO(current_running_node) != TCB_BASE_PRIO(current_running_node))
    {
        uint8_t max_inherited = TCB_BASE_PRIO(current_running_node);

        for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
        {
            mutex_t *held = held_get(current_running_node, i);
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

/* =========================================================================
 * SECTION 14 — FAULT HANDLERS
 *
 * Hard fault, MemManage, BusFault, and UsageFault handlers.  All spin
 * forever to halt the system in a detectable state.  The HardFault handler
 * passes the faulting stack frame to the C function hardfault() for
 * optional debugger inspection.
 * ========================================================================= */

/**
 * @brief HardFault exception entry (naked trampoline).
 *
 * @details
 * Reads bit 2 of EXC_RETURN to determine whether the fault occurred in
 * Thread mode (PSP) or Handler mode (MSP) and branches to @c hardfault()
 * with the relevant stack frame pointer in R0.  This allows a debugger to
 * read the stacked PC, LR, xPSR, and general-purpose registers to identify
 * the faulting instruction.
 */
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

/**
 * @brief C-level HardFault handler — halts the system.
 *
 * @details
 * Receives the faulting stack frame so a debugger can inspect the saved
 * context (PC at @c stack[6], LR at @c stack[5], xPSR at @c stack[7]).
 * Spins indefinitely; in a production system this could trigger a watchdog
 * reset or log diagnostic data to non-volatile storage.
 *
 * @param[in]  stack  Pointer to the exception stack frame.
 */
void hardfault(uint32_t *stack)
{
    (void)stack;
    while (1);
}

/**
 * @brief MemManage fault handler — halts the system.
 *
 * @details
 * Triggered by MPU access violations (including stack-guard hits) or
 * other memory-protection faults.  Spins indefinitely.
 */
void MemManage_Handler(void)
{
    
    while (1);
}

/**
 * @brief BusFault handler — halts the system.
 *
 * @details
 * Triggered by bus errors such as invalid memory accesses detected by the
 * AHB interconnect.  Spins indefinitely.
 */
void BusFault_Handler(void)   
{ 

    while (1);
}

/**
 * @brief UsageFault handler — halts the system.
 *
 * @details
 * Triggered by undefined instructions, unaligned accesses (if enabled),
 * invalid EXC_RETURN values, and similar software-detectable faults.
 * Spins indefinitely.
 */
void UsageFault_Handler(void) 
{
    
     while (1);     
}