#include <stddef.h>
#include <stdint.h>

#include "mcu.h"
#include "sched.h"
#include "mem.h"


volatile uint32_t global_systick    = 0U;
TCB_t            *current_running_node = NULL;


static uint32_t *next_task_psp  = STACK_START;
static uint32_t  msp_start;

static uint32_t tick_count       = 0U;
static uint32_t time_slice_ticks = SCHED_TIME_SLICE;

static TCB_t   *ready_queue[PRIO_LEVELS];
static uint32_t ready_bitmap    = 0U;
static TCB_t   *sleep_list_head = NULL;

static mutex_t *held_mutex_table[MAX_TASKS][HELD_MUTEX_MAX];

typedef struct {
    TCB_t *tcb;
    void (*handler)(void);
} task_init_entry_t;

static task_init_entry_t task_init_table[MAX_TASKS];
static uint8_t           task_init_count = 0U;

static uint32_t critical_nesting = 0U;
static uint32_t saved_basepri    = 0U;


static inline void request_context_switch(void);

__attribute__((naked, used)) void scheduler_init(void);
static void __attribute__((used)) scheduler_start(void);
static void                       systick_init(void);
void                              SysTick_Handler(void);
static uint8_t                    task_wake(void);
static void __attribute__((used)) init_helper(void);

static void core_faults_init(void);
static void mpu_init(void);
static void mpu_update_stack_guard(TCB_t *t);

static uint32_t *find_stack_area(uint32_t stack_words, uint32_t **guard_base_out);
static TCB_t    *alloc_new_tcb(void);
static void      create_idle_task(void);
static void __attribute__((used))           task_stack_init(void);
static void __attribute__((used, noreturn)) kernel_panic(void);
static void idle_task(void);

static void __attribute__((used)) SVC_Handler_C(uint32_t *stack_frame);
__attribute__((naked)) static void svc_start_first_task(void);

static uint8_t update_task_priority(TCB_t *task, uint8_t new_prio);
static uint8_t highest_waiter_priority(mutex_t *m);
static void    propagate_priority(mutex_t *m, uint8_t prio);

static void svc_task_delay(uint32_t ticks);
static void svc_task_sleep_until(uint32_t *last_wake, uint32_t period);
static void svc_semaphore_wait(semaphore_t *sem);
static void svc_semaphore_post(semaphore_t *sem);
static void svc_mutex_lock(mutex_t *m);
static void svc_mutex_unlock(mutex_t *m);


void sched_enter_critical(void)
{
    uint32_t prev;
    __asm volatile (
        "mrs  %0, basepri   \n"
        "msr  basepri, %1   \n"
        "dsb                \n"
        "isb                \n"
        : "=r" (prev)
        : "r"  (KERNEL_INTERRUPT_MASK)
        : "memory"
    );
    if (critical_nesting == 0U)
    {
        saved_basepri = prev;
    }
    critical_nesting++;
}


void sched_exit_critical(void)
{
    critical_nesting--;
    if (critical_nesting == 0U)
    {
        __asm volatile (
            "msr  basepri, %0   \n"
            "dsb                \n"
            "isb                \n"
            :
            : "r" (saved_basepri)
            : "memory"
        );
    }
}


static inline void request_context_switch(void)
{
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}


static inline mutex_t **held_row(TCB_t *t)
{
    return held_mutex_table[t->tcb_idx];
}


static inline uint8_t held_alloc_slot(TCB_t *t)
{
    uint16_t bm   = t->held_mutex_bitmap;
    uint16_t full = (uint16_t)((1U << HELD_MUTEX_MAX) - 1U);
    if (bm == full)
    {
        return HELD_MUTEX_MAX;
    }
    return (uint8_t)__builtin_ctz((uint32_t)~bm);
}


static inline void held_set(TCB_t *t, uint8_t slot, mutex_t *m)
{
    held_row(t)[slot]     = m;
    t->held_mutex_bitmap |= (uint16_t)(1U << slot);
}


static inline void held_clear_mutex(TCB_t *t, mutex_t *m)
{
    mutex_t **row = held_row(t);
    uint32_t  bm  = (uint32_t)t->held_mutex_bitmap;
    while (bm != 0U)
    {
        uint8_t slot = (uint8_t)__builtin_ctz(bm);
        if (row[slot] == m)
        {
            row[slot]              = NULL;
            t->held_mutex_bitmap  &= (uint16_t)~(1U << slot);
            return;
        }
        bm &= bm - 1U;
    }
}


static inline mutex_t *held_get(TCB_t *t, uint8_t slot)
{
    if ((t->held_mutex_bitmap & (uint16_t)(1U << slot)) == 0U)
    {
        return NULL;
    }
    return held_row(t)[slot];
}


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


static inline TCB_t *rq_highest(void)
{
    if (ready_bitmap == 0U)
    {
        return NULL;
    }
    uint8_t p = (uint8_t)(31U - __builtin_clz(ready_bitmap));
    return ready_queue[p];
}


static void sl_insert(TCB_t *t)
{
    TCB_SET_STATE(t, TASK_SLEEP);
    t->sl_next = NULL;

    if ((sleep_list_head == NULL) ||
        ((int32_t)(t->block_count - sleep_list_head->block_count) <= 0))
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


static void wq_reorder_for_blocked_task(TCB_t *task, uint8_t new_prio)
{
    mutex_t *m = (mutex_t *)task->waiting_on;
    if (m == NULL)
    {
        TCB_SET_EFF_PRIO(task, new_prio);
        return;
    }

    uint8_t old_p = TCB_EFF_PRIO(task);

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

    TCB_SET_EFF_PRIO(task, new_prio);
    wq_enqueue(&m->wq_head, &m->wq_bitmap, task);
    m->highest_waiting_prio = highest_waiter_priority(m);
}


static void mpu_init(void)
{
    MPU->RNR  = STACK_GUARD_MPU_REGION;
    MPU->RBAR = 0U;
    MPU->RASR = 0U;
    __DSB();
    __ISB();
}


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


static void core_faults_init(void)
{
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk
               |  SCB_SHCSR_BUSFAULTENA_Msk
               |  SCB_SHCSR_USGFAULTENA_Msk;
    mpu_init();
}

static void idle_task(void)
{
    while (1)
    {
#ifdef ENABLE_STOP_MODE
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_CWUF;
        PWR->CR |= PWR_CR_LPDS;
        PWR->CR &= ~PWR_CR_PDDS;
        __asm volatile ("wfi");
        PWR->CR &= ~PWR_CR_LPDS;
#else
        __asm volatile ("wfi");
#endif
    }
}


static uint32_t *find_stack_area(uint32_t stack_words, uint32_t **guard_base_out)
{
    stack_words    = (stack_words + 7U) & ~7U;
    uint32_t *top  = next_task_psp;
    next_task_psp -= stack_words;
    *guard_base_out = (uint32_t *)(((uint32_t)next_task_psp) & ~31U);
    return top;
}


static TCB_t *alloc_new_tcb(void)
{
    TCB_t *t = TCB_pool();
    if (t != NULL)
    {
        t->rq_next           = NULL;
        t->rq_prev           = NULL;
        t->sl_next           = NULL;
        t->wq_next           = NULL;
        t->held_mutex_bitmap = 0U;
        t->stack_guard_base  = NULL;
        t->waiting_on        = NULL;
    }
    return t;
}


static void create_idle_task(void)
{
    TCB_t *idle = alloc_new_tcb();
    if (idle == NULL)
    {
        return;
    }

    uint32_t *guard  = NULL;
    idle->psp_value        = find_stack_area(IDLE_TASK_STACK_SIZE, &guard);
    idle->block_count      = 0U;
    idle->stack_guard_base = guard;
    idle->tcb_idx          = task_init_count;
    TCB_INIT_STATE_PRIO(idle, TASK_WAKE, 0U, 0U);

#ifdef SCHED_DEBUG
    idle->name = "idle";
#endif

    task_init_table[task_init_count].tcb     = idle;
    task_init_table[task_init_count].handler = idle_task;
    task_init_count++;

    rq_add(idle);
}


static void __attribute__((used)) task_stack_init(void)
{
    for (uint8_t i = 0U; i < task_init_count; i++)
    {
        TCB_t *t         = task_init_table[i].tcb;
        void (*fn)(void) = task_init_table[i].handler;

        uint32_t *sp = (uint32_t *)(((uint32_t)t->psp_value) & ~0x7U);

        *(--sp) = 0x01000000U;
        *(--sp) = (uint32_t)fn;
        *(--sp) = (uint32_t)kernel_panic;
        *(--sp) = 0x0000000CU;
        *(--sp) = 0x00000003U;
        *(--sp) = 0x00000002U;
        *(--sp) = 0x00000001U;
        *(--sp) = 0x00000000U;
        *(--sp) = 0U;
        *(--sp) = 0U;
        *(--sp) = 0U;
        *(--sp) = 0U;
        *(--sp) = 0U;
        *(--sp) = 0U;
        *(--sp) = 0U;
        *(--sp) = 0U;

        t->psp_value = sp;
    }
}


uint8_t create_task(uint8_t priority, void (*handler)(void),
                    uint32_t stack_words, const char *name)
{
    if ((handler == NULL) || (task_init_count >= MAX_TASKS))
    {
        return 0U;
    }

    TCB_t *tcb = alloc_new_tcb();
    if (tcb == NULL)
    {
        return 0U;
    }

    uint32_t *guard       = NULL;
    tcb->psp_value        = find_stack_area(stack_words, &guard);
    tcb->block_count      = 0U;
    tcb->stack_guard_base = guard;
    tcb->tcb_idx          = task_init_count;
    TCB_INIT_STATE_PRIO(tcb, TASK_WAKE, priority, priority);

#ifdef SCHED_DEBUG
    tcb->name = name;
#else
    (void)name;
#endif

    task_init_table[task_init_count].tcb     = tcb;
    task_init_table[task_init_count].handler = handler;
    task_init_count++;

    rq_add(tcb);
    return 1U;
}


void kite_start(void)
{
    create_idle_task();
    scheduler_init();
}


static void __attribute__((used)) init_helper(void)
{
    msp_start            = (uint32_t)next_task_psp;
    current_running_node = rq_highest();
    core_faults_init();
}


__attribute__((naked, used)) void scheduler_init(void)
{
    __asm volatile (
        "PUSH {R3, LR}        \n"
        "BL   init_helper     \n"
        "POP  {R3, LR}        \n"
        "LDR  R0, =msp_start  \n"
        "LDR  R0, [R0]        \n"
        "MSR  MSP, R0         \n"
        "ISB                  \n"
        "BL   task_stack_init \n"
        "B    scheduler_start \n"
    );
}


static void scheduler_start(void)
{
    systick_init();
    mpu_update_stack_guard(current_running_node);
    __asm volatile ("svc 0");
}


inline uint32_t get_systick_counter(void)
{
    return global_systick;
}


static void systick_init(void)
{
    SysTick_Config(SystemCoreClock / TICK_HZ);
    NVIC_SetPriority(PendSV_IRQn,  0xFFU);
    NVIC_SetPriority(SysTick_IRQn, KERNEL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(SVCall_IRQn,  KERNEL_INTERRUPT_PRIORITY);
}


void set_time_slice_ticks(uint32_t ticks)
{
    if (ticks == 0U)
    {
        ticks = 1U;
    }
    ENTER_CRITICAL();
    time_slice_ticks = ticks;
    tick_count       = 0U;
    EXIT_CRITICAL();
}


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
    __asm volatile (
        "TST   LR, #4        \n"
        "ITE   EQ            \n"
        "MRSEQ R0, MSP       \n"
        "MRSNE R0, PSP       \n"
        "B     SVC_Handler_C \n"
    );
}


static void __attribute__((used)) SVC_Handler_C(uint32_t *stack_frame)
{
    uint8_t svc_number = ((uint8_t *)stack_frame[6])[-2];

    switch (svc_number)
    {
        case SVC_START_FIRST_TASK:  svc_start_first_task();                                break;
        case SVC_YIELD:             request_context_switch();                              break;
        case SVC_DELAY:             svc_task_delay(stack_frame[0]);                        break;
        case SVC_SLEEP_UNTIL:       svc_task_sleep_until((uint32_t *)stack_frame[0],
                                                          stack_frame[1]);                 break;
        case SVC_SEM_WAIT:          svc_semaphore_wait((semaphore_t *)stack_frame[0]);     break;
        case SVC_SEM_POST:          svc_semaphore_post((semaphore_t *)stack_frame[0]);     break;
        case SVC_MUTEX_LOCK:        svc_mutex_lock((mutex_t *)stack_frame[0]);             break;
        case SVC_MUTEX_UNLOCK:      svc_mutex_unlock((mutex_t *)stack_frame[0]);           break;
        default:                    kernel_panic();                                        break;
    }
}


__attribute__((naked)) static void svc_start_first_task(void)
{
    __asm volatile (
        "LDR   R0, =current_running_node \n"
        "LDR   R0, [R0]                  \n"
        "LDR   R0, [R0, #0]              \n"
        "LDMIA R0!, {R4-R11}             \n"
        "MSR   PSP, R0                   \n"
        "MOV   R0, #3                    \n"
        "MSR   CONTROL, R0               \n"
        "ISB                             \n"
        "LDR   LR, =0xFFFFFFFD           \n"
        "BX    LR                        \n"
    );
}


__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile (
        "MRS    R0, PSP                      \n"
        "STMDB  R0!, {R4-R11}                \n"

        "LDR    R3, =current_running_node    \n"
        "LDR    R3, [R3]                     \n"
        "STR    R0, [R3, #0]                 \n"

        "PUSH   {R3, LR}                     \n"
        "MOV    R1, %[mask]                  \n"
        "MSR    BASEPRI, R1                  \n"
        "DSB                                 \n"
        "ISB                                 \n"
        "BL     scheduler                    \n"
        "MOV    R1, #0                       \n"
        "MSR    BASEPRI, R1                  \n"
        "POP    {R3, LR}                     \n"

        "LDR    R3, =current_running_node    \n"
        "LDR    R3, [R3]                     \n"
        "LDR    R0, [R3, #0]                 \n"
        "LDMIA  R0!, {R4-R11}                \n"
        "MSR    PSP, R0                      \n"
        "ISB                                 \n"
        "BX     LR                           \n"

        : : [mask] "i" (KERNEL_INTERRUPT_MASK)
    );
}


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


void task_yield(void)
{
    __asm volatile ("svc 1" ::: "memory");
}

void task_delay(uint32_t ticks)
{
    register uint32_t r0 __asm("r0") = ticks;
    __asm volatile ("svc 2" : : "r"(r0) : "memory");
}

void task_sleep_until(uint32_t *last_wake, uint32_t period)
{
    register uint32_t *r0 __asm("r0") = last_wake;
    register uint32_t  r1 __asm("r1") = period;
    __asm volatile ("svc 3" : : "r"(r0), "r"(r1) : "memory");
}


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


void sem_init(semaphore_t *sem, int32_t initial_count)
{
    sem->count     = initial_count;
    sem->wq_head   = NULL;
    sem->wq_bitmap = 0U;
}

void sem_wait(semaphore_t *sem)
{
    register semaphore_t *r0 __asm("r0") = sem;
    __asm volatile ("svc 4" : : "r"(r0) : "memory");
}

void sem_post(semaphore_t *sem)
{
    register semaphore_t *r0 __asm("r0") = sem;
    __asm volatile ("svc 5" : : "r"(r0) : "memory");
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


void mutex_init(mutex_t *m)
{
    m->locked               = 0U;
    m->owner                = NULL;
    m->highest_waiting_prio = 0U;
    m->wq_head              = NULL;
    m->wq_bitmap            = 0U;
}

void mutex_lock(mutex_t *m)
{
    register mutex_t *r0 __asm("r0") = m;
    __asm volatile ("svc 6" : : "r"(r0) : "memory");
}

void mutex_unlock(mutex_t *m)
{
    register mutex_t *r0 __asm("r0") = m;
    __asm volatile ("svc 7" : : "r"(r0) : "memory");
}


static uint8_t highest_waiter_priority(mutex_t *m)
{
    if (m->wq_bitmap == 0U)
    {
        return 0U;
    }
    return (uint8_t)(31U - __builtin_clz(m->wq_bitmap));
}


static uint8_t update_task_priority(TCB_t *task, uint8_t new_prio)
{
    if (TCB_EFF_PRIO(task) == new_prio)
    {
        return 0U;
    }
    uint8_t state = TCB_STATE(task);
    if (state == TASK_WAKE)
    {
        rq_remove(task);
        TCB_SET_EFF_PRIO(task, new_prio);
        rq_add(task);
    }
    else if (state == TASK_BLOCKED)
    {
        wq_reorder_for_blocked_task(task, new_prio);
    }
    else
    {
        TCB_SET_EFF_PRIO(task, new_prio);
    }
    return 1U;
}


static void propagate_priority(mutex_t *m, uint8_t prio)
{
    mutex_t *cur = m;
    while (cur != NULL)
    {
        TCB_t *owner = cur->owner;
        if (owner == NULL)
        {
            break;
        }
        if (prio > cur->highest_waiting_prio)
        {
            cur->highest_waiting_prio = prio;
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
        cur = (mutex_t *)owner->waiting_on;
    }
}


static void svc_mutex_lock(mutex_t *m)
{
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
            EXIT_CRITICAL();
        }
        else
        {
            EXIT_CRITICAL();
            __asm volatile ("udf #0");
        }
    }
    else
    {
        uint8_t my_prio = TCB_EFF_PRIO(current_running_node);
        current_running_node->waiting_on = m;
        TCB_SET_STATE(current_running_node, TASK_BLOCKED);
        if (my_prio > m->highest_waiting_prio)
        {
            m->highest_waiting_prio = my_prio;
        }
        rq_remove(current_running_node);
        wq_enqueue(&m->wq_head, &m->wq_bitmap, current_running_node);
        propagate_priority(m, my_prio);
        EXIT_CRITICAL();
        request_context_switch();
    }
}


static void svc_mutex_unlock(mutex_t *m)
{
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
            EXIT_CRITICAL();
            __asm volatile ("udf #0");
        }
        next_owner->waiting_on  = NULL;
        TCB_SET_STATE(next_owner, TASK_WAKE);
        rq_add(next_owner);
        m->owner                = next_owner;
        m->locked               = 1U;
        m->highest_waiting_prio = highest_waiter_priority(m);
        if (m->highest_waiting_prio > TCB_EFF_PRIO(next_owner))
        {
            update_task_priority(next_owner, m->highest_waiting_prio);
        }
    }
    else
    {
        m->locked               = 0U;
        m->owner                = NULL;
        m->highest_waiting_prio = 0U;
    }

    if (TCB_EFF_PRIO(current_running_node) != TCB_BASE_PRIO(current_running_node))
    {
        uint8_t  max_inh = TCB_BASE_PRIO(current_running_node);
        uint32_t bm      = (uint32_t)current_running_node->held_mutex_bitmap;
        while (bm != 0U)
        {
            uint8_t  slot = (uint8_t)__builtin_ctz(bm);
            mutex_t *held = held_get(current_running_node, slot);
            if ((held != NULL) && (held->highest_waiting_prio > max_inh))
            {
                max_inh = held->highest_waiting_prio;
            }
            bm &= bm - 1U;
        }
        update_task_priority(current_running_node, max_inh);
    }

    EXIT_CRITICAL();

    if (next_owner != NULL)
    {
        request_context_switch();
    }
}

static void __attribute__((used, noreturn)) kernel_panic(void)
{
    while (1) {}
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile (
        "TST  lr, #4    \n"
        "ITE  EQ        \n"
        "MRSEQ r0, MSP  \n"
        "MRSNE r0, PSP  \n"
        "B    hardfault \n"
    );
}

void hardfault(uint32_t *stack)
{
    (void)stack;
    while (1) {}
}

void MemManage_Handler(void)  { while (1) {} }
void BusFault_Handler(void)   { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }