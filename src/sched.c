#include <stddef.h>
#include <stdint.h>

#include "mcu.h"
#include "sched.h"
#include "mem.h"
#include "timer.h"

volatile uint32_t global_systick = 0;

TCB_t *current_running_node = NULL;
TCB_t *head_node             = NULL;
TCB_t *link_node             = NULL;

static uint32_t *new_task_psp  = STACK_START;
static uint32_t *next_task_psp = STACK_START;
static uint32_t  msp_start;

static volatile uint32_t tick_count       = 0;
static          uint32_t time_slice_ticks = SCHED_TIME_SLICE;


static TCB_t   *ready_queue[PRIO_LEVELS];
static uint32_t ready_bitmap = 0U;

static TCB_t *sleep_list_head = NULL;

__attribute__((naked, used)) void scheduler_init(void);

static void __attribute__((used)) scheduler_start(void);
static void    systick_init(void);
void           SysTick_Handler(void);
static uint8_t task_wake(void);
static void    idle_task(void);

static void core_faults_init(void);
static void __attribute__((used)) init_helper(void);
static void update_task_runtime_on_switch(void);
static void set_current_task_start_time(void);

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

static inline void rq_add(TCB_t *t)
{
    uint8_t p = t->effective_priority;

    if (ready_queue[p] == NULL)
    {
        t->rq_next     = t;
        t->rq_prev     = t;
        ready_queue[p] = t;
        ready_bitmap  |= (1U << p);
    }
    else
    {
        TCB_t *head    = ready_queue[p];
        TCB_t *tail    = head->rq_prev;
        tail->rq_next  = t;
        t->rq_prev     = tail;
        t->rq_next     = head;
        head->rq_prev  = t;
        
    }
}

static inline void rq_remove(TCB_t *t)
{
    uint8_t p = t->effective_priority;

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
    uint8_t p = (uint8_t)(31U - __builtin_clz(ready_bitmap));
    return ready_queue[p];
}

static void sl_insert(TCB_t *t)
{
    t->current_state = TASK_SLEEP;
    t->sl_next       = NULL;

    if (sleep_list_head == NULL ||
        (int32_t)(t->block_count - sleep_list_head->block_count) <= 0)
    {
        t->sl_next       = sleep_list_head;
        sleep_list_head  = t;
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
    uint8_t p    = t->effective_priority;
    *wq_bitmap  |= (1U << p);
    t->wq_next   = NULL;

    if ((*wq_head == NULL) || (p > (*wq_head)->effective_priority))
    {
        t->wq_next = *wq_head;
        *wq_head   = t;
        return;
    }

    TCB_t *curr = *wq_head;
    while ((curr->wq_next != NULL) &&
           (curr->wq_next->effective_priority >= p))
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

    uint8_t p  = t->effective_priority;
    *wq_head   = t->wq_next;
    t->wq_next = NULL;

    if ((*wq_head == NULL) || ((*wq_head)->effective_priority != p))
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
        task->effective_priority = new_prio;
        return;
    }

    uint8_t old_p = task->effective_priority;

    
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
            if (curr->effective_priority == old_p)
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

    
    task->effective_priority = new_prio;
    m->wq_bitmap |= (1U << new_prio);

    if ((m->wq_head == NULL) || (new_prio > m->wq_head->effective_priority))
    {
        task->wq_next = m->wq_head;
        m->wq_head    = task;
    }
    else
    {
        TCB_t *curr = m->wq_head;
        while ((curr->wq_next != NULL) &&
               (curr->wq_next->effective_priority >= new_prio))
        {
            curr = curr->wq_next;
        }
        task->wq_next = curr->wq_next;
        curr->wq_next = task;
    }

    
    m->highest_waiting_prio = highest_waiter_priority(m);
}


static volatile uint32_t critical_nesting = 0;
static          uint32_t saved_basepri    = 0;

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
        : "r" (KERNEL_INTERRUPT_MASK)
        : "memory"
    );

    if (critical_nesting == 0U)
    {
        saved_basepri = current_basepri;
    }

    critical_nesting++;
}

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

static inline void request_context_switch(void)
{
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}


void kite_start(void)
{
    scheduler_init();
}

static void core_faults_init(void)
{
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk  |
                  SCB_SHCSR_USGFAULTENA_Msk;
}

static void find_high_priority_task(void)
{
    current_running_node = rq_highest();
}

static void __attribute__((used)) init_helper(void)
{
    msp_start = (uint32_t)next_task_psp;
    
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
    set_current_task_start_time();
    __asm volatile("svc 0");
}


static void __attribute__((used)) update_task_runtime_on_switch(void)
{
    if (current_running_node == NULL)
    {
        return;
    }

    uint32_t now = timer_get_us();
    if (current_running_node->last_run_start_us != 0U)
    {
        current_running_node->runtime_us +=
            now - current_running_node->last_run_start_us;
    }
}

static void __attribute__((used)) set_current_task_start_time(void)
{
    if (current_running_node == NULL)
    {
        return;
    }

    current_running_node->last_run_start_us = timer_get_us();
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

static uint8_t task_wake(void)
{
    uint8_t woke = 0U;

    while ((sleep_list_head != NULL) &&
           ((int32_t)(global_systick - sleep_list_head->block_count) >= 0))
    {
        TCB_t *t        = sleep_list_head;
        sleep_list_head = t->sl_next;
        t->sl_next      = NULL;
        t->current_state = TASK_WAKE;
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
        "MRS R0, PSP                      \n"
        "STMDB R0!, {R4-R11}              \n"
        "PUSH {R3, LR}                    \n"
        "BL __set_psp                     \n"
        "BL update_task_runtime_on_switch \n"
        "BL fair_priority_sched           \n"   
        "BL set_current_task_start_time   \n"
        "BL __get_psp                     \n"
        "POP {R3, LR}                     \n"
        "LDMIA R0!, {R4-R11}              \n"
        "MSR PSP, R0                      \n"
        "BX LR                            \n"
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

static void __attribute__((used)) fair_priority_sched(void)
{
    if ((current_running_node != NULL) &&
        (current_running_node->current_state == TASK_WAKE))
    {
        uint8_t p      = current_running_node->effective_priority;
        ready_queue[p] = current_running_node->rq_next;
    }

    current_running_node = rq_highest();   
}


static void __attribute__((used)) task_stack_init(void)
{
    TCB_t *iter = head_node;

    while (iter != NULL)
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

        iter->psp_value        = stack;
        iter->runtime_us       = 0U;
        iter->last_run_start_us = 0U;

        iter = iter->next_tcb_node;          
    }
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
    if (node != NULL)
    {
        node->next_tcb_node = NULL;
        node->rq_next       = NULL;
        node->rq_prev       = NULL;
        node->sl_next       = NULL;
        node->wq_next       = NULL;
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
    idle->name               = "idle";
    idle->runtime_us         = 0U;
    idle->last_run_start_us  = 0U;
    idle->waiting_on         = NULL;

    for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
    {
        idle->held_mutex[i] = NULL;
    }

    link_node = idle;
    head_node = idle;

    rq_add(idle);   
}

uint8_t create_task(uint8_t priority, void (*handler)(void),
                    uint32_t stack_words, const char *name)
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
    tcb->name               = name;
    tcb->runtime_us         = 0U;
    tcb->last_run_start_us  = 0U;
    tcb->waiting_on         = NULL;

    for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
    {
        tcb->held_mutex[i] = NULL;
    }

    
    link_node->next_tcb_node = tcb;
    link_node                = tcb;

    
    rq_add(tcb);

    return 1U;
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


void semaphore_init(semaphore_t *sem, int32_t initial_count)
{
    sem->count     = initial_count;
    sem->wq_head   = NULL;
    sem->wq_bitmap = 0U;
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
            best->waiting_on    = NULL;
            best->current_state = TASK_WAKE;
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
    __asm volatile("svc 6" : : "r"(r0) : "memory");
}

void mutex_unlock(mutex_t *m)
{
    register mutex_t *r0 __asm("r0") = m;
    __asm volatile("svc 7" : : "r"(r0) : "memory");
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
    if (task->effective_priority == new_prio)
    {
        return 0U;
    }

    if (task->current_state == TASK_WAKE)
    {
        rq_remove(task);
        task->effective_priority = new_prio;
        rq_add(task);
    }
    else if (task->current_state == TASK_BLOCKED)
    {
        wq_reorder_for_blocked_task(task, new_prio);
    }
    else 
    {
        task->effective_priority = new_prio;
    }

    return 1U;
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
            m->highest_waiting_prio                = 0U;
            current_running_node->held_mutex[slot] = m;
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

static void svc_mutex_unlock(mutex_t *m)
{
    uint8_t need_switch = 0U;

    if ((m == NULL) || (m->owner != current_running_node))
    {
        return;
    }

    ENTER_CRITICAL();

    
    for (uint8_t i = 0U; i < HELD_MUTEX_MAX; i++)
    {
        if (current_running_node->held_mutex[i] == m)
        {
            current_running_node->held_mutex[i] = NULL;
            break;
        }
    }

    TCB_t *next_owner = wq_dequeue_highest(&m->wq_head, &m->wq_bitmap);

    if (next_owner != NULL)
    {
        
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

        next_owner->waiting_on    = NULL;
        next_owner->current_state = TASK_WAKE;
        rq_add(next_owner);

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