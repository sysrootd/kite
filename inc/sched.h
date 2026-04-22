#ifndef SCHED_H
#define SCHED_H

#include <stdint.h>
#include "mcu.h"
#include "config.h"

extern volatile uint32_t global_systick;
extern uint32_t SystemCoreClock;

extern uint32_t _estack;
#define SRAM_END_ADDR   ((uint32_t)&_estack)
#define STACK_START     ((uint32_t *)SRAM_END_ADDR)

#define IDLE_TASK_STACK_SIZE   64U
#define TICK_HZ                1000U

#define PRIO_LEVELS            32U

#define TASK_SLEEP    0U
#define TASK_WAKE     1U
#define TASK_BLOCKED  2U

#define KERNEL_INTERRUPT_PRIORITY  15U
#define KERNEL_INTERRUPT_MASK      (KERNEL_INTERRUPT_PRIORITY << (8U - __NVIC_PRIO_BITS))

#define STACK_GUARD_MPU_REGION  1U
#define STACK_GUARD_SIZE_WORDS  8U

#define SVC_START_FIRST_TASK  0U
#define SVC_YIELD             1U
#define SVC_DELAY             2U
#define SVC_SLEEP_UNTIL       3U
#define SVC_SEM_WAIT          4U
#define SVC_SEM_POST          5U
#define SVC_MUTEX_LOCK        6U
#define SVC_MUTEX_UNLOCK      7U

#define KITE_STATIC_ASSERT(expr, msg)  _Static_assert(expr, msg)

KITE_STATIC_ASSERT(HELD_MUTEX_MAX >= 1U,
    "HELD_MUTEX_MAX must be at least 1");
KITE_STATIC_ASSERT(HELD_MUTEX_MAX <= 16U,
    "HELD_MUTEX_MAX must be <= 16 (bitmap is uint16_t)");
KITE_STATIC_ASSERT(TICK_HZ >= 1U && TICK_HZ <= 10000U,
    "TICK_HZ out of sane range (1 - 10000)");
KITE_STATIC_ASSERT(SCHED_TIME_SLICE >= 1U,
    "SCHED_TIME_SLICE must be at least 1 tick");
KITE_STATIC_ASSERT(IDLE_TASK_STACK_SIZE >= 32U,
    "IDLE_TASK_STACK_SIZE too small, minimum 32 words");
KITE_STATIC_ASSERT(PRIO_LEVELS <= 32U,
    "PRIO_LEVELS must be <= 32 to fit in a single 32-bit bitmap word");

void sched_enter_critical(void);
void sched_exit_critical(void);

#define ENTER_CRITICAL()  sched_enter_critical()
#define EXIT_CRITICAL()   sched_exit_critical()

struct TCB;
typedef struct TCB TCB_t;

struct semaphore;
typedef struct semaphore semaphore_t;

struct mutex;
typedef struct mutex mutex_t;

struct semaphore {
    int32_t  count;
    TCB_t   *wq_head;
    uint32_t wq_bitmap;
};

struct mutex {
    uint8_t  locked;
    TCB_t   *owner;
    uint8_t  highest_waiting_prio;
    TCB_t   *wq_head;
    uint32_t wq_bitmap;
};

struct TCB {
    
    uint32_t  *psp_value;       
    uint32_t   block_count;     
    uint16_t   state_prio;
    uint16_t   held_mutex_bitmap;
    void      *waiting_on;
    uint32_t  *stack_guard_base ; 

    TCB_t     *next_tcb_node;   
    TCB_t     *rq_next;         
    TCB_t     *rq_prev;
    TCB_t     *sl_next;         
    TCB_t     *wq_next;         

#ifdef SCHED_DEBUG
    const char *name;           
#endif
};

#define TCB_STATE(t)           ((uint8_t)( (t)->state_prio        & 0x03U))
#define TCB_BASE_PRIO(t)       ((uint8_t)(((t)->state_prio >> 2U) & 0x1FU))
#define TCB_EFF_PRIO(t)        ((uint8_t)(((t)->state_prio >> 7U) & 0x1FU))

#define TCB_SET_STATE(t,s)     ((t)->state_prio = (uint16_t)(       \
                                    ((t)->state_prio & ~0x0003U) |  \
                                    ((s) & 0x03U)))

#define TCB_SET_BASE_PRIO(t,p) ((t)->state_prio = (uint16_t)(       \
                                    ((t)->state_prio & ~0x007CU) |  \
                                    (((p) & 0x1FU) << 2U)))

#define TCB_SET_EFF_PRIO(t,p)  ((t)->state_prio = (uint16_t)(       \
                                    ((t)->state_prio & ~0x0F80U) |  \
                                    (((p) & 0x1FU) << 7U)))

#define TCB_INIT_STATE_PRIO(t, state, base, eff)                     \
    ((t)->state_prio = (uint16_t)(                                   \
        ((state) & 0x03U)         |                                  \
        (((base) & 0x1FU) << 2U) |                                   \
        (((eff)  & 0x1FU) << 7U)))

void kite_start(void);

uint32_t get_systick_counter(void);

uint8_t  create_task(uint8_t priority, void (*handler)(void),
                     uint32_t stack_words, const char *name);
void     task_yield(void);
void     task_delay(uint32_t ticks);
void     task_sleep_until(uint32_t *last_wake, uint32_t period);
void     set_time_slice_ticks(uint32_t ticks);

void semaphore_init(semaphore_t *sem, int32_t initial_count);
void semaphore_wait(semaphore_t *sem);
void semaphore_post(semaphore_t *sem);

void mutex_init(mutex_t *m);
void mutex_lock(mutex_t *m);
void mutex_unlock(mutex_t *m);

void hardfault(uint32_t *stack);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);

#endif 