#ifndef SCHED_H
#define SCHED_H

#include <stdint.h>
#include "mcu.h"

extern volatile uint32_t global_systick;
extern uint32_t SystemCoreClock;

extern uint32_t _estack;
#define SRAM_END_ADDR   ((uint32_t)&_estack)
#define STACK_START     ((uint32_t *)SRAM_END_ADDR)

#define IDLE_TASK_STACK_SIZE   64U
#define TICK_HZ                1000U

#define MAX_TASKS         16U  /* Max concurrent tasks for internal pool */
#define SCHED_TIME_SLICE  5U
#define HELD_MUTEX_MAX    4U
#define PSP_VALUE_OFFSET  0

#define TASK_SLEEP    0
#define TASK_WAKE     1
#define TASK_BLOCKED  2

#define KERNEL_INTERRUPT_PRIORITY  15U
#define KERNEL_INTERRUPT_MASK      (KERNEL_INTERRUPT_PRIORITY << (8U - __NVIC_PRIO_BITS))

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
    "HELD_MUTEX_MAX too large, check TCB size vs stack budget");
KITE_STATIC_ASSERT(TICK_HZ >= 1U && TICK_HZ <= 10000U,
    "TICK_HZ out of sane range (1 - 10000)");
KITE_STATIC_ASSERT(SCHED_TIME_SLICE >= 1U,
    "SCHED_TIME_SLICE must be at least 1 tick");
KITE_STATIC_ASSERT(IDLE_TASK_STACK_SIZE >= 32U,
    "IDLE_TASK_STACK_SIZE too small, minimum 32 words");

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
    int32_t count;
};

struct mutex {
    uint8_t  locked;
    TCB_t   *owner;
    uint8_t  highest_waiting_prio;
};

struct TCB {
    uint32_t  *psp_value;
    uint32_t   block_count;
    uint8_t    current_state;
    void     (*task_handler)(void);

    uint8_t    base_priority;
    uint8_t    effective_priority;

    const char *name;
    uint32_t    runtime_us;
    uint32_t    last_run_start_us;

    TCB_t     *next_tcb_node;
    void      *waiting_on;
    mutex_t   *held_mutex[HELD_MUTEX_MAX];
};

void kite_start(void);

uint32_t get_systick_counter(void);

uint8_t  create_task(uint8_t priority, void (*handler)(void), uint32_t stack_words, const char *name);
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