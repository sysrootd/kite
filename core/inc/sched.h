#ifndef SCHED_H
#define SCHED_H

#include <stdint.h>
#include "mcu.h"
#include "config.h"

/* ============================================================
 *                    GLOBAL SYSTEM SYMBOLS
 * ============================================================ */

/**
 * @brief Global system tick counter (incremented in SysTick ISR)
 */
extern volatile uint32_t global_systick;

/**
 * @brief Core clock frequency (provided by system startup code)
 */
extern uint32_t SystemCoreClock;

/**
 * @brief End of stack symbol defined in linker script
 */
extern uint32_t _estack;

/**
 * @brief SRAM end address derived from linker symbol
 */
#define SRAM_END_ADDR   ((uint32_t)&_estack)

/**
 * @brief Initial stack pointer location
 */
#define STACK_START     ((uint32_t *)SRAM_END_ADDR)

/* ============================================================
 *                    KERNEL CONFIGURATION
 * ============================================================ */

/**
 * @brief Idle task stack size in words
 */
#define IDLE_TASK_STACK_SIZE   64U

/**
 * @brief System tick frequency (Hz)
 */
#define TICK_HZ                1000U

/**
 * @brief Number of priority levels supported (max 32 for bitmap)
 */
#define PRIO_LEVELS            32U

/* ============================================================
 *                    TASK STATES
 * ============================================================ */

#define TASK_SLEEP    0U   /**< Task is sleeping (timed delay) */
#define TASK_WAKE     1U   /**< Task is ready to run */
#define TASK_BLOCKED  2U   /**< Task is blocked on sync object */

/* ============================================================
 *                INTERRUPT PRIORITY CONFIGURATION
 * ============================================================ */

/**
 * @brief Kernel interrupt priority (lowest priority)
 */
#define KERNEL_INTERRUPT_PRIORITY  15U

/**
 * @brief Mask value used to disable interrupts up to kernel level
 */
#define KERNEL_INTERRUPT_MASK \
    (KERNEL_INTERRUPT_PRIORITY << (8U - __NVIC_PRIO_BITS))

/* ============================================================
 *                STACK PROTECTION CONFIGURATION
 * ============================================================ */

/**
 * @brief MPU region index used for stack guard
 */
#define STACK_GUARD_MPU_REGION  1U

/**
 * @brief Stack guard size (in words)
 */
#define STACK_GUARD_SIZE_WORDS  8U

/* ============================================================
 *                SVC (SUPERVISOR CALL) IDS
 * ============================================================ */

#define SVC_START_FIRST_TASK  0U
#define SVC_YIELD             1U
#define SVC_DELAY             2U
#define SVC_SLEEP_UNTIL       3U
#define SVC_SEM_WAIT          4U
#define SVC_SEM_POST          5U
#define SVC_MUTEX_LOCK        6U
#define SVC_MUTEX_UNLOCK      7U

/* ============================================================
 *                STATIC CONFIG VALIDATION
 * ============================================================ */

/**
 * @brief Compile-time assertion macro
 */
#define KITE_STATIC_ASSERT(expr, msg)  _Static_assert(expr, msg)

/* Validate kernel configuration at compile-time */
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
    "PRIO_LEVELS must be <= 32 (fits in 32-bit bitmap)");

/* ============================================================
 *                CRITICAL SECTION CONTROL
 * ============================================================ */

/**
 * @brief Enter critical section (disable interrupts)
 */
void sched_enter_critical(void);

/**
 * @brief Exit critical section (restore interrupts)
 */
void sched_exit_critical(void);

/**
 * @brief Macro wrappers for critical section handling
 */
#define ENTER_CRITICAL()  sched_enter_critical()
#define EXIT_CRITICAL()   sched_exit_critical()

/* ============================================================
 *                FORWARD DECLARATIONS
 * ============================================================ */

struct TCB;
typedef struct TCB TCB_t;

struct semaphore;
typedef struct semaphore semaphore_t;

struct mutex;
typedef struct mutex mutex_t;

/* ============================================================
 *                SYNCHRONIZATION PRIMITIVES
 * ============================================================ */

/**
 * @brief Counting semaphore structure
 */
struct semaphore {
    int32_t  count;        /**< Current semaphore count */
    TCB_t   *wq_head;      /**< Head of waiting queue */
    uint32_t wq_bitmap;    /**< Priority bitmap for waiting tasks */
};

/**
 * @brief Mutex structure with priority inheritance support
 */
struct mutex {
    uint8_t  locked;               /**< Lock state */
    TCB_t   *owner;                /**< Current owner */
    uint8_t  highest_waiting_prio; /**< Highest waiting priority */
    TCB_t   *wq_head;              /**< Waiting queue head */
    uint32_t wq_bitmap;            /**< Waiting tasks bitmap */
};

/* ============================================================
 *                TASK CONTROL BLOCK (TCB)
 * ============================================================ */

/**
 * @brief Task Control Block (TCB)
 *
 * Core structure representing a thread in the RTOS.
 */
struct TCB {

    uint32_t  *psp_value;        /**< Process Stack Pointer */
    uint32_t   block_count;      /**< Delay/block timeout */

    /**
     * Packed field:
     * [1:0]   -> state
     * [6:2]   -> base priority
     * [11:7]  -> effective priority (after inheritance)
     */
    uint16_t   state_prio;

    uint16_t   held_mutex_bitmap; /**< Bitmask of owned mutexes */
    void      *waiting_on;        /**< Object currently waiting on */

    uint32_t  *stack_guard_base;  /**< Base address for stack guard */

    /* Linked list pointers */
    TCB_t     *next_tcb_node;     /**< Global TCB list */
    TCB_t     *rq_next;           /**< Run queue next */
    TCB_t     *rq_prev;           /**< Run queue previous */
    TCB_t     *sl_next;           /**< Sleep list next */
    TCB_t     *wq_next;           /**< Wait queue next */

#ifdef SCHED_DEBUG
    const char *name;             /**< Debug name */
#endif
};

/* ============================================================
 *                TCB FIELD ACCESS MACROS
 * ============================================================ */

/* Extractors */
#define TCB_STATE(t)           ((uint8_t)((t)->state_prio & 0x03U))
#define TCB_BASE_PRIO(t)       ((uint8_t)(((t)->state_prio >> 2U) & 0x1FU))
#define TCB_EFF_PRIO(t)        ((uint8_t)(((t)->state_prio >> 7U) & 0x1FU))

/* Mutators */
#define TCB_SET_STATE(t,s)     ((t)->state_prio = \
    ((t)->state_prio & ~0x0003U) | ((s) & 0x03U))

#define TCB_SET_BASE_PRIO(t,p) ((t)->state_prio = \
    ((t)->state_prio & ~0x007CU) | (((p) & 0x1FU) << 2U))

#define TCB_SET_EFF_PRIO(t,p)  ((t)->state_prio = \
    ((t)->state_prio & ~0x0F80U) | (((p) & 0x1FU) << 7U))

/**
 * @brief Initialize combined state/priority field
 */
#define TCB_INIT_STATE_PRIO(t, state, base, eff) \
    ((t)->state_prio = \
        ((state) & 0x03U) | \
        (((base) & 0x1FU) << 2U) | \
        (((eff)  & 0x1FU) << 7U))

/* ============================================================
 *                KERNEL API
 * ============================================================ */

/**
 * @brief Start the scheduler
 */
void kite_start(void);

/**
 * @brief Get current system tick count
 */
uint32_t get_systick_counter(void);

/**
 * @brief Create a new task
 *
 * @param priority Task priority
 * @param handler  Entry function
 * @param stack_words Stack size in words
 * @param name Debug name (optional)
 * @return status (0 = fail, 1 = success)
 */
uint8_t create_task(uint8_t priority, void (*handler)(void),
                    uint32_t stack_words, const char *name);

/**
 * @brief Yield CPU voluntarily
 */
void task_yield(void);

/**
 * @brief Delay task for specified ticks
 */
void task_delay(uint32_t ticks);

/**
 * @brief Periodic wake mechanism
 */
void task_sleep_until(uint32_t *last_wake, uint32_t period);

/**
 * @brief Set scheduler time slice
 */
void set_time_slice_ticks(uint32_t ticks);

/* ============================================================
 *                SYNCHRONIZATION API
 * ============================================================ */

void semaphore_init(semaphore_t *sem, int32_t initial_count);
void semaphore_wait(semaphore_t *sem);
void semaphore_post(semaphore_t *sem);

void mutex_init(mutex_t *m);
void mutex_lock(mutex_t *m);
void mutex_unlock(mutex_t *m);

/* ============================================================
 *                FAULT HANDLERS
 * ============================================================ */

/**
 * @brief HardFault handler with stack frame capture
 */
void hardfault(uint32_t *stack);

void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);

#endif