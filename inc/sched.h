/**
 * @file sched.h
 * @brief Simple RTOS scheduler for STM32F4
 *
 * This header defines the public API for the cooperative/priority-based scheduler.
 * It includes task control block (TCB) structures, semaphores, mutexes, and
 * scheduler functions.
 */

#ifndef SCHED_H
#define SCHED_H

#include <stdint.h>
#include "stm32f4xx.h"

/*============================================================================
 *                      Linker Symbols & Memory Layout
 *============================================================================*/
extern uint32_t _estack;                /**< End of stack from linker script */
#define SRAM_END_ADDR   ((uint32_t)&_estack)   /**< SRAM end address */

#define STACK_START     ((uint32_t *)SRAM_END_ADDR)   /**< Initial stack top */

/*============================================================================
 *                      Configuration Constants
 *============================================================================*/
#define IDLE_TASK_STACK_SIZE   64U      /**< Idle task stack size (in 32-bit words) */
#define TICK_HZ                1000U    /**< System tick frequency (Hz) */
#define SYSTEM_CLK             SystemCoreClock /**< System clock frequency */

/*============================================================================
 *                      Task States
 *============================================================================*/
#define TASK_SLEEP              0       /**< Task is sleeping/delayed */
#define TASK_WAKE               1       /**< Task is ready to run */
#define TASK_BLOCKED            2       /**< Task blocked on semaphore/mutex */

/*============================================================================
 *                      Critical Section Macros
 *============================================================================*/
/** Disable interrupts (enter critical section) */
#define ENTER_CRITICAL()        __disable_irq()
/** Enable interrupts (exit critical section) */
#define EXIT_CRITICAL()         __enable_irq()

/*============================================================================
 *                      Type Forward Declarations
 *============================================================================*/
struct TCB;
typedef struct TCB TCB_t;               /**< Task Control Block */

struct semaphore;
typedef struct semaphore semaphore_t;   /**< Semaphore */

struct mutex;
typedef struct mutex mutex_t;           /**< Mutex */

/*============================================================================
 *                      Synchronisation Structures
 *============================================================================*/
/**
 * @brief Semaphore structure
 */
struct semaphore {
    int32_t count;      /**< Semaphore count (negative means tasks waiting) */
};

/**
 * @brief Mutex structure (supports priority inheritance? Not implemented here)
 */
struct mutex {
    uint8_t         locked;               // 1 if locked
    TCB_t          *owner;                 // current owner
    uint8_t         highest_waiting_prio;  // max priority among waiters (0 if none)
};

/*============================================================================
 *                      Task Control Block (TCB)
 *============================================================================*/
/**
 * @brief Task Control Block – represents a single thread/task
 */
struct TCB {
    uint32_t       *psp_value;          // Saved Process Stack Pointer
    uint32_t        block_count;        // Timeout count for sleep/blocking
    uint8_t         current_state;      // TASK_SLEEP, TASK_WAKE, TASK_BLOCKED
    void          (*task_handler)(void); // Pointer to task function
    
    // Priority fields (old "priority" replaced)
    uint8_t         base_priority;      // Original, unchanging priority
    uint8_t         effective_priority; // Current priority (may be boosted)
    
    TCB_t          *next_tcb_node;      // Next TCB in circular linked list
    void           *waiting_on;         // Pointer to semaphore/mutex being waited on
    mutex_t        *held_mutex;         // Mutex currently held (NULL if none)
};

/*============================================================================
 *                      Global Variables (extern)
 *============================================================================*/
/**
 * @brief System tick counter incremented every SysTick interrupt.
 *        Use get_systick_counter() to read.
 */
extern volatile uint32_t global_systick;
extern uint32_t SystemCoreClock;

/*============================================================================
 *                      Public Function Prototypes
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Initialisation and startup
 *----------------------------------------------------------------------------*/
void systick_init(void);                       /**< Configure SysTick timer */
__attribute__((naked)) void scheduler_init(void); /**< Initialise scheduler (sets MSP, initialises stacks) */
__attribute__((naked)) void scheduler_start(void); /**< Start multitasking (triggers SVC) */

/*----------------------------------------------------------------------------
 * System tick accessor
 *----------------------------------------------------------------------------*/
uint32_t get_systick_counter(void);             /**< Read current system tick value */

/*----------------------------------------------------------------------------
 * Task creation and management
 *----------------------------------------------------------------------------*/
void idle_task_init(void);                      /**< Create the idle task */
void task_init(uint8_t priority, void (*handler)(void), uint32_t stack_words); /**< Create a new task */
void task_yield(void);                          /**< Yield CPU to next ready task */
void task_delay(uint32_t ticks);                 /**< Delay current task for given ticks */
void task_sleep_until(uint32_t *last_wake, uint32_t period); /**< Periodic sleep */
void task_wake(void);                            /**< Wake tasks whose delay has expired (called from SysTick) */

/*----------------------------------------------------------------------------
 * Stack and TCB allocation (internal, but exposed for low-level init)
 *----------------------------------------------------------------------------*/
uint32_t *find_stack_area(uint32_t stack_words); /**< Allocate stack from top of memory */
TCB_t    *alloc_new_tcb_node(void);              /**< Allocate a new TCB from pool */

/*----------------------------------------------------------------------------
 * Scheduling control
 *----------------------------------------------------------------------------*/
void schedule(void);                             /**< Request a context switch (PendSV) */

/*----------------------------------------------------------------------------
 * Synchronisation primitives
 *----------------------------------------------------------------------------*/
void semaphore_init(semaphore_t *sem, int32_t initial_count);
void semaphore_wait(semaphore_t *sem);
void semaphore_post(semaphore_t *sem);

void mutex_init(mutex_t *m);
void mutex_lock(mutex_t *m);
void mutex_unlock(mutex_t *m);

/*----------------------------------------------------------------------------
 * Idle task
 *----------------------------------------------------------------------------*/
void idle_task(void);                            /**< Idle loop (WFI) */

/*----------------------------------------------------------------------------
 * Fault handlers (public only for debugging)
 *----------------------------------------------------------------------------*/
void hardfault(uint32_t *stack);                  /**< C handler for HardFault */
void MemManage_Handler(void);                      /**< Memory management fault */
void BusFault_Handler(void);                       /**< Bus fault */
void UsageFault_Handler(void);                     /**< Usage fault */

#endif /* SCHED_H */