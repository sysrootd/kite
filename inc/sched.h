#ifndef SCHED_H
#define SCHED_H

#include <stdint.h>

// Symbols from linker script - SRAM END
extern uint32_t _estack;                 // End of stack from linker script
#define SRAM_END_ADDR     ((uint32_t)&_estack)  // Address of SRAM end

extern uint32_t SystemCoreClock;

// Stack and task configuration
#define STACK_START       ((uint32_t *)SRAM_END_ADDR)  // Initial stack pointer

#define IDLE_TASK_STACK_SIZE  64U // Stack size for idle task (in 32-bit words)

// Timing and clock configuration
#define TICK_HZ             1000U     // System tick frequency in Hz
#define SYSTEM_CLK          SystemCoreClock // Internal high-speed clock frequency

// Task state definitions
#define TASK_SLEEP          0         // Task is sleeping/delayed
#define TASK_WAKE           1         // Task is ready to run
#define TASK_BLOCKED        2         // Task is blocked on a semaphore/mutex

// Critical section protection
#define ENTER_CRITICAL()  __asm volatile("CPSID i" ::: "memory") // Disable interrupts
#define EXIT_CRITICAL()   __asm volatile("CPSIE i" ::: "memory") // Enable interrupts

// Forward declarations
struct TCB;
typedef struct TCB TCB_t;              // Task Control Block type

struct semaphore;
typedef struct semaphore semaphore_t;  // Semaphore type

struct mutex;
typedef struct mutex mutex_t;          // Mutex type

// Semaphore structure
struct semaphore {
    int32_t  count;                    // Semaphore count
};

// Mutex structures
struct mutex
{
    uint8_t  locked;                    // 1 if locked, 0 if free
    TCB_t   *owner;                     // Task that currently holds the mutex
};

// Task Control Block (TCB) structure
struct TCB {
    uint32_t       *psp_value;          // Saved Process Stack Pointer
    uint32_t        block_count;        // Timeout count for sleep/blocking
    uint8_t         current_state;      // TASK_SLEEP, TASK_WAKE, TASK_BLOCKED
    void          (*task_handler)(void); // Pointer to task function
    TCB_t          *next_tcb_node;      // Next TCB in linked list
    void           *waiting_on;          // Pointer to semaphore/mutex being waited on
};

// System and scheduler APIs
void systick_init(void);                // Initialise SysTick timer

__attribute__((naked)) void scheduler_init(void);   // Initialise scheduler context
__attribute__((naked)) void scheduler_start(void);  // Start multitasking

// Scheduler API - task delay/sleep
void task_delay(uint32_t tick_count);   // Delay current task for given ticks

// Stack allocator - find stack area for new task
uint32_t *find_stack_area(uint32_t stack_size_in_words); // Allocate stack from top

// TCB allocator - allocate new task control block
TCB_t *alloc_new_tcb_node(void);         // Get a new TCB from pool

// Task creation routines
void idle_task_init(void);               // Create the idle task
void task_init(uint8_t task_priority, void (*task_handler)(void), uint32_t stack_size); // Create a task

// Trigger PendSV exception for context switch
void schedule(void);                     // Request a context switch

// Tick-based task unblocking
void task_wake(void);                     // Called from SysTick to wake sleeping tasks

// Idle task handler
void idle_task(void);                     // Idle loop when no tasks are ready

// Semaphore operations
void semaphore_init(semaphore_t *sem, int32_t initial_count); // Initialise semaphore
void semaphore_wait(semaphore_t *sem);                         // Wait (decrement)
void semaphore_post(semaphore_t *sem);                         // Signal (increment)

// Mutex operations
void mutex_init(mutex_t *m);              // Initialise mutex
void mutex_lock(mutex_t *m);               // Lock mutex (block if needed)
void mutex_unlock(mutex_t *m);             // Unlock mutex

#endif // SCHED_H