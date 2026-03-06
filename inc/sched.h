#ifndef SCHED_H
#define SCHED_H

#include <stdint.h>

// Symbols from linker script - SRAM END
extern uint32_t _estack;
#define SRAM_END_ADDR     ((uint32_t)&_estack)

// Stack and task configuration
#define STACK_START       ((uint32_t *)SRAM_END_ADDR)

#define IDLE_TASK_STACK_WORDS  256U

// Timing and clock configuration
#define TICK_HZ             1000U
#define HSI_CLK             16000000U
#define SYSTICK_TIM_CLK     HSI_CLK

// Task state definitions
#define TASK_SLEEP          0
#define TASK_WAKE           1
#define TASK_BLOCKED        2

// Stack frame constants for ARM Cortex-M4
#define STACK_FRAME_XPSR        0x01000000UL  // xPSR with Thumb bit set
#define STACK_FRAME_LR          0xFFFFFFFDUL  // Return to Thread mode using PSP
#define NUM_GP_REGS             13            // Number of GP registers (R0-R12)
#define STACK_ALIGN_8BYTE       0x7U          // 8-byte alignment mask

// Critical section protection
#define ENTER_CRITICAL()  __asm volatile("CPSID i" ::: "memory")
#define EXIT_CRITICAL()   __asm volatile("CPSIE i" ::: "memory")

// Forward declarations
struct TCB;
typedef struct TCB TCB_t;

struct semaphore;
typedef struct semaphore semaphore_t;

struct mutex;
typedef struct mutex mutex_t;

// Semaphore structure
struct semaphore {
    int32_t  count;
};

// Mutex structures
struct mutex
{
    uint8_t  locked;
    TCB_t   *owner;
};

// Task Control Block (TCB) structure
struct TCB {
    uint32_t       *psp_value;         
    uint32_t        block_count;       
    uint8_t         current_state;     
    void          (*task_handler)(void);
    TCB_t          *next_tcb_node;
    void           *waiting_on;     
};

// System and scheduler APIs
void systick_init(void);

__attribute__((naked)) void scheduler_init(void);

__attribute__((naked)) void scheduler_start(void);

// Scheduler API - task delay/sleep
void task_delay(uint32_t tick_count);

// Stack allocator - find stack area for new task
uint32_t *find_stack_area(uint32_t stack_size_in_words);

// TCB allocator - allocate new task control block
TCB_t *alloc_new_tcb_node(void);

// Task creation routines
void create_idle_task(void);

void create_task(uint8_t task_priority, void (*task_handler)(void), uint32_t stack_size);

// Trigger PendSV exception for context switch
void schedule(void);

// Tick-based task unblocking
void task_wake(void);

// Idle task handler
void idle_task(void);

void semaphore_init(semaphore_t *sem, int32_t initial_count);

void semaphore_wait(semaphore_t *sem);

void semaphore_post(semaphore_t *sem);

void mutex_init(mutex_t *m);

void mutex_lock(mutex_t *m);

void mutex_unlock(mutex_t *m);

#endif

