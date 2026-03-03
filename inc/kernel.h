#ifndef KERNEL_H
#define KERNEL_H

#include <stdint.h>

// Symbols from linker script - SRAM configuration
extern uint32_t _sram_start;   // SRAM start address (0x20000000)
extern uint32_t _sram_size;    // Total SRAM size (64KB)
extern uint32_t _estack;       // End of SRAM (top of stack)

// Derive SRAM addresses from linker symbols
#define SRAM_START_ADDR   ((uint32_t)&_sram_start)
#define SRAM_SIZE_BYTES   ((uint32_t)&_sram_size)
#define SRAM_END_ADDR     ((uint32_t)&_estack)

// Stack and task configuration
#define STACK_START       ((uint32_t *)SRAM_END_ADDR)
#define IDLE_TASK_STACK_WORDS  256

// Timing and clock configuration
#define TICK_HZ           1000U
#define HSI_CLOCK         16000000U
#define SYSTICK_TIM_CLK   HSI_CLOCK

// Task state definitions
#define TASK_SLEEP        0x00
#define TASK_WAKE         0xFF

// Critical section protection
#define ENTER_CRITICAL()  __asm volatile("CPSID i" ::: "memory")
#define EXIT_CRITICAL()   __asm volatile("CPSIE i" ::: "memory")

// Forward declarations
struct TCB;
typedef struct TCB TCB_t;

struct semaphore;
typedef struct semaphore semaphore_t;

// Semaphore structure
struct semaphore {
    int32_t         count;
    struct TCB_t  *wait_list;  // Linked list of blocked tasks
};

// Task Control Block (TCB) structure
struct TCB {
    uint32_t       *psp_value;        // Process stack pointer value
    uint32_t        block_count;      // Block/sleep count
    uint8_t         current_state;    // Current task state
    void          (*task_handler)(void); // Task function pointer
    uint8_t         state;            // Task state flag
    TCB_t          *next_tcb_node;    // Next TCB in linked list
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

#endif  // KERNEL_H

