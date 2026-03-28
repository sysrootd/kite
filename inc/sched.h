#ifndef SCHED_H
#define SCHED_H

#include <stdint.h>
#include "stm32f4xx.h"

// Linker symbols & memory layout
extern uint32_t _estack;                // end of stack from linker script
#define SRAM_END_ADDR   ((uint32_t)&_estack)   // SRAM end address

#define STACK_START     ((uint32_t *)SRAM_END_ADDR)   // initial stack top

// Configuration constants
#define IDLE_TASK_STACK_SIZE   64U      // idle task stack size (in 32-bit words)
#define TICK_HZ                1000U    // system tick frequency (Hz)
#define SYSTEM_CLK             SystemCoreClock // system clock frequency

// Task states
#define TASK_SLEEP              0       // task is sleeping/delayed
#define TASK_WAKE               1       // task is ready to run
#define TASK_BLOCKED            2       // task blocked on semaphore/mutex

// Critical section macros
#define KERNEL_INTERRUPT_PRIORITY      15U
#define KERNEL_INTERRUPT_MASK          (KERNEL_INTERRUPT_PRIORITY << (8U - __NVIC_PRIO_BITS))

void sched_enter_critical(void);
void sched_exit_critical(void);

#define ENTER_CRITICAL()  sched_enter_critical()
#define EXIT_CRITICAL()   sched_exit_critical()

// Type forward declarations
struct TCB;
typedef struct TCB TCB_t;               // task control block

struct semaphore;
typedef struct semaphore semaphore_t;   // semaphore

struct mutex;
typedef struct mutex mutex_t;           // mutex

// Synchronisation structures
struct semaphore {
    int32_t count;      // semaphore count (negative means tasks waiting)
};

struct mutex {
    uint8_t         locked;               // 1 if locked
    TCB_t          *owner;                // current owner
    uint8_t         highest_waiting_prio; // max priority among waiters (0 if none)
};

// Task control block (TCB)
struct TCB {
    uint32_t       *psp_value;          // saved process stack pointer
    uint32_t        block_count;        // timeout count for sleep/blocking
    uint8_t         current_state;      // TASK_SLEEP, TASK_WAKE, TASK_BLOCKED
    void          (*task_handler)(void); // pointer to task function
    
    // priority fields
    uint8_t         base_priority;      // original, unchanging priority
    uint8_t         effective_priority; // current priority (may be boosted)
    
    TCB_t          *next_tcb_node;      // next TCB in circular linked list
    void           *waiting_on;         // pointer to semaphore/mutex being waited on
    mutex_t        *held_mutex;         // mutex currently held (NULL if none)
};

// Global variables (extern)
extern volatile uint32_t global_systick; // system tick counter
extern uint32_t SystemCoreClock;

// Initialisation and startup
void systick_init(void);                       // configure SysTick timer
void init_helper(void);
__attribute__((naked)) void scheduler_init(void); // initialise scheduler (sets MSP, initialises stacks)
void scheduler_start(void); // start multitasking (triggers SVC)

// System tick accessor
uint32_t get_systick_counter(void);             // read current system tick value

// Task creation and management
void create_idle_task(void);                      // create the idle task
void create_task(uint8_t priority, void (*handler)(void), uint32_t stack_words); // create a new task
void task_yield(void);                          // yield CPU to next ready task
void task_delay(uint32_t ticks);                 // delay current task for given ticks
void task_sleep_until(uint32_t *last_wake, uint32_t period); // periodic sleep
void task_wake(void);                            // wake tasks whose delay has expired (called from SysTick)

// Stack and TCB allocation (internal, but exposed for low-level init)
uint32_t *find_stack_area(uint32_t stack_words); // allocate stack from top of memory
TCB_t    *alloc_new_tcb_node(void);              // allocate a new TCB from pool

// Scheduling control
void schedule(void);                             // request a context switch (PendSV)

// Synchronisation primitives
void semaphore_init(semaphore_t *sem, int32_t initial_count);
void semaphore_wait(semaphore_t *sem);
void semaphore_post(semaphore_t *sem);

void mutex_init(mutex_t *m);
void mutex_lock(mutex_t *m);
void mutex_unlock(mutex_t *m);

// Idle task
void idle_task(void);                            // idle loop (WFI)

// Fault handlers (public only for debugging)
void hardfault(uint32_t *stack);                  // C handler for HardFault
void MemManage_Handler(void);                      // memory management fault
void BusFault_Handler(void);                       // bus fault
void UsageFault_Handler(void);                     // usage fault

#endif /* SCHED_H */