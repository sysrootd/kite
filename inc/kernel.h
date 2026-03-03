#include <stdint.h>


#define SRAM_START_ADDR   0x20000000UL
#define SRAM_SIZE_BYTES   (64UL * 1024UL)
#define SRAM_END_ADDR     (SRAM_START_ADDR + SRAM_SIZE_BYTES)

#define STACK_START       ((uint32_t *)SRAM_END_ADDR)
#define IDLE_TASK_STACK_WORDS  	256

#define TICK_HZ					1000U
#define HSI_CLOCK             	16000000U
#define SYSTICK_TIM_CLK       	HSI_CLOCK

#define TASK_SLEEP     	0x00
#define TASK_WAKE    	0xFF

/* PRIMASK manipulation */
#define ENTER_CRITICAL()  __asm volatile("CPSID i" ::: "memory")
#define EXIT_CRITICAL()   __asm volatile("CPSIE i" ::: "memory")

/* ------------------ Forward Declaration ------------------ */

struct TCB;
typedef struct TCB TCB_t;

struct semaphore;
typedef struct semaphore semaphore_t;


/* ---------------------- Structures ------------------------ */

struct semaphore
{
    int32_t count;
    struct TCB_t *wait_list;   //linked list of blocked tasks
};

struct TCB
{
    uint32_t *psp_value;
    uint32_t block_count;
    uint8_t  current_state;
    void (*task_handler)(void);
    uint8_t state;
    TCB_t *next_tcb_node;
};

/* ------------------- System / Scheduler APIs ------------------ */

void systick_init();

__attribute__((naked)) void scheduler_init(void);

__attribute__((naked)) void scheduler_start(void);

/* scheduler API */
void task_delay(uint32_t tick_count);

/* stack allocator*/
uint32_t *find_stack_area(uint32_t stack_size_in_words);

/* tcb allocator */
TCB_t *alloc_new_tcb_node(void);

/* task creation routines */
void create_idle_task(void);

void create_task(uint8_t task_priority, void (*task_handler)(void), uint32_t stack_size);

/* trigger PendSV */
void schedule(void);

/* tick-unblocking */
void task_wake(void);

void idle_task();



