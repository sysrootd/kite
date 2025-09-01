#ifndef RTOS_H
#define RTOS_H

#include <stdint.h>

#define MAX_TASKS   8
#define STACK_SIZE  256   // per task (words)

typedef void (*task_func_t)(void *);

typedef enum {
    TASK_READY,
    TASK_BLOCKED,
} task_state_t;

typedef struct {
    uint32_t *stack_ptr;          // current stack pointer
    uint32_t stack_mem[STACK_SIZE];
    uint32_t delay_ticks;         // sleep counter
    uint8_t priority;             // priority (higher = better)
    task_state_t state;
    uint8_t active;
} TCB;

void rtos_init(void);
void rtos_create_task(task_func_t func, void *arg, uint8_t priority);
void rtos_start(void);
void rtos_delay(uint32_t ms);

#endif
