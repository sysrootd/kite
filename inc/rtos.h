#ifndef RTOS_H
#define RTOS_H

#include <stdint.h>

#define MAX_TASKS 8
#define STACK_SIZE 128

typedef void (*task_func_t)(void *);

typedef enum {
    TASK_READY,
    TASK_BLOCKED,
    TASK_RUNNING
} task_state_t;

typedef struct {
    uint32_t *stack_ptr;
    uint32_t stack_mem[STACK_SIZE];
    uint8_t priority;
    task_state_t state;
    uint32_t delay_ticks;
    uint8_t active;
} TCB;

void rtos_init(void);
void rtos_create_task(task_func_t func, void *arg, uint8_t priority);
void rtos_delay(uint32_t ms);
void rtos_start(void);
int scheduler_pick_next(void);

#endif