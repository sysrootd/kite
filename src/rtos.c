#include "rtos.h"
#include "stm32f401.h"

static TCB tasks[MAX_TASKS];
static int task_count = 0;
static int current = -1;

volatile uint32_t sys_ticks = 0;

void rtos_init(void) {
    for (int i = 0; i < MAX_TASKS; i++) {
        tasks[i].active = 0;
        tasks[i].stack_ptr = 0;
    }

    // Setup SysTick (1ms tick @ 16 MHz)
    SysTick->LOAD = 16000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = 0x07;
    
    // Set PendSV priority to lowest
    SCB->SHP[10] = 0xFF;
}

void rtos_create_task(task_func_t func, void *arg, uint8_t priority) {
    if (task_count >= MAX_TASKS) return;

    TCB *t = &tasks[task_count];

    // Initialize stack properly - use the END of the stack memory
    uint32_t *sp = (uint32_t *)((uint8_t *)t->stack_mem + STACK_SIZE);
    
    // Create initial stack frame for exception return
    // Stack grows downward, so we decrement the pointer
    *(--sp) = 0x01000000;       // xPSR (Thumb state)
    *(--sp) = (uint32_t)func;   // PC (task entry point)
    *(--sp) = 0xFFFFFFFD;       // LR (return to thread mode with PSP)
    *(--sp) = 0;                // R12
    *(--sp) = 0;                // R3
    *(--sp) = 0;                // R2
    *(--sp) = 0;                // R1
    *(--sp) = (uint32_t)arg;    // R0 (argument)
    
    // Callee-saved registers (will be manually popped)
    *(--sp) = 0;                // R11
    *(--sp) = 0;                // R10
    *(--sp) = 0;                // R9
    *(--sp) = 0;                // R8
    *(--sp) = 0;                // R7
    *(--sp) = 0;                // R6
    *(--sp) = 0;                // R5
    *(--sp) = 0;                // R4

    t->stack_ptr = sp;
    t->priority = priority;
    t->state = TASK_READY;
    t->delay_ticks = 0;
    t->active = 1;

    task_count++;
}

void rtos_delay(uint32_t ms) {
    if (current >= 0 && current < MAX_TASKS) {
        tasks[current].delay_ticks = ms;
        tasks[current].state = TASK_BLOCKED;
        SCB->ICSR |= (1 << 28);
    }
}

int scheduler_pick_next(void) {
    int best = -1;
    uint8_t best_prio = 0;

    for (int i = 0; i < task_count; i++) {
        if (tasks[i].active && tasks[i].state == TASK_READY) {
            if (best == -1 || tasks[i].priority > best_prio) {
                best = i;
                best_prio = tasks[i].priority;
            }
        }
    }
    return best;
}

void rtos_start(void) {
    current = scheduler_pick_next();
    if (current < 0) return;

    TCB *t = &tasks[current];
    uint32_t *stack = (uint32_t *)t->stack_ptr;
    
    uint32_t func_addr = stack[7];  // PC is at index 7 (28 bytes)
    uint32_t arg = stack[0];        // R0 is at index 0
    
    // Use the Cortex-M exception return mechanism
    __asm volatile(
        "MSR psp, %0              \n"   // Set process stack pointer
        "MOV r1, #2               \n"   // Switch to PSP
        "MSR CONTROL, r1          \n"
        "ISB                      \n"
        "MOV r0, %1               \n"   // Load argument into R0
        "BX %2                    \n"   // Jump to task function
        :
        : "r" (t->stack_ptr), "r" (arg), "r" (func_addr)
        : "r0", "r1", "memory"
    );
}

void SysTick_Handler(void) {
    sys_ticks++;

    for (int i = 0; i < task_count; i++) {
        if (tasks[i].state == TASK_BLOCKED && tasks[i].delay_ticks > 0) {
            tasks[i].delay_ticks--;
            if (tasks[i].delay_ticks == 0) {
                tasks[i].state = TASK_READY;
            }
        }
    }

    // Only trigger PendSV if we need to switch tasks
    int next = scheduler_pick_next();
    if (next != current && next != -1) {
        SCB->ICSR |= (1 << 28);
    }
}

__attribute__((naked)) void PendSV_Handler(void) {
    __asm volatile(
        // Save current context
        "MRS r0, psp              \n"
        "STMDB r0!, {r4-r11}      \n"
        
        "LDR r1, =tasks           \n"
        "LDR r2, =current         \n"
        "LDR r3, [r2]             \n"
        "MOV r4, #64              \n"   // sizeof(TCB)
        "MUL r3, r3, r4           \n"
        "ADD r1, r1, r3           \n"
        "STR r0, [r1]             \n"   // save stack pointer
        
        // Call scheduler
        "BL scheduler_pick_next   \n"
        "LDR r2, =current         \n"
        "STR r0, [r2]             \n"
        
        // Load next context
        "LDR r1, =tasks           \n"
        "MOV r4, #64              \n"
        "MUL r0, r0, r4           \n"
        "ADD r1, r1, r0           \n"
        "LDR r0, [r1]             \n"
        "LDMIA r0!, {r4-r11}      \n"
        "MSR psp, r0              \n"
        
        // Return from exception
        "BX lr                    \n"
    );
}