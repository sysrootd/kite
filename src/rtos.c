#include "rtos.h"
#include "stm32f401.h"

static TCB tasks[MAX_TASKS];
static int task_count = 0;
static int current = -1;

volatile uint32_t sys_ticks = 0;

extern void PendSV_Handler(void);

void rtos_init(void) {
    for (int i = 0; i < MAX_TASKS; i++) tasks[i].active = 0;

    // Setup SysTick (1ms tick @ 16 MHz)
    SYST->RVR = 16000 - 1;
    SYST->CVR = 0;
    SYST->CSR = 7; // ENABLE | TICKINT | CLKSOURCE
}

void rtos_create_task(task_func_t func, void *arg, uint8_t priority) {
    if (task_count >= MAX_TASKS) return;

    TCB *t = &tasks[task_count];

    // Initialize stack (descending)
    uint32_t *sp = &(t->stack_mem[STACK_SIZE - 16]); // room for 16 regs
    sp[8]  = 0x01000000;       // xPSR (Thumb state)
    sp[9]  = (uint32_t)func;   // PC (task entry)
    sp[10] = 0xFFFFFFFD;       // LR (return to thread mode)
    sp[0]  = (uint32_t)arg;    // R0 (argument)

    t->stack_ptr = sp;
    t->priority = priority;
    t->state = TASK_READY;
    t->delay_ticks = 0;
    t->active = 1;

    task_count++;
}

// Delay current task for ms ticks
void rtos_delay(uint32_t ms) {
    tasks[current].delay_ticks = ms;
    tasks[current].state = TASK_BLOCKED;
    SCB->ICSR |= (1 << 28); // trigger PendSV immediately
}

static int scheduler_pick_next(void) {
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

    __asm volatile(
        "LDR r0, =tasks           \n"
        "LDR r1, =current         \n"
        "LDR r1, [r1]             \n"
        "LSLS r1, r1, #6          \n"   // index * sizeof(TCB approx)
        "ADD r0, r0, r1           \n"
        "LDR r0, [r0]             \n"   // stack_ptr
        "MSR psp, r0              \n"
        "MOVS r0, #2              \n"
        "MSR CONTROL, r0          \n"
        "ISB                      \n"
        "POP {r4-r11}             \n"
        "POP {r0-r3, r12, lr, pc, xPSR} \n"
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

    SCB_ICSR |= (1 << 28); // trigger PendSV
}

void PendSV_Handler(void) {
    __asm volatile(
        // Save context of current task
        "MRS r0, psp              \n"
        "STMDB r0!, {r4-r11}      \n"

        "LDR r1, =tasks           \n"
        "LDR r2, =current         \n"
        "LDR r3, [r2]             \n"
        "LSLS r3, r3, #6          \n"  // index * sizeof(TCB approx)
        "ADD r1, r1, r3           \n"
        "STR r0, [r1]             \n"  // save PSP

        // Call scheduler
        "BL scheduler_pick_next   \n"
        "STR r0, [r2]             \n"

        // Load next task PSP
        "LDR r1, =tasks           \n"
        "LSLS r0, r0, #6          \n"
        "ADD r1, r1, r0           \n"
        "LDR r0, [r1]             \n"
        "LDMIA r0!, {r4-r11}      \n"
        "MSR psp, r0              \n"

        "BX lr                    \n"
    );
}
