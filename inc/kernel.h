#ifndef KERNEL_H_
#define KERNEL_H_

#include <stdint.h>
#include "../system/cmsis/cmsis_gcc.h"

#define MAX_TASKS   3

/* some stack memory calculations */
#define SIZE_TASK_STACK          1024U
#define SIZE_SCHED_STACK         1024U

/*--- memory layout provided by linker ---*/
extern uint32_t _estack;           /* top of RAM (initial MSP) */
extern uint32_t _sbss;             /* start of BSS in RAM, also start of free RAM */
extern uint32_t _ebss;             /* end of BSS in RAM */
extern uint32_t _Min_Heap_Size;    /* heap size from linker script */
extern uint32_t _Min_Stack_Size;   /* stack size from linker script */

/* compute useful addresses at compile time */
#define SRAM_START   ((uint32_t)(&_sbss))   /* first free byte after pre‑init */
#define SRAM_END     ((uint32_t)(&_estack)) /* one‑past‑the‑end of RAM */

/* compute stack starts relative to top of RAM (provided by linker) */
#define RAM_END   ((uint32_t)(&_estack))
#define T1_STACK_START   RAM_END
#define T2_STACK_START   (RAM_END - SIZE_TASK_STACK)
#define IDLE_STACK_START (RAM_END - 2 * SIZE_TASK_STACK)
#define SCHED_STACK_START (RAM_END - 3 * SIZE_TASK_STACK)

#define TICK_HZ 1000U

#define DUMMY_XPSR  0x01000000U

#define TASK_READY_STATE  0x00
#define TASK_BLOCKED_STATE  0XFF

#define INTERRUPT_DISABLE()  do{__asm volatile ("MOV R0,#0x1"); asm volatile("MSR PRIMASK,R0"); } while(0)

#define INTERRUPT_ENABLE()  do{__asm volatile ("MOV R0,#0x0"); asm volatile("MSR PRIMASK,R0"); } while(0)


void task1_handler(void); //This is task1
void task2_handler(void); //this is task2


void init_systick_timer(uint32_t tick_hz);
__attribute__((naked)) void init_scheduler_stack(uint32_t sched_top_of_stack);
void init_tasks_stack(void);
void processor_faults_init(void);
__attribute__((naked)) void switch_sp_to_psp(void);
uint32_t get_psp_value(void);

void task_delay(uint32_t tick_count);

typedef struct
{
	uint32_t psp_value;
	uint32_t block_count;
	uint8_t  current_state;
	void (*task_handler)(void);
}TCB_t;

#endif