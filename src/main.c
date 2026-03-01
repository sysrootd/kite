#include "kernel.h"
#include "gpio.h"

void task1_handler(void) {
    while (1) {
        gpio_write(GPIOB, 14, 1);
        task_delay(1000000);
        gpio_write(GPIOB, 14, 0);
        task_delay(1000000);
    }
}

void task2_handler(void) {
    while (1) {
        gpio_write(GPIOB, 13, 1);
        task_delay(200);
        gpio_write(GPIOB, 13, 0);
        task_delay(200);
    }
}

int main(void) {

	processor_faults_init();

	init_scheduler_stack(SCHED_STACK_START);

    gpio_init(GPIOB, 14, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, 13, OUTPUT, PP, FAST, PU, 0);

	init_tasks_stack();

	init_systick_timer(TICK_HZ);

	switch_sp_to_psp();

	task1_handler();

	for(;;);
}





