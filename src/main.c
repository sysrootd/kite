#include "kernel.h"
#include "gpio.h"

void task1_handler(void) {
    while (1) {
        gpio_write(GPIOB, 14, 1);
        task_delay(500);
        gpio_write(GPIOB, 14, 0);
        task_delay(500);
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

    gpio_init(GPIOB, 14, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, 13, OUTPUT, PP, FAST, PU, 0);

	create_task(0, task1_handler, 256U);
	create_task(0, task2_handler, 256U);

	scheduler_init();
	scheduler_start();


    for (;;);

}





