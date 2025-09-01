#include "rtos.h"
#include "gpio.h"
#include "uart.h"

void task1(void *arg) {
    while (1) {
        gpio_write(GPIOB, 13, 1);
        rtos_delay(500); 
        gpio_write(GPIOB, 13, 0);
        rtos_delay(500);
    }
}

void task2(void *arg) {
        while (1) {
        gpio_write(GPIOB, 14, 1);
        rtos_delay(500); 
        gpio_write(GPIOB, 14, 0);
        rtos_delay(500);
    }
}

int main(void) {

    gpio_init(GPIOb, 13, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOb, 14, OUTPUT, PP, FAST, PU, 0);

    rtos_init();
    rtos_create_task(task1, 0, 2);
    rtos_create_task(task2, 0, 1);
    rtos_start();
}
