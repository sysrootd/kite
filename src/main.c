#include "stm32f401.h"
#include "kernel.h"
#include "gpio.h"

void task1(void) {
    while (1) {
        gpio_write(GPIOB, 13, 1);
        for (volatile int i = 0; i < 100000; i++); // crude delay
        gpio_write(GPIOB, 13, 0);
        for (volatile int i = 0; i < 100000; i++);
    }
}

void task2(void) {
    while (1) {
        gpio_write(GPIOB, 14, 1);
        for (volatile int i = 0; i < 100000; i++);
        gpio_write(GPIOB, 14, 0);
        for (volatile int i = 0; i < 100000; i++);
    }
}

void task3(void) {
    while (1) {
        gpio_write(GPIOB, 15, 1);
        for (volatile int i = 0; i < 100000; i++);
        gpio_write(GPIOB, 15, 0);
        for (volatile int i = 0; i < 100000; i++);
    }
}

int main(void) {
    // Initialize GPIOs
    gpio_init(GPIOB, 13, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, 14, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, 15, OUTPUT, PP, FAST, PU, 0);

    // Initialize kernel
    KernelInit();

    KernelAddThreads(&task1, &task2, &task3);

    KernelLaunch(1);

    while (1);
}
