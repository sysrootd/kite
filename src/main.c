#include "kernel.h"
#include "gpio.h"

void task1(void) {
    while (1) {
        gpio_write(GPIOB, 13, 1);
        for (volatile int i = 0; i < 100000; i++);
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


int main(void) {

    gpio_init(GPIOB, 13, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, 14, OUTPUT, PP, FAST, PU, 0);

    KernelInit();

    KernelAddThreads(&task1, &task2);

    KernelLaunch(1);

    while (1);
}


void HardFault_Handler(void) {
    __asm volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "ldr r1, [r0, #24]\n"
        "b .\n"
    );
}
