#include "kernel.h"
#include "gpio.h"

void task0(void) {
    while (1) {
        gpio_write(GPIOB, 14, 1);
        for (volatile int i = 0; i < 100000; i++);
        gpio_write(GPIOB, 14, 0);
        for (volatile int i = 0; i < 100000; i++);
        ThreadYield();  // Add explicit yield
    }
}

void task1(void) {
    while (1) {
        gpio_write(GPIOB, 13, 1);
        for (volatile int i = 0; i < 100000; i++);
        gpio_write(GPIOB, 13, 0);
        for (volatile int i = 0; i < 100000; i++);
        ThreadYield();  // Add explicit yield
    }
}

int main(void) {

    gpio_init(GPIOB, 14, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, 13, OUTPUT, PP, FAST, PU, 0);
    
    KernelInit();
    
    KernelAddThreads(task0, task1);

    DebugStackFrame();
    
    KernelLaunch(1);
    
    while (1);
}

void HardFault_Handler(void) {
    volatile uint32_t *sp;
    volatile uint32_t fault_pc;
    
    __asm volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq %0, msp\n"
        "mrsne %0, psp\n"
        : "=r"(sp)
    );
    
    fault_pc = sp[6];  // PC is at offset 24 (6 * 4) in stack frame
 
    while(1);
}