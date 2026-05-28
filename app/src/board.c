#include "board.h"

void board_init(void)
{
    gpio_irq_input(GPIOC, UP_SWITCH, GPIO_TRIGGER_FALLING, GPIO_PULL_UP, KERNEL_INTERRUPT_PRIORITY);
    gpio_irq_input(GPIOC, DN_SWITCH, GPIO_TRIGGER_FALLING, GPIO_PULL_UP, KERNEL_INTERRUPT_PRIORITY);

    gpio_output(GPIOB, RED_LED);
    gpio_output(GPIOB, GREEN_LED);
    gpio_output(GPIOB, BUZZER);

    uart_init(USART2, BAUD_RATE);

    adc_init(GPIOC, LM35);

    lcd_init();

}