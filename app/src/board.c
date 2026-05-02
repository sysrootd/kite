#include "board.h"
#include "gpio.h"
#include "uart.h"
#include "sched.h"
#include "adc.h"
#include "lcd.h"

#define LM35       0
#define UP_SWITCH  8U
#define DN_SWITCH  9U
#define BUZZER     12U
#define RED_LED    13U
#define GREEN_LED  14U
#define BAUD_RATE  115200U

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