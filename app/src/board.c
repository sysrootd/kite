#include "board.h"
#include "gpio.h"
#include "uart.h"

#define BUZZER     12U
#define RED_LED    13U
#define GREEN_LED  14U
#define BAUD_RATE  115200U

void board_init(void)
{
    gpio_init(GPIOB, RED_LED, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, GREEN_LED, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, BUZZER, OUTPUT, PP, FAST, PU, 0);

    uart_init(USART2, BAUD_RATE);
}