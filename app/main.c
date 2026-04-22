#include "mcu.h"
#include "sched.h"
#include "gpio.h"
#include "uart.h"

#define BUZZER     12U
#define RED_LED    13U
#define GREEN_LED  14U
#define BAUD_RATE  115200U

static mutex_t uart_mutex;

static void high_task(void)
{
    while (1)
    {
        mutex_lock(&uart_mutex);
        uart_printf(USART2, "High task\n\r");
        mutex_unlock(&uart_mutex);
        task_delay(100);
    }
}

static void medium_task(void)
{
    while (1)
    {
        mutex_lock(&uart_mutex);
        uart_printf(USART2, "Medium task\n\r");
        task_delay(50);
        mutex_unlock(&uart_mutex);
        task_delay(100);
    }
}

static void low_task(void)
{
    volatile uint32_t i;
    while (1)
    {
        mutex_lock(&uart_mutex);
        uart_printf(USART2, "Low task\n\r");
        mutex_unlock(&uart_mutex);
        for (i = 0; i < 10000; i++);
        task_delay(500);
    }
}

static void led_task_1(void)
{
    while (1)
    {
        gpio_toggle(GPIOB, GREEN_LED);
        task_delay(5000);
    }
}

static void led_task_2(void)
{
    while (1)
    {
        gpio_toggle(GPIOB, RED_LED);
        task_delay(500);

    }
}

static void buzzer_task(void)
{
    while (1)
    {
        gpio_write(GPIOB, BUZZER, 1);
        task_delay(500);
        gpio_write(GPIOB, BUZZER, 0);
        task_delay(1000);
    }
}

int main(void)
{
    gpio_init(GPIOB, RED_LED, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, GREEN_LED, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, BUZZER, OUTPUT, PP, FAST, PU, 0);
    uart_init(USART2, BAUD_RATE);

    mutex_init(&uart_mutex);

    create_task(4, high_task,   64U, "high");
    create_task(3, medium_task, 64U, "medium");
    create_task(2, low_task,    64U, "low");
    create_task(1, led_task_1,  64U, "led1");
    create_task(1, led_task_2,  64U, "led2");
    create_task(1, buzzer_task, 64U, "buzzer");

    uart_printf(USART2, ">>>>>BOOT: KITE RTOS<<<<\n\r");
    uart_printf(USART2, "SYSTEM CLOCK: %luMz\n\r\n", SystemCoreClock);

    kite_start();

    while (1)
    {
        __WFI();
    }
}