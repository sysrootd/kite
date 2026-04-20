#include "mcu_pheripherial.h"
#include "sched.h"
#include "gpio.h"
#include "uart.h"

#define RED_LED    13U
#define GREEN_LED  14U
#define BAUD_RATE  115200U

static mutex_t uart_mutex;

static void high_task(void)
{
    while (1)
    {
        mutex_lock(&uart_mutex);
        uart_printf(USART2, "H\n\r");
        mutex_unlock(&uart_mutex);
        task_delay(10);
    }
}

static void medium_task(void)
{
    while (1)
    {
        mutex_lock(&uart_mutex);
        uart_printf(USART2, "M\n\r");
        task_delay(50);
        mutex_unlock(&uart_mutex);
        task_delay(20);
    }
}

static void low_task(void)
{
    volatile uint32_t i;
    while (1)
    {
        for (i = 0; i < 10000; i++);
        task_delay(5);
    }
}

static void led_task_1(void)
{
    while (1)
    {
        gpio_write(GPIOB, GREEN_LED, 1);
        task_delay(100);
        gpio_write(GPIOB, GREEN_LED, 0);
        task_delay(100);
    }
}

static void led_task_2(void)
{
    while (1)
    {
        gpio_write(GPIOB, RED_LED, 1);
        task_delay(100);
        gpio_write(GPIOB, RED_LED, 0);
        task_delay(100);
    }
}

int main(void)
{
    gpio_init(GPIOB, RED_LED, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, GREEN_LED, OUTPUT, PP, FAST, PU, 0);
    uart_init(USART2, BAUD_RATE);

    mutex_init(&uart_mutex);

    create_task(4, high_task,   128U, "high");
    create_task(3, medium_task, 128U, "medium");
    create_task(2, low_task,    128U, "low");
    create_task(1, led_task_1,  128U, "led1");
    create_task(1, led_task_2,  128U, "led2");

    kite_start();

    while (1)
    {
        __WFI();
    }
}