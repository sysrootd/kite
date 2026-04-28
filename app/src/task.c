#include "task.h"
#include "sched.h"
#include "gpio.h"
#include "uart.h"

#define UP_SWITCH  8U
#define DN_SWITCH  9u
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

static void led_task(void)
{
    while (1)
    {
        gpio_toggle(GPIOB, GREEN_LED);
        task_delay(100);
    }
}

void EXTI9_5_IRQHandler(void)
{
    if (gpio_irq_is_pending(DN_SWITCH))
    {
        gpio_write(GPIOB, RED_LED, 1);
        gpio_irq_clear_pending(DN_SWITCH);
    }
 
    if (gpio_irq_is_pending(UP_SWITCH))
    {
        gpio_write(GPIOB, RED_LED, 0);
        gpio_irq_clear_pending(UP_SWITCH);
    }
}

void tasks_init(void)
{
    mutex_init(&uart_mutex);

    create_task(4, high_task,   64U, "high");
    create_task(3, medium_task, 64U, "medium");
    create_task(2, led_task,    64U, "led1");
    create_task(1, low_task,    64U, "low");

    uart_printf(USART2, ">>>>>BOOTING: KITE RTOS<<<<\n\r");
    uart_printf(USART2, "SYSTEM CLOCK: %luMz\n\r\n", SystemCoreClock);
}