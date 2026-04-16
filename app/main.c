#include "mcu_pheripherial.h"
#include "sched.h"
#include "gpio.h"
#include "uart.h"
#include "timer.h"
#include "stats.h"
#include "config.h"

#define RED_LED       13U
#define GREEN_LED     14U
#define BAUD_RATE     115200U

static mutex_t uart_mutex;

static void blink_task(void)
{
    while (1)
    {
        gpio_write(GPIOB, RED_LED, 1);
        task_delay(500);
        gpio_write(GPIOB, RED_LED, 0);
        task_delay(500);
    }
}

static void heartbeat_task(void)
{
    while (1)
    {
        gpio_write(GPIOB, GREEN_LED, 1);
        task_delay(100);
        gpio_write(GPIOB, GREEN_LED, 0);
        task_delay(900);
    }
}

static void cpu_workload_task(void)
{
    while (1)
    {
        uint32_t start_us = timer_get_us();
        uint32_t work = 0U;

        for (volatile uint32_t i = 0; i < 600000U; i++)
        {
            work += (i & 1U);
        }

        uint32_t elapsed = timer_get_us() - start_us;

        mutex_lock(&uart_mutex);
        uart_printf(USART2,
            "[WORK] run=%lu us work=%lu\n\r",
            (unsigned long)elapsed,
            (unsigned long)work);
        mutex_unlock(&uart_mutex);

        task_delay(2000);
    }
}

static void stats_task(void)
{
    while (1)
    {
        mutex_lock(&uart_mutex);
        stats_report(USART2);
        mutex_unlock(&uart_mutex);

        task_delay(3000);
    }
}

int main(void)
{
    gpio_init(GPIOB, RED_LED, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, GREEN_LED, OUTPUT, PP, FAST, PU, 0);

    uart_init(USART2, BAUD_RATE);
    timer_init();
    stats_init();

    mutex_init(&uart_mutex);

    uart_printf(USART2,
        "Kite RTOS boot: CPU=%lu Hz, DWT=%u\n\r",
        (unsigned long)SystemCoreClock,
        timer_has_cycle_counter() ? 1U : 0U);

    create_task(1, blink_task,     64U, "blink");
    create_task(1, heartbeat_task, 64U, "heartbeat");
    create_task(2, cpu_workload_task, 128U, "workload");
    create_task(3, stats_task,     128U, "stats");

    kite_start();

    while (1)
    {
        __asm volatile ("wfi");
    }
}
