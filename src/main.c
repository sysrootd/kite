#include "stm32f4xx.h"
#include "sched.h"
#include "gpio.h"
#include "uart.h"

#define RED_LED       13U
#define GREEN_LED     14U
#define BAUD_RATE     115200U

static mutex_t uart2_mutex;
static mutex_t test_mutex;

static volatile uint32_t low_cnt  = 0;
static volatile uint32_t mid_cnt  = 0;
static volatile uint32_t high_cnt = 0;

void low_task(void)
{
    uint32_t next = global_systick;

    while (1)
    {
        mutex_lock(&test_mutex);

        mutex_lock(&uart2_mutex);
        uart_printf(USART2, "[LOW ] lock now=%lu\r\n", global_systick);
        mutex_unlock(&uart2_mutex);

        for (volatile uint32_t i = 0; i < 1200000; i++) { }

        low_cnt++;

        mutex_lock(&uart2_mutex);
        uart_printf(USART2, "[LOW ] unlock now=%lu cnt=%lu\r\n", global_systick, low_cnt);
        mutex_unlock(&uart2_mutex);

        mutex_unlock(&test_mutex);

        task_sleep_until(&next, 3000);
    }
}

void mid_task(void)
{
    volatile uint32_t x = 0;

    task_delay(100);

    while (1)
    {
        mid_cnt++;

        mutex_lock(&uart2_mutex);
        uart_printf(USART2, "[MID ] run now=%lu cnt=%lu\r\n", global_systick, mid_cnt);
        mutex_unlock(&uart2_mutex);

        for (volatile uint32_t i = 0; i < 400000; i++) { x++; }

        task_delay(1);   // ← blocks for 1 tick – allows lower priority tasks to run
    }
}

void high_task(void)
{
    uint32_t next = global_systick;

    task_delay(300);

    while (1)
    {
        mutex_lock(&uart2_mutex);
        uart_printf(USART2, "[HIGH] wait lock now=%lu\r\n", global_systick);
        mutex_unlock(&uart2_mutex);

        mutex_lock(&test_mutex);

        high_cnt++;

        mutex_lock(&uart2_mutex);
        uart_printf(USART2, "[HIGH] got lock now=%lu cnt=%lu\r\n", global_systick, high_cnt);
        mutex_unlock(&uart2_mutex);

        for (volatile uint32_t i = 0; i < 100000; i++) { }

        mutex_unlock(&test_mutex);

        task_sleep_until(&next, 3000);
    }
}

void red_led_task(void)
{
    while (1)
    {
        gpio_write(GPIOB, RED_LED, 1);
        task_delay(500);
        gpio_write(GPIOB, RED_LED, 0);
        task_delay(500);
    }
}


void green_led_task(void)
{
    while (1)
    {
        gpio_write(GPIOB, GREEN_LED, 1);
        task_delay(1000);
        gpio_write(GPIOB, GREEN_LED, 0);
        task_delay(1000);
    }
}

int main(void)
{
    gpio_init(GPIOB, RED_LED, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, GREEN_LED, OUTPUT, PP, FAST, PU, 0);

    uart_init(USART2, BAUD_RATE);

    mutex_init(&uart2_mutex);
    mutex_init(&test_mutex);

    create_task(1, low_task,      128U);
    create_task(2, mid_task,      128U);
    create_task(3, red_led_task,   64U);
    create_task(3, green_led_task, 64U);
    create_task(3, high_task,     128U);

    kite_start();

    while (1);
}