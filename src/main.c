#include "stm32f4xx.h"
#include "sched.h"
#include "gpio.h"
#include "uart.h"

#define   BLUE_LED     13

extern uint32_t global_systick_counter;
static mutex_t uart2_mutex;

void led_task(void)
{

    while (1)
    {
        gpio_write(GPIOC, BLUE_LED, 1);
        task_delay(200);

        gpio_write(GPIOC, BLUE_LED, 0);
        task_delay(200);
    }
}

void task_A(void)
{
    uint32_t next = global_systick_counter;
    uint32_t count = 0;

    while (1)
    {
        if ((count % 10) == 0)
        {
            uint32_t observed_now = global_systick_counter;
            long late = (long)((int32_t)(observed_now - next));

            mutex_lock(&uart2_mutex);
            uart_printf(USART2,
                        "[A] rel=%lu now=%lu late=%ld cnt=%lu\r\n",
                        next,
                        observed_now,
                        late,
                        count);
            mutex_unlock(&uart2_mutex);
        }

        count++;
        task_sleep_until(&next, 500);
    }
}

void task_B(void)
{
    uint32_t next = global_systick_counter;
    uint32_t count = 0;

    while (1)
    {
        uint32_t observed_now = global_systick_counter;
        long late = (long)((int32_t)(observed_now - next));

        mutex_lock(&uart2_mutex);
        uart_printf(USART2,
                    "[B] rel=%lu now=%lu late=%ld cnt=%lu\r\n",
                    next,
                    observed_now,
                    late,
                    count);
        mutex_unlock(&uart2_mutex);

        count++;
        task_sleep_until(&next, 500);
    }
}

void hog_task(void)
{
    volatile uint32_t x = 0;

    while (1)
    {
        for (volatile uint32_t i = 0; i < 50000; i++)
        {
            x++;
        }
        task_yeild();
    }
}

int main(void)
{
    //------------------system init-------------------------

    gpio_init(GPIOC, BLUE_LED, OUTPUT, PP, FAST, PU, 0);
    
    uart_init(USART2, SYSTEM_CLK, 115200);

    mutex_init(&uart2_mutex);

    task_init(4, task_A, 64U);
    task_init(3, task_B, 64U);
    task_init(2, hog_task, 64U);
    task_init(2, led_task, 64U);

    scheduler_init();

    //------------------------------------------------------
    scheduler_start();

    while (1);
}