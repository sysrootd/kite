
#include "stm32f4xx.h"
#include "sched.h"
#include "gpio.h"
#include "uart.h"

#define   RED_LED       13U
#define   GREEN_LED     14U
#define   BAUD_RATE     115200U

static mutex_t uart2_mutex;

void red_led_task(void)
{

    while (1)
    {
        gpio_write(GPIOB, RED_LED, 1);
        task_delay(100);

        gpio_write(GPIOB, RED_LED, 0);
        task_delay(100);
    }
}

void green_led_task(void)
{
    while (1)
    {
        gpio_write(GPIOB, GREEN_LED, 1);
        task_delay(200);

        gpio_write(GPIOB, GREEN_LED, 0);
        task_delay(200);
    }

}
void task_A(void)
{
    uint32_t next = global_systick;
    uint32_t count = 0;

    while (1)
    {
        if ((count % 10) == 0)
        {
            uint32_t observed_now = global_systick;
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
    uint32_t next = global_systick;
    uint32_t count = 0;

    while (1)
    {
        uint32_t observed_now = global_systick;
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
        task_yield();
    }
}

int main(void)
{
    //------------------system init-------------------------
    gpio_init(GPIOB, RED_LED, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, GREEN_LED, OUTPUT, PP, FAST, PU, 0);
    
    uart_init(USART2, BAUD_RATE);

    mutex_init(&uart2_mutex);

    create_task(2, task_A, 64U);
    create_task(2, task_B, 64U);
    create_task(1, hog_task, 64U);
    create_task(3, red_led_task, 64U);
    create_task(4, green_led_task, 64U);
    //‐‐------------------------------‐----------------------------------------------------------------------
    kite_start();

    while (1);
}
