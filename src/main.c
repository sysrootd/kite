#include "stm32f4xx.h"
#include "sched.h"
#include "gpio.h"
#include "uart.h"

#define RED_LED       13U
#define GREEN_LED     14U
#define BAUD_RATE     115200U

static mutex_t uart2_mutex;
static semaphore_t sem1;
static mutex_t test_mutex;

static volatile uint32_t rr_a_cnt = 0;
static volatile uint32_t rr_b_cnt = 0;
static volatile uint32_t hp_cnt   = 0;
static volatile uint32_t prod_cnt = 0;
static volatile uint32_t cons_cnt = 0;
static volatile uint32_t low_cnt  = 0;
static volatile uint32_t mid_cnt  = 0;
static volatile uint32_t high_cnt = 0;

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
        task_delay(100);

        gpio_write(GPIOB, GREEN_LED, 0);
        task_delay(100);
    }
}

void task_rr_a(void)
{
    uint32_t next = global_systick;

    while (1)
    {
        rr_a_cnt++;

        mutex_lock(&uart2_mutex);
        uart_printf(USART2,
                    "[RR-A] rel=%lu now=%lu cnt=%lu\r\n",
                    next,
                    global_systick,
                    rr_a_cnt);
        mutex_unlock(&uart2_mutex);

        task_sleep_until(&next, 1000);
    }
}

void task_rr_b(void)
{
    uint32_t next = global_systick;

    while (1)
    {
        rr_b_cnt++;

        mutex_lock(&uart2_mutex);
        uart_printf(USART2,
                    "[RR-B] rel=%lu now=%lu cnt=%lu\r\n",
                    next,
                    global_systick,
                    rr_b_cnt);
        mutex_unlock(&uart2_mutex);

        task_sleep_until(&next, 1000);
    }
}

void task_high_periodic(void)
{
    uint32_t next = global_systick;

    while (1)
    {
        hp_cnt++;

        mutex_lock(&uart2_mutex);
        uart_printf(USART2,
                    "[HP  ] rel=%lu now=%lu cnt=%lu\r\n",
                    next,
                    global_systick,
                    hp_cnt);
        mutex_unlock(&uart2_mutex);

        task_sleep_until(&next, 500);
    }
}

void task_producer(void)
{
    uint32_t next = global_systick;

    while (1)
    {
        prod_cnt++;

        mutex_lock(&uart2_mutex);
        uart_printf(USART2,
                    "[PROD] post now=%lu cnt=%lu\r\n",
                    global_systick,
                    prod_cnt);
        mutex_unlock(&uart2_mutex);

        semaphore_post(&sem1);

        task_sleep_until(&next, 700);
    }
}

void task_consumer(void)
{
    while (1)
    {
        semaphore_wait(&sem1);

        cons_cnt++;

        mutex_lock(&uart2_mutex);
        uart_printf(USART2,
                    "[CONS] wake now=%lu cnt=%lu\r\n",
                    global_systick,
                    cons_cnt);
        mutex_unlock(&uart2_mutex);
    }
}

void task_low_mutex(void)
{
    uint32_t next = global_systick;

    while (1)
    {
        mutex_lock(&test_mutex);

        mutex_lock(&uart2_mutex);
        uart_printf(USART2,
                    "[LOW ] lock now=%lu\r\n",
                    global_systick);
        mutex_unlock(&uart2_mutex);

        for (volatile uint32_t i = 0; i < 400000; i++)
        {
        }

        low_cnt++;

        mutex_lock(&uart2_mutex);
        uart_printf(USART2,
                    "[LOW ] unlock now=%lu cnt=%lu\r\n",
                    global_systick,
                    low_cnt);
        mutex_unlock(&uart2_mutex);

        mutex_unlock(&test_mutex);

        task_sleep_until(&next, 3000);
    }
}

void task_mid_busy(void)
{
    uint32_t next = global_systick;
    volatile uint32_t x = 0;

    while (1)
    {
        mid_cnt++;

        mutex_lock(&uart2_mutex);
        uart_printf(USART2,
                    "[MID ] run now=%lu cnt=%lu\r\n",
                    global_systick,
                    mid_cnt);
        mutex_unlock(&uart2_mutex);

        for (volatile uint32_t i = 0; i < 250000; i++)
        {
            x++;
        }

        task_sleep_until(&next, 800);
    }
}

void task_high_mutex(void)
{
    uint32_t next = global_systick;

    task_delay(200);

    while (1)
    {
        mutex_lock(&uart2_mutex);
        uart_printf(USART2,
                    "[HIGH] wait lock now=%lu\r\n",
                    global_systick);
        mutex_unlock(&uart2_mutex);

        mutex_lock(&test_mutex);

        high_cnt++;

        mutex_lock(&uart2_mutex);
        uart_printf(USART2,
                    "[HIGH] got lock now=%lu cnt=%lu\r\n",
                    global_systick,
                    high_cnt);
        mutex_unlock(&uart2_mutex);

        for (volatile uint32_t i = 0; i < 80000; i++)
        {
        }

        mutex_unlock(&test_mutex);

        task_sleep_until(&next, 3000);
    }
}

int main(void)
{
    gpio_init(GPIOB, RED_LED, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, GREEN_LED, OUTPUT, PP, FAST, PU, 0);

    uart_init(USART2, BAUD_RATE);

    mutex_init(&uart2_mutex);
    mutex_init(&test_mutex);
    semaphore_init(&sem1, 0);

    create_task(2, task_rr_a,         64U);
    create_task(2, task_rr_b,         64U);
    create_task(4, task_high_periodic, 64U);

    create_task(2, task_producer,     64U);
    create_task(2, task_consumer,     64U);

    create_task(1, task_low_mutex,    96U);
    create_task(3, task_mid_busy,     64U);
    create_task(4, task_high_mutex,   96U);

    create_task(3, red_led_task,      64U);
    create_task(3, green_led_task,    64U);

    kite_start();

    while (1)
    {
    }
}