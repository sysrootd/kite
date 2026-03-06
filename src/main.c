#include "stm32f4xx.h"
#include "sched.h"
#include "uart.h"
#include "gpio.h"

#define RED_LED     14
#define GREEN_LED   13
#define BUFFER_SIZE 1

static volatile int buffer[BUFFER_SIZE];
static volatile int in  = 0;
static volatile int out = 0;

semaphore_t led_sem;

semaphore_t empty;
semaphore_t full;
mutex_t buffer_mutex;

void red_led_handler(void)
{
    while (1)
    {
        semaphore_wait(&led_sem);
        gpio_write(GPIOB, RED_LED, 1);
        task_delay(500);
        gpio_write(GPIOB, RED_LED, 0);
        semaphore_post(&led_sem);
    }
}

void green_led_handler(void)
{
    while (1)
    {
        semaphore_wait(&led_sem);
        gpio_write(GPIOB, GREEN_LED, 1);
        task_delay(500);
        gpio_write(GPIOB, GREEN_LED, 0);
        semaphore_post(&led_sem);
    }
}

void producer_task(void)
{
    int item = 0;

    while (1)
    {
        semaphore_wait(&empty);
        mutex_lock(&buffer_mutex);

        buffer[in] = item;
        uart_printf(USART2, "Produced %d\r\n", item);

        in = (in + 1) % BUFFER_SIZE;
        item++;

        mutex_unlock(&buffer_mutex);
        semaphore_post(&full);

    }
}

void consumer_task(void)
{
    int item;

    while (1)
    {
        semaphore_wait(&full);
        mutex_lock(&buffer_mutex);

        item = buffer[out];
        uart_printf(USART2, "Consumed %d\r\n", item);

        out = (out + 1) % BUFFER_SIZE;

        mutex_unlock(&buffer_mutex);
        semaphore_post(&empty);

    }
}

int main(void)
{
    gpio_init(GPIOB, RED_LED, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, GREEN_LED, OUTPUT, PP, FAST, PU, 0);

    uart_init(USART2, 16000000, 9600);

    semaphore_init(&led_sem, 1);

    semaphore_init(&empty, BUFFER_SIZE);
    semaphore_init(&full, 0);
    mutex_init(&buffer_mutex);

    create_task(1, producer_task, 512U);
    create_task(1, consumer_task, 512U);
    create_task(1, red_led_handler, 512U);
    create_task(1, green_led_handler, 512U);

    scheduler_init();
    scheduler_start();

    while (1);
}