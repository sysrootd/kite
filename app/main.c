#include "mcu_pheripherial.h"
#include "sched.h"
#include "gpio.h"
#include "uart.h"
#include "timer.h"
#include "config.h"

#define RED_LED            13U
#define GREEN_LED          14U
#define BAUD_RATE          115200U
#define TIMER_CLOCK_HZ     SystemCoreClock
#define RTOS_TIMER_HZ      1000U

static mutex_t uart_mutex;
static volatile uint32_t low_priority_ticks = 0U;

// Hardware timer callback
static void periodic_timer_cb(void)
{
    global_systick++;
}

static void high_priority_task(void)
{
    while (1)
    {
        mutex_lock(&uart_mutex);
        uart_printf(USART2, "[HIGH] Acquired UART. Quick message!\n\r");
        mutex_unlock(&uart_mutex);
        task_delay(500);
    }
}

static void medium_priority_task(void)
{
    while (1)
    {
        mutex_lock(&uart_mutex);
        uart_printf(USART2, "[MED] Acquired UART. Holding for a long time...\n\r");
        
        task_delay(300);
        
        uart_printf(USART2, "[MED] Releasing UART now.\n\r");
        mutex_unlock(&uart_mutex);
        
        task_delay(400);
    }
}

static void low_priority_task(void)
{
    while (1)
    {
        low_priority_ticks++;
        
        if (low_priority_ticks % 10 == 0)
        {
            mutex_lock(&uart_mutex);
            uart_printf(USART2, "[LOW] Alive! Background ticks: %lu\n\r", (unsigned long)low_priority_ticks);
            mutex_unlock(&uart_mutex);
        }
        
        task_delay(50);
    }
}

static void green_led_task(void)
{
    while (1)
    {
        gpio_write(GPIOB, GREEN_LED, 1);
        task_delay(500);
        gpio_write(GPIOB, GREEN_LED, 0);
        task_delay(500);
    }
}

static void red_led_task(void)
{
    while (1)
    {
        gpio_write(GPIOB, RED_LED, 1);
        task_delay(500);
        gpio_write(GPIOB, RED_LED, 0);
        task_delay(500);
    }
}

void TIM2_IRQHandler(void)
{
    timer_irq_handler(TIM2);
}

int main(void)
{
    gpio_init(GPIOB, RED_LED, OUTPUT, PP, FAST, PU, 0);
    gpio_init(GPIOB, GREEN_LED, OUTPUT, PP, FAST, PU, 0);
    uart_init(USART2, BAUD_RATE);

    timer_us_init(TIM5, TIMER_CLOCK_HZ);
    timer_start(TIM5);
    timer_set_callback(TIM2, periodic_timer_cb);
    timer_init(TIM2, TIMER_CLOCK_HZ, RTOS_TIMER_HZ, TIMER_PERIODIC);
    NVIC_EnableIRQ(TIM2_IRQn);
    timer_start(TIM2);

    mutex_init(&uart_mutex);

    uart_printf(USART2, "\n\r=================================\n\r");
    uart_printf(USART2, ">>>>>Kite RTOS<<<<<\n\r");
    uart_printf(USART2, "CPU=%lu Hz\n\r", (unsigned long)SystemCoreClock);
    uart_printf(USART2, "=================================\n\r");

    create_task(3, high_priority_task, 256U, "high_task");
    create_task(2, medium_priority_task, 256U, "med_task");
    create_task(1, low_priority_task, 256U, "low_task");
    create_task(4, green_led_task, 256U, "green_led_task");
    create_task(4, red_led_task, 256U, "red_led_task");

    kite_start();

    while (1)
    {
        __asm volatile("wfi");
    }
}