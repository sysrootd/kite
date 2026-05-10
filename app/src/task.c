/*
 * task.c — Application tasks for KITE RTOS
 *
 * Concurrency model
 * -----------------
 *
 *  Producer-Consumer (temperature reading → display)
 *  ┌──────────────┐  sem_full   ┌──────────────────┐
 *  │ producer_task│ ──────────► │  consumer_task   │
 *  │   (reads ADC)│             │  (LCD + UART log) │
 *  └──────────────┘ ◄────────── └──────────────────┘
 *                    sem_empty
 *
 *  sem_empty  – initialised to 1  (one free slot in the shared buffer)
 *  sem_full   – initialised to 0  (nothing ready to consume yet)
 *
 *  UART mutual exclusion
 *  ---------------------
 *  uart_mutex  – any task that calls uart_printf() must hold this mutex
 *                for the duration of the call so that output lines from
 *                different tasks never interleave.
 */

#include "task.h"
#include "sched.h"
#include "gpio.h"
#include "uart.h"
#include "adc.h"
#include "lcd.h"

#define LM35        0
#define UP_SWITCH   8U
#define DN_SWITCH   9U
#define BUZZER      12U
#define RED_LED     13U
#define GREEN_LED   14U

#define LM35_VREF_MV    1200U   /* Reference voltage in mV (calibrated)    */
#define LM35_ADC_MAX    4095U   /* 12-bit ADC full scale                   */
#define LM35_MV_PER_DEG   10U  /* LM35 output: 10 mV / °C                 */

#define SAMPLE_DELAY    5000U   /* Producer sampling period (ticks)        */

// Shared buffer
static uint32_t shared_temp;
static uint8_t led_flag;

/*
 * Producer-consumer semaphores (classic two-semaphore pattern):
 *   sem_empty  counts free slots  – producer calls wait(), consumer calls post()
 *   sem_full   counts filled slots – producer calls post(), consumer calls wait()
 */
static semaphore_t sem_empty;  /* init = 1: one slot available             */
static semaphore_t sem_full;   /* init = 0: nothing to consume yet         */

/*
 * UART mutex:
 *   Any task that needs to call uart_printf() must lock uart_mutex first
 *   and unlock it immediately after, so output lines never overlap.
 */
static mutex_t uart_mutex;

static const char customChar[8] = {
    0x0E, 0x0A, 0x0E,
    0x00, 0x00, 0x00, 0x00, 0x00
};

static void load_degree_char(void)
{
    int i;
    lcd_write_cmd(0x40);
    for (i = 0; i < 8; i++)
        lcd_write_data(customChar[i]);
}

static uint32_t lm35_read_celsius(GPIO_TypeDef *port, int pin)
{
    uint32_t raw = adc_read_pin(port, pin);
    uint32_t temp = (raw * 330U) / 4095U;
    return temp;
}

static void uart_print_locked(const char *msg)
{
    mutex_lock(&uart_mutex);
    uart_printf(USART2, "%s\n\r", msg);
    mutex_unlock(&uart_mutex);
}

static void led1_task(void)
{
    while (1) {
        gpio_toggle(GPIOB, GREEN_LED);
        task_delay(500);
    }
}

static void led2_task(void)
{
    while (1) {
        if(led_flag)
            task_yield();
        else {
            gpio_toggle(GPIOB, RED_LED);
            task_delay(250);
        }
    }
}

static void temp_log_task(void)
{
    while (1) {
        uint32_t raw = adc_read_pin(GPIOC, LM35);
        uint32_t temp = (raw * 120U) / 4095U;
        
        mutex_lock(&uart_mutex);
        uart_printf(USART2, "ADC: %lu | Temp: %lu C\n\r", raw, temp);
        mutex_unlock(&uart_mutex);
        
        task_delay(SAMPLE_DELAY);
    }
}

/*/
 *
 *  Flow:
 *    1. semaphore_wait(&sem_empty)  – block until there is a free slot
 *    2. Write a new temperature sample into shared_temp
 *    3. semaphore_post(&sem_full)   – signal the consumer that data is ready
 *    4. Log "producer" over UART (mutex-protected)
 *    5. Wait SAMPLE_DELAY ticks before the next reading
 */
static void producer_task(void)
{
    while (1) {
        /* Wait for the consumer to have consumed the previous value */
        semaphore_wait(&sem_empty);

        /* Critical section: update the shared buffer */
        shared_temp = lm35_read_celsius(GPIOC, LM35);

        /* Notify the consumer that new data is available */
        semaphore_post(&sem_full);

        /* UART log – mutex ensures no overlap with consumer's log line */
        uart_print_locked("producer");

        /* Yield the CPU for the rest of the sampling period */
        task_delay(SAMPLE_DELAY);
    }
}

/*
 *
 *  Flow:
 *    1. semaphore_wait(&sem_full)   – block until the producer has new data
 *    2. Read shared_temp (safe: producer is blocked on sem_empty)
 *    3. Update LCD display
 *    4. semaphore_post(&sem_empty)  – release the slot for the next sample
 *    5. Log "consumer" over UART (mutex-protected)
 */
static void consumer_task(void)
{
    char temp_str[10];

    lcd_write_cmd(0x83);
    lcd_write_str("Temp:   C");
    load_degree_char();

    while (1) {
        /* Block until the producer has deposited a new value */
        semaphore_wait(&sem_full);

        /* Safe to read: producer is now blocked on sem_empty */
        itoa(shared_temp, temp_str);

        /* Update LCD */
        lcd_write_cmd(0x88);
        lcd_write_str(temp_str);
        lcd_write_cmd(0x8a);
        lcd_write_data(0x00);          /* custom degree symbol             */

        /* Release the slot so the producer can write the next sample */
        semaphore_post(&sem_empty);

        /* UART log – mutex ensures no overlap with producer's log line */
        uart_print_locked("consumer");
    }
}

void EXTI9_5_IRQHandler(void)
{
    if (gpio_irq_is_pending(DN_SWITCH)) {
        led_flag = 1;
        gpio_write(GPIOB, BUZZER, 1);
        gpio_irq_clear_pending(DN_SWITCH);
    }

    if (gpio_irq_is_pending(UP_SWITCH)) {
        led_flag = 0;
        gpio_write(GPIOB, BUZZER, 0);
        gpio_irq_clear_pending(UP_SWITCH);
    }
}

void tasks_init(void)
{
    /* sem_empty = 1  → one free slot exists in the shared buffer          */
    semaphore_init(&sem_empty, 1);

    /* sem_full  = 0  → nothing ready to consume yet                       */
    semaphore_init(&sem_full,  0);

    mutex_init(&uart_mutex);

    create_task(4, led1_task,       128U,  "led1");
    create_task(4, led2_task,       128U,  "led2");
    create_task(3, temp_log_task,   128U,  "temp_log");
    create_task(2, producer_task,   128U,  "producer");
    create_task(1, consumer_task,   128U,  "consumer");

    uart_printf(USART2, ">>>>>BOOTING: KITE RTOS<<<<\n\r");
    uart_printf(USART2, "SYSTEM CLOCK: %luHz\n\r\n", SystemCoreClock);
}