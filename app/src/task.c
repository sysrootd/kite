/*
 * task.c - Application Tasks for KITE RTOS
 *
 * Overview:
 *   This file implements the main application tasks for a temperature monitoring
 *   system using the KITE real-time operating system. The system reads temperature
 *   data from an LM35 temperature sensor, displays it on an LCD, controls LEDs,
 *   and logs data via UART.
 *
 * Main Tasks:
 *   - led1_task:      Toggles GREEN_LED at 500ms intervals (heartbeat)
 *   - led2_task:      Controls RED_LED based on led_flag (conditional blinking)
 *   - temp_log_task:  Reads ADC and logs temperature via UART at regular intervals
 *   - producer_task:  Reads temperature sensor and places data in shared buffer
 *   - consumer_task:  Retrieves temperature from shared buffer and displays on LCD
 *
 * Synchronization:
 *   - sem_empty:      Semaphore tracking free slots in shared buffer (init=1)
 *   - sem_full:       Semaphore tracking filled slots in shared buffer (init=0)
 *   - uart_mutex:     Mutex protecting UART output to prevent line interleaving
 *
 * Shared Resources:
 *   - shared_temp:    Temperature value shared between producer and consumer tasks
 *   - led_flag:       Flag to control RED_LED behavior from interrupt handler
 *
 * Interrupt Handler:
 *   - EXTI9_5_IRQHandler: Handles GPIO interrupts from UP_SWITCH and DN_SWITCH buttons
 */

#include "gpio.h"
#include "uart.h"
#include "adc.h"
#include "lcd.h"
#include "sched.h"  

#include "task.h"

static uint32_t shared_temp;
static uint8_t led_flag;

static semaphore_t sem_empty;  
static semaphore_t sem_full;   

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
        task_delay(100);
    }
}

static void led2_task(void)
{
    while (1) {
        if(led_flag) {
            gpio_write(GPIOB, RED_LED, 1);
            task_yield();
        }
        else {
            gpio_toggle(GPIOB, RED_LED);
            task_delay(500);
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

static void producer_task(void)
{
    while (1) {
        
        sem_wait(&sem_empty);
        shared_temp = lm35_read_celsius(GPIOC, LM35);        
        sem_post(&sem_full);
        uart_print_locked("producer");
        task_delay(SAMPLE_DELAY);
    }
}

static void consumer_task(void)
{
    char temp_str[10];

    lcd_write_cmd(0x83);
    lcd_write_str("Temp:   C");
    load_degree_char();

    while (1) {
        
        sem_wait(&sem_full);        
        itoa(shared_temp, temp_str);

        lcd_write_cmd(0x88);
        lcd_write_str(temp_str);
        lcd_write_cmd(0x8a);
        lcd_write_data(0x00);          
        sem_post(&sem_empty);
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
    
    sem_init(&sem_empty, 1);
    sem_init(&sem_full,  0);

    mutex_init(&uart_mutex);

    create_task(4, led1_task,       128U,  "led1");
    create_task(4, led2_task,       128U,  "led2");
    create_task(3, temp_log_task,   128U,  "temp_log");
    create_task(2, producer_task,   128U,  "producer");
    create_task(1, consumer_task,   128U,  "consumer");

    uart_printf(USART2, ">>>>>BOOTING: KITE RTOS<<<<\n\r");
    uart_printf(USART2, "SYSTEM CLOCK: %luHz\n\r\n", SystemCoreClock);
}