#include "task.h"
#include "sched.h"
#include "gpio.h"
#include "uart.h"
#include "adc.h"
#include "lcd.h"

#define LM35       0
#define UP_SWITCH  8U
#define DN_SWITCH  9U
#define BUZZER     12U
#define RED_LED    13U
#define GREEN_LED  14U
#define BAUD_RATE  115200U

#define LM35_VREF_MV        3300U
#define LM35_ADC_MAX        4095U
#define LM35_MV_PER_DEG     10U
#define LM35_SAMPLE_DELAY   5000U

static mutex_t uart_mutex;


uint32_t celsious;
char temp[10];

char customChar[] = {
  0x0E,
  0x0A,
  0x0E,
};

static void custom_char_temp(char cc[]){
	int i;
	for(i=0;i<8;i++)
        lcd_write_data(cc[i]);
}

static uint32_t lm35_read_celsius(GPIO_TypeDef *port, int pin)
{
    uint32_t raw = adc_read_pin(port, pin);
    return (raw * LM35_VREF_MV) / (LM35_ADC_MAX * LM35_MV_PER_DEG);
}

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

static void temp_task(void)
{
    uint32_t temp_ch10;
    lcd_write_cmd(0x83);
    lcd_write_str("Temp:   C");
    lcd_write_cmd(0x40);
    custom_char_temp(customChar);

    while (1)
    {
        temp_ch10 = lm35_read_celsius(GPIOC, LM35);
        itoa(temp_ch10, temp);
        mutex_lock(&uart_mutex);
        uart_printf(USART2, "TEMP: %u\n\r", temp_ch10);
        lcd_write_cmd(0x8a);
        lcd_write_data(0x00);
		lcd_write_cmd(0x88);
		lcd_write_str(temp);
        mutex_unlock(&uart_mutex);

        task_delay(LM35_SAMPLE_DELAY);
    }
}

void EXTI9_5_IRQHandler(void)
{
    if (gpio_irq_is_pending(DN_SWITCH))
    {
        gpio_write(GPIOB, RED_LED, 1);
        gpio_write(GPIOB, BUZZER, 0);
        gpio_irq_clear_pending(DN_SWITCH);
    }

    if (gpio_irq_is_pending(UP_SWITCH))
    {
        gpio_write(GPIOB, RED_LED, 0);
        gpio_write(GPIOB, BUZZER, 1);
        gpio_irq_clear_pending(UP_SWITCH);
    }
}

void tasks_init(void)
{
    mutex_init(&uart_mutex);

    create_task(4, high_task,   64U, "high");
    create_task(3, medium_task, 64U, "medium");
    create_task(2, led_task,    64U, "led1");
    create_task(2, temp_task,   128U, "temp");
    create_task(1, low_task,    64U, "low");

    uart_printf(USART2, ">>>>>BOOTING: KITE RTOS<<<<\n\r");
    uart_printf(USART2, "SYSTEM CLOCK: %luMz\n\r\n", SystemCoreClock);
}