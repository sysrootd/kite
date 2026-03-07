#include <stdarg.h>
#include <stdint.h>

#include "stm32f4xx.h"
#include "uart.h"


uint32_t cstm_strlen(const char *str)
{
    uint32_t len = 0;
    while (*str++) len++;
    return len;
}

static void uart_print_int(USART_TypeDef *uart, int num, int base)
{
    char buf[16];
    int i = 0;
    unsigned int n;

    if (num < 0 && base == 10)
    {
        uart_outchar(uart, '-');
        n = -num;
    }
    else
    {
        n = num;
    }

    do
    {
        int digit = n % base;

        if (digit < 10)
            buf[i++] = digit + '0';
        else
            buf[i++] = digit - 10 + 'A';

        n /= base;

    } while (n);

    while (i--)
        uart_outchar(uart, buf[i]);
}

// ---------------- uart init----------------

void uart_init(USART_TypeDef *uart, uint32_t pclk, uint32_t baud)
{
    if (uart == USART1)
    {
        RCC->APB2ENR |= (1U << 4);

        RCC->AHB1ENR |= (1U << 0);

        GPIOA->MODER |= (2U << 18) | (2U << 20);
        GPIOA->AFRH |= (7U << 4) | (7U << 8);   // AF7 for PA9, PA10
    }
    else if (uart == USART2)
    {
        RCC->APB1ENR |= (1U << 17);

        RCC->AHB1ENR |= (1U << 0);

        GPIOA->MODER |= (2U << 4) | (2U << 6);
        GPIOA->AFRL |= (7U << 8) | (7U << 12);  // AF7 for PA2, PA3
    }
    else if (uart == USART6)
    {
        RCC->APB2ENR |= (1U << 5);

        RCC->AHB1ENR |= (1U << 2);

        GPIOC->MODER |= (2U << 12) | (2U << 14);
        GPIOC->AFRL |= (8U << 24) | (8U << 28); // AF8 for PC6, PC7
    }

    uart->BRR = (pclk + (baud / 2U)) / baud;

    uart->CR1 = (1U << 13) | (1U << 2) | (1U << 3);
}

// ---------------- low level IO ----------------

void uart_outchar(USART_TypeDef *uart, uint8_t data)
{
    while (!(uart->SR & (1U << 7))); // TXE
    uart->DR = data;
}

void uart_outstr(USART_TypeDef *uart, const char *str)
{
    while (*str)
    {
        uart_outchar(uart, *str++);
    }
}

// ---------------- uart_printf ----------------

int uart_printf(USART_TypeDef *uart, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    int count = 0;

    while (*fmt)
    {
        if (*fmt != '%')
        {
            uart_outchar(uart, *fmt++);
            count++;
            continue;
        }

        fmt++;

        switch (*fmt)
        {
            case 'c':
            {
                char c = va_arg(args, int);
                uart_outchar(uart, c);
                break;
            }

            case 'd':
            {
                int val = va_arg(args, int);
                uart_print_int(uart, val, 10);
                break;
            }

            case 'x':
            {
                int val = va_arg(args, int);
                uart_print_int(uart, val, 16);
                break;
            }

            case 'o':
            {
                int val = va_arg(args, int);
                uart_print_int(uart, val, 8);
                break;
            }

            case 's':
            {
                char *str = va_arg(args, char *);
                uart_outstr(uart, str);
                break;
            }

            case '%':
            {
                uart_outchar(uart, '%');
                break;
            }

            default:
                uart_outchar(uart, '%');
                uart_outchar(uart, *fmt);
                break;
        }

        fmt++;
    }

    va_end(args);
    return count;
}