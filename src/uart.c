#include <stdarg.h>
#include <stdint.h>

#include "stm32f4xx.h"
#include "uart.h"

extern uint32_t SystemCoreClock;

#define RCC_CFGR_PPRE1_Pos      10U
#define RCC_CFGR_PPRE1_Msk      (0x7U << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE2_Pos      13U
#define RCC_CFGR_PPRE2_Msk      (0x7U << RCC_CFGR_PPRE2_Pos)

static uint32_t apb_prescaler(uint32_t cfgr, uint32_t mask, uint32_t pos)
{
    uint32_t presc = (cfgr & mask) >> pos;

    if (presc < 4U)
        return 1U;

    switch (presc)
    {
        case 4U: return 2U;
        case 5U: return 4U;
        case 6U: return 8U;
        case 7U: return 16U;
        default: return 1U;
    }
}

static uint32_t get_uart_pclk(USART_TypeDef *uart)
{
    uint32_t hclk = SystemCoreClock;

    if (uart == USART1 || uart == USART6)
    {
        return hclk / apb_prescaler(RCC->CFGR, RCC_CFGR_PPRE2_Msk, RCC_CFGR_PPRE2_Pos);
    }

    return hclk / apb_prescaler(RCC->CFGR, RCC_CFGR_PPRE1_Msk, RCC_CFGR_PPRE1_Pos);
}

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
        n = (unsigned int)(-(num + 1)) + 1U;
    }
    else
    {
        n = (unsigned int)num;
    }

    do
    {
        int digit = n % (unsigned int)base;

        if (digit < 10)
            buf[i++] = digit + '0';
        else
            buf[i++] = digit - 10 + 'A';

        n /= (unsigned int)base;

    } while (n);

    while (i--)
        uart_outchar(uart, buf[i]);
}

static void uart_print_uint(USART_TypeDef *uart, unsigned long num, int base)
{
    char buf[32];
    int i = 0;

    do
    {
        unsigned long digit = num % (unsigned long)base;

        if (digit < 10)
            buf[i++] = (char)(digit + '0');
        else
            buf[i++] = (char)(digit - 10 + 'A');

        num /= (unsigned long)base;

    } while (num);

    while (i--)
        uart_outchar(uart, buf[i]);
}

static void uart_print_long(USART_TypeDef *uart, long num, int base)
{
    unsigned long n;

    if (num < 0 && base == 10)
    {
        uart_outchar(uart, '-');
        n = (unsigned long)(-(num + 1)) + 1UL;
    }
    else
    {
        n = (unsigned long)num;
    }

    uart_print_uint(uart, n, base);
}

void uart_init(USART_TypeDef *uart, uint32_t baud)
{
    uint32_t pclk = get_uart_pclk(uart);

    if (uart == USART1)
    {
        RCC->APB2ENR |= (1U << 4);

        RCC->AHB1ENR |= (1U << 0);

        GPIOA->MODER |= (2U << 18) | (2U << 20);
        GPIOA->AFRH |= (7U << 4) | (7U << 8);
    }
    else if (uart == USART2)
    {
        RCC->APB1ENR |= (1U << 17);

        RCC->AHB1ENR |= (1U << 0);

        GPIOA->MODER |= (2U << 4) | (2U << 6);
        GPIOA->AFRL |= (7U << 8) | (7U << 12);
    }
    else if (uart == USART6)
    {
        RCC->APB2ENR |= (1U << 5);

        RCC->AHB1ENR |= (1U << 2);

        GPIOC->MODER |= (2U << 12) | (2U << 14);
        GPIOC->AFRL |= (8U << 24) | (8U << 28);
    }

    uart->BRR = (pclk + (baud / 2U)) / baud;

    uart->CR1 = (1U << 13) | (1U << 2) | (1U << 3);
}

void uart_outchar(USART_TypeDef *uart, uint8_t data)
{
    while (!(uart->SR & (1U << 7)));
    uart->DR = data;
}

void uart_outstr(USART_TypeDef *uart, const char *str)
{
    while (*str)
    {
        uart_outchar(uart, *str++);
    }
}

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
                char c = (char)va_arg(args, int);
                uart_outchar(uart, c);
                break;
            }

            case 'd':
            {
                int val = va_arg(args, int);
                uart_print_int(uart, val, 10);
                break;
            }

            case 'u':
            {
                unsigned int val = va_arg(args, unsigned int);
                uart_print_uint(uart, (unsigned long)val, 10);
                break;
            }

            case 'x':
            {
                unsigned int val = va_arg(args, unsigned int);
                uart_print_uint(uart, (unsigned long)val, 16);
                break;
            }

            case 'o':
            {
                unsigned int val = va_arg(args, unsigned int);
                uart_print_uint(uart, (unsigned long)val, 8);
                break;
            }

            case 's':
            {
                char *str = va_arg(args, char *);
                if (!str) str = "(null)";
                uart_outstr(uart, str);
                break;
            }

            case 'l':
            {
                fmt++;

                if (*fmt == 'u')
                {
                    unsigned long val = va_arg(args, unsigned long);
                    uart_print_uint(uart, val, 10);
                }
                else if (*fmt == 'd')
                {
                    long val = va_arg(args, long);
                    uart_print_long(uart, val, 10);
                }
                else if (*fmt == 'x')
                {
                    unsigned long val = va_arg(args, unsigned long);
                    uart_print_uint(uart, val, 16);
                }
                else
                {
                    uart_outchar(uart, '%');
                    uart_outchar(uart, 'l');
                    if (*fmt)
                        uart_outchar(uart, *fmt);
                }
                break;
            }

            case '%':
            {
                uart_outchar(uart, '%');
                break;
            }

            default:
            {
                uart_outchar(uart, '%');
                uart_outchar(uart, *fmt);
                break;
            }
        }

        if (*fmt)
            fmt++;
    }

    va_end(args);
    return count;
}