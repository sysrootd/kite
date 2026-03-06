#ifndef UART_H
#define UART_H

#include "stm32f4xx.h"
#include <stdarg.h>
#include <stdint.h>

uint32_t cstm_strlen(const char *str);
void uart_init(USART_TypeDef *uart, uint32_t pclk, uint32_t baud);

void uart_outchar(USART_TypeDef *uart, uint8_t data);
void uart_outstr(USART_TypeDef *uart, const char *str);

int uart_printf(USART_TypeDef *uart, const char *fmt, ...);

#endif