#ifndef UART_H
#define UART_H

#include "stm32f401.h"
#include "gpio.h"

#define UART_TX_BUF_SIZE   128
#define UART_RX_BUF_SIZE   128

typedef struct {
    USART_TypeDef *inst;

    // TX ring buffer
    uint8_t tx_buf[UART_TX_BUF_SIZE];
    volatile uint16_t tx_head;
    volatile uint16_t tx_tail;

    // RX ring buffer
    uint8_t rx_buf[UART_RX_BUF_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
} UART_Context;

void uart_init(USART_TypeDef *uart, uint32_t pclk, uint32_t baud);
void uart_write(USART_TypeDef *uart, const uint8_t *data, uint16_t len);
int  uart_read(USART_TypeDef *uart);   // returns -1 if no data available

void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART6_IRQHandler(void);

#endif
