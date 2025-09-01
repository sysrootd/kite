#include "uart.h"

static UART_Context ctx1, ctx2, ctx6;

static UART_Context* get_ctx(USART_TypeDef *uart) {
    if (uart == USART1) return &ctx1;
    if (uart == USART2) return &ctx2;
    if (uart == USART6) return &ctx6;
    return 0;
}

void uart_init(USART_TypeDef *uart, uint32_t pclk, uint32_t baud) {
    if (uart == USART1) {
        RCC_APB2ENR |= (1U << 4);
        gpio_init(GPIOA, 9, AF, PP, FAST, PU, 7);   // TX
        gpio_init(GPIOA, 10, AF, PP, FAST, PU, 7);  // RX
        NVIC_ISER1 |= (1U << (USART1_IRQn - 32));
    } else if (uart == USART2) {
        RCC_APB1ENR |= (1U << 17);
        gpio_init(GPIOA, 2, AF, PP, FAST, PU, 7);   
        gpio_init(GPIOA, 3, AF, PP, FAST, PU, 7);  
        NVIC_ISER1 |= (1U << (USART2_IRQn - 32));
    } else if (uart == USART6) {
        RCC_APB2ENR |= (1U << 5);
        gpio_init(GPIOC, 6, AF, PP, FAST, PU, 8);   
        gpio_init(GPIOC, 7, AF, PP, FAST, PU, 8);   
        NVIC_ISER2 |= (1U << (USART6_IRQn - 64));
    }

    UART_Context *ctx = get_ctx(uart);
    ctx->inst = uart;
    ctx->tx_head = ctx->tx_tail = 0;
    ctx->rx_head = ctx->rx_tail = 0;

    uart->CR1 = 0;
    uart->BRR = (pclk + (baud / 2U)) / baud;
    uart->CR1 |= (1U << 13) | (1U << 2) | (1U << 3);  // UE, RE, TE
    uart->CR1 |= (1U << 5); // RXNEIE
}

void uart_write(USART_TypeDef *uart, const uint8_t *data, uint16_t len) {
    UART_Context *ctx = get_ctx(uart);

    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (ctx->tx_head + 1) % UART_TX_BUF_SIZE;
        while (next == ctx->tx_tail); // wait if buffer full
        ctx->tx_buf[ctx->tx_head] = data[i];
        ctx->tx_head = next;
    }
    uart->CR1 |= (1U << 7); // enable TXEIE
}

int uart_read(USART_TypeDef *uart) {
    UART_Context *ctx = get_ctx(uart);
    if (ctx->rx_head == ctx->rx_tail) return -1; // no data
    uint8_t d = ctx->rx_buf[ctx->rx_tail];
    ctx->rx_tail = (ctx->rx_tail + 1) % UART_RX_BUF_SIZE;
    return d;
}

static void uart_irq_handler(UART_Context *ctx) {
    USART_TypeDef *uart = ctx->inst;

    // RX
    if ((uart->SR & (1U << 5))) {
        uint8_t d = uart->DR;
        uint16_t next = (ctx->rx_head + 1) % UART_RX_BUF_SIZE;
        if (next != ctx->rx_tail) { // drop if full
            ctx->rx_buf[ctx->rx_head] = d;
            ctx->rx_head = next;
        }
    }

    // TX
    if ((uart->SR & (1U << 7)) && (uart->CR1 & (1U << 7))) {
        if (ctx->tx_head != ctx->tx_tail) {
            uart->DR = ctx->tx_buf[ctx->tx_tail];
            ctx->tx_tail = (ctx->tx_tail + 1) % UART_TX_BUF_SIZE;
        } else {
            uart->CR1 &= ~(1U << 7); // disable TXEIE
        }
    }
}

void USART1_IRQHandler(void) { uart_irq_handler(&ctx1); }
void USART2_IRQHandler(void) { uart_irq_handler(&ctx2); }
void USART6_IRQHandler(void) { uart_irq_handler(&ctx6); }
