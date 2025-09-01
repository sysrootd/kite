#include "spi.h"

static SPI_Context ctx1, ctx2;

static SPI_Context* get_ctx(SPI_TypeDef *spi) {
    if (spi == SPI1) return &ctx1;
    if (spi == SPI2) return &ctx2;
    return 0;
}

void spi_init(SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint8_t cs_pin) {
    if (spi == SPI1) {
        RCC_APB2ENR |= (1U << 12);

        // PA5 = SCK, PA6 = MISO, PA7 = MOSI (AF5)
        gpio_init(GPIOA, 5, AF, PP, FAST, PU, 5);
        gpio_init(GPIOA, 6, AF, PP, FAST, PU, 5);
        gpio_init(GPIOA, 7, AF, PP, FAST, PU, 5);

        NVIC_ISER0 |= (1U << SPI1_IRQn);
    } else if (spi == SPI2) {
        RCC_APB1ENR |= (1U << 14);

        // PB13 = SCK, PB14 = MISO, PB15 = MOSI (AF5)
        gpio_init(GPIOB, 13, AF, PP, FAST, PU, 5);
        gpio_init(GPIOB, 14, AF, PP, FAST, PU, 5);
        gpio_init(GPIOB, 15, AF, PP, FAST, PU, 5);

        NVIC_ISER1 |= (1U << (SPI2_IRQn - 32));
    }

    SPI_Context *ctx = get_ctx(spi);
    ctx->inst = spi;
    ctx->tx_head = ctx->tx_tail = 0;
    ctx->rx_head = ctx->rx_tail = 0;
    ctx->cs_port = cs_port;
    ctx->cs_pin = cs_pin;

    // Configure CS pin as output high (inactive)
    gpio_init(cs_port, cs_pin, OUTPUT, PP, FAST, PU, 0);
    gpio_write(cs_port, cs_pin, 1);

    // Configure SPI: Master, fPCLK/16, 8-bit, CPOL=0, CPHA=0
    spi->CR1 = (1U << 2) | (1U << 6) | (0x3 << 3); // MSTR, SPE, BR=F_PCLK/16
    spi->CR2 = (1U << 6) | (1U << 7); // TXEIE, RXNEIE
}

void spi_write(SPI_TypeDef *spi, const uint8_t *data, uint16_t len) {
    SPI_Context *ctx = get_ctx(spi);

    // Assert CS low before writing
    gpio_write(ctx->cs_port, ctx->cs_pin, 0);

    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (ctx->tx_head + 1) % SPI_TX_BUF_SIZE;
        while (next == ctx->tx_tail); // wait if full
        ctx->tx_buf[ctx->tx_head] = data[i];
        ctx->tx_head = next;
    }
    spi->CR2 |= (1U << 7); // enable TXEIE
}

int spi_read(SPI_TypeDef *spi) {
    SPI_Context *ctx = get_ctx(spi);
    if (ctx->rx_head == ctx->rx_tail) return -1; // no data
    uint8_t d = ctx->rx_buf[ctx->rx_tail];
    ctx->rx_tail = (ctx->rx_tail + 1) % SPI_RX_BUF_SIZE;
    return d;
}

static void spi_irq_handler(SPI_Context *ctx) {
    SPI_TypeDef *spi = ctx->inst;

    // RX
    if (spi->SR & (1U << 0)) { // RXNE
        uint8_t d = spi->DR;
        uint16_t next = (ctx->rx_head + 1) % SPI_RX_BUF_SIZE;
        if (next != ctx->rx_tail) { // store if not full
            ctx->rx_buf[ctx->rx_head] = d;
            ctx->rx_head = next;
        }
    }

    // TX
    if ((spi->SR & (1U << 1)) && (spi->CR2 & (1U << 7))) { // TXE
        if (ctx->tx_head != ctx->tx_tail) {
            spi->DR = ctx->tx_buf[ctx->tx_tail];
            ctx->tx_tail = (ctx->tx_tail + 1) % SPI_TX_BUF_SIZE;
        } else {
            spi->CR2 &= ~(1U << 7); // disable TXEIE
            // Release CS high after last byte
            gpio_write(ctx->cs_port, ctx->cs_pin, 1);
        }
    }
}

void SPI1_IRQHandler(void) { spi_irq_handler(&ctx1); }
void SPI2_IRQHandler(void) { spi_irq_handler(&ctx2); }
