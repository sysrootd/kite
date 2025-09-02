#ifndef SPI_H
#define SPI_H

#include "stm32f401.h"
#include "gpio.h"

#define SPI_TX_BUF_SIZE 128
#define SPI_RX_BUF_SIZE 128

typedef struct {
    SPI_TypeDef *inst;

    GPIO_TypeDef *cs_port;
    uint8_t cs_pin;

    // TX ring buffer
    uint8_t tx_buf[SPI_TX_BUF_SIZE];
    volatile uint16_t tx_head;
    volatile uint16_t tx_tail;

    // RX ring buffer
    uint8_t rx_buf[SPI_RX_BUF_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
} SPI_Context;

void spi_init(SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint8_t cs_pin);
void spi_write(SPI_TypeDef *spi, const uint8_t *data, uint16_t len);
int  spi_read(SPI_TypeDef *spi);   // returns -1 if no data available

void SPI1_IRQHandler(void);
void SPI2_IRQHandler(void);

#endif
