#ifndef ADC_H
#define ADC_H

#include "mcu.h"
#include "gpio.h"
#include "coreM4.h"


#define ADC_COMMON_BASE           (ADC1_BASE + 0x300UL)

#define DMA2_BASE                 (AHB1PERIPH_BASE + 0x6400UL)
#define DMA2                      ((DMA_TypeDef *) DMA2_BASE)

#define DMA2_Stream0_BASE         (DMA2_BASE + 0x010UL)
#define DMA2_Stream0              ((DMA_Stream_TypeDef *) DMA2_Stream0_BASE)

static inline void adc_clk(void) {
    RCC->APB2ENR |= (1UL << 8);
}

static inline void adc_enable(ADC_TypeDef *adc) {
    adc->CR2 |= (1UL << 0);
}

static inline void adc_disable(ADC_TypeDef *adc) {
    adc->CR2 &= ~(1UL << 0);
}

static inline void adc_resolution(ADC_TypeDef *adc, int res) {
    adc->CR1 &= ~(3UL << 24);
    adc->CR1 |= ((uint32_t)res << 24);
}

static inline void adc_align(ADC_TypeDef *adc, int align) {
    if (align) adc->CR2 |= (1UL << 11);
    else adc->CR2 &= ~(1UL << 11);
}

static inline void adc_sample_time(ADC_TypeDef *adc, int ch, int sample) {
    if (ch < 10) {
        adc->SMPR2 &= ~(7UL << (ch * 3));
        adc->SMPR2 |=  (sample << (ch * 3));
    } else {
        ch -= 10;
        adc->SMPR1 &= ~(7UL << (ch * 3));
        adc->SMPR1 |=  (sample << (ch * 3));
    }
}

static inline int _adc_channel_from_pin(GPIO_TypeDef *port, int pin)
{
    if (port == GPIOA) {
        if (pin <= 7) return pin;
    }
    if (port == GPIOB) {
        if (pin == 0) return 8;
        if (pin == 1) return 9;
    }
    if (port == GPIOC) {
        if (pin <= 5) return pin + 10;
    }
    return -1;
}


static inline void adc_channel(ADC_TypeDef *adc, int ch) {
    adc->SQR3 = ch;
    adc->SQR1 &= ~(0xF << 20);
}

static inline void adc_start(ADC_TypeDef *adc) {
    adc->CR2 |= (1UL << 30);
}

static inline int adc_done(ADC_TypeDef *adc) {
    return (adc->SR & (1UL << 1));
}

static inline uint16_t adc_read(ADC_TypeDef *adc) {
    return (uint16_t)(adc->DR);
}

static inline uint16_t adc_read_blocking(ADC_TypeDef *adc) {
    adc_start(adc);
    while (!adc_done(adc));
    return adc_read(adc);
}


static inline int adc_init_pin(GPIO_TypeDef *port, int pin)
{
    int ch = _adc_channel_from_pin(port, pin);
    if (ch < 0) return -1;

    gpio_init(port, pin, GPIO_MODE_ANALOG, GPIO_OTYPE_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE, 0);

    adc_clk();
    adc_resolution(ADC1, 0);
    adc_align(ADC1, 0);

    adc_sample_time(ADC1, ch, 1);
    adc_channel(ADC1, ch);

    adc_enable(ADC1);

    return ch;
}

static inline uint16_t adc_read_pin(GPIO_TypeDef *port, int pin)
{
    int ch = _adc_channel_from_pin(port, pin);
    if (ch < 0) return 0;

    adc_channel(ADC1, ch);
    return adc_read_blocking(ADC1);
}

static inline void adc_scan_init(ADC_TypeDef *adc,
                                 GPIO_TypeDef **ports,
                                 int *pins,
                                 int len)
{
    adc_clk();

    adc->CR1 |= (1UL << 8);

    adc->SQR1 &= ~(0xF << 20);
    adc->SQR1 |= ((len - 1) << 20);

    for (int i = 0; i < len; i++) {

        int ch = _adc_channel_from_pin(ports[i], pins[i]);

        gpio_init(ports[i], pins[i], GPIO_MODE_ANALOG,
                  GPIO_OTYPE_PP, GPIO_SPEED_LOW,
                  GPIO_PULL_NONE, 0);

        adc_sample_time(adc, ch, 1);

        if (i < 6) {
            adc->SQR3 |= (ch << (i * 5));
        } else if (i < 12) {
            adc->SQR2 |= (ch << ((i - 6) * 5));
        } else {
            adc->SQR1 |= (ch << ((i - 12) * 5));
        }
    }

    adc_enable(adc);
}

static inline void adc_scan_start(ADC_TypeDef *adc) {
    adc->CR2 |= (1UL << 1);
    adc_start(adc);
}


static inline void adc_irq_enable(ADC_TypeDef *adc, int priority)
{
    adc->CR1 |= (1UL << 5);

    NVIC_SetPriority(ADC_IRQn, priority);
    NVIC_EnableIRQ(ADC_IRQn);
}

static inline void adc_irq_disable(ADC_TypeDef *adc)
{
    adc->CR1 &= ~(1UL << 5);
}

static inline int adc_irq_is_pending(void)
{
    return (ADC1->SR & (1UL << 1));
}

static inline void adc_irq_clear(void)
{
    ADC1->SR &= ~(1UL << 1);
}

static inline void adc_dma_init(ADC_TypeDef *adc, uint32_t *buf, int len)
{
    RCC->AHB1ENR |= (1UL << 22);

    DMA2_Stream0->CR &= ~(1UL << 0);
    while (DMA2_Stream0->CR & 1);

    DMA2->LIFCR = 0x3F;

    DMA2_Stream0->PAR  = (uint32_t)&adc->DR;
    DMA2_Stream0->M0AR = (uint32_t)buf;
    DMA2_Stream0->NDTR = len;

    DMA2_Stream0->CR =
        (0U << 25) |
        (1U << 13) |
        (1U << 11) |
        (1U << 10) |
        (1U << 8)  |
        (1U << 4);

    adc->CR2 |= (1UL << 8) | (1UL << 9);
}

static inline void adc_dma_start(void)
{
    DMA2_Stream0->CR |= 1;
    adc_start(ADC1);
}

static inline void adc_dma_stop(void)
{
    DMA2_Stream0->CR &= ~1UL;
    ADC1->CR2 &= ~(1UL << 8);
}


static inline void adc_enable_temp(void) {
    ADC_COMMON->CCR |= (1UL << 23);
}

static inline void adc_enable_vref(void) {
    ADC_COMMON->CCR |= (1UL << 23);
}

static inline void adc_enable_vbat(void) {
    ADC_COMMON->CCR |= (1UL << 22);
}

#endif