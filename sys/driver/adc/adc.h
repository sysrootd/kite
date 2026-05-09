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

void adc_clk(void);
void adc_enable(ADC_TypeDef *adc);
void adc_disable(ADC_TypeDef *adc);
void adc_resolution(ADC_TypeDef *adc, int res);
void adc_align(ADC_TypeDef *adc, int align);
void adc_sample_time(ADC_TypeDef *adc, int ch, int sample);
int _adc_channel_from_pin(GPIO_TypeDef *port, int pin);
void adc_channel(ADC_TypeDef *adc, int ch);
void adc_start(ADC_TypeDef *adc);
int adc_done(ADC_TypeDef *adc);
uint16_t adc_read(ADC_TypeDef *adc);
uint16_t adc_read_blocking(ADC_TypeDef *adc);
int adc_init(GPIO_TypeDef *port, int pin);
uint16_t adc_read_pin(GPIO_TypeDef *port, int pin);
void adc_scan_init(ADC_TypeDef *adc, GPIO_TypeDef **ports, int *pins, int len);
void adc_scan_start(ADC_TypeDef *adc);
void adc_irq_enable(ADC_TypeDef *adc, int priority);
void adc_irq_disable(ADC_TypeDef *adc);
int adc_irq_is_pending(void);
void adc_irq_clear(void);
void adc_dma_init(ADC_TypeDef *adc, uint32_t *buf, int len);
void adc_dma_start(void);
void adc_dma_stop(void);
void adc_enable_temp(void);
void adc_enable_vref(void);
void adc_enable_vbat(void);

#endif