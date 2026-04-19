#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include "mcu_pheripherial.h"

typedef enum
{
    TIMER_PERIODIC = 0U,
    TIMER_ONESHOT  = 1U
} timer_mode_t;

typedef void (*timer_callback_t)(void);

void timer_init(TIM_TypeDef *tim, uint32_t timer_clock_hz, uint32_t frequency_hz, timer_mode_t mode);
void timer_us_init(TIM_TypeDef *tim, uint32_t timer_clock_hz);
void timer_set_callback(TIM_TypeDef *tim, timer_callback_t callback);
void timer_start(TIM_TypeDef *tim);
void timer_stop(TIM_TypeDef *tim);
void timer_reset(TIM_TypeDef *tim);
void timer_irq_handler(TIM_TypeDef *tim);
uint32_t timer_get_us(void);

#endif