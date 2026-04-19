#include <stdint.h>
#include "timer.h"

#define TIM_CR1_CEN   (1U << 0)
#define TIM_CR1_OPM   (1U << 3)
#define TIM_DIER_UIE  (1U << 0)
#define TIM_SR_UIF    (1U << 0)
#define TIM_EGR_UG    (1U << 0)

#define RCC_APB1ENR_TIM2EN   (1U << 0)
#define RCC_APB1ENR_TIM3EN   (1U << 1)
#define RCC_APB1ENR_TIM4EN   (1U << 2)
#define RCC_APB1ENR_TIM5EN   (1U << 3)

#define RCC_APB2ENR_TIM1EN   (1U << 0)
#define RCC_APB2ENR_TIM9EN   (1U << 2)
#define RCC_APB2ENR_TIM10EN  (1U << 3)
#define RCC_APB2ENR_TIM11EN  (1U << 4)

typedef struct
{
    TIM_TypeDef *tim;
    timer_callback_t callback;
    uint8_t in_use;
    uint8_t is_us_timebase;
} timer_state_t;

static timer_state_t timer_state[8];
static TIM_TypeDef *us_timebase = 0;

static void rcc_enable_timer(TIM_TypeDef *tim)
{
    if (tim == TIM2)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    }
    else if (tim == TIM3)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    }
    else if (tim == TIM4)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    }
    else if (tim == TIM5)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    }
    else if (tim == TIM1)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    }
    else if (tim == TIM9)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
    }
    else if (tim == TIM10)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    }
    else if (tim == TIM11)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
    }
}

static timer_state_t *find_state(TIM_TypeDef *tim)
{
    for (uint32_t i = 0U; i < (uint32_t)(sizeof(timer_state) / sizeof(timer_state[0])); i++)
    {
        if (timer_state[i].in_use != 0U && timer_state[i].tim == tim)
        {
            return &timer_state[i];
        }
    }

    for (uint32_t i = 0U; i < (uint32_t)(sizeof(timer_state) / sizeof(timer_state[0])); i++)
    {
        if (timer_state[i].in_use == 0U)
        {
            timer_state[i].in_use = 1U;
            timer_state[i].tim = tim;
            timer_state[i].callback = 0;
            timer_state[i].is_us_timebase = 0U;
            return &timer_state[i];
        }
    }

    return 0;
}

void timer_set_callback(TIM_TypeDef *tim, timer_callback_t callback)
{
    timer_state_t *st = find_state(tim);
    if (st != 0)
    {
        st->callback = callback;
    }
}

void timer_init(TIM_TypeDef *tim, uint32_t timer_clock_hz, uint32_t frequency_hz, timer_mode_t mode)
{
    timer_state_t *st = find_state(tim);
    uint32_t prescaler;
    uint32_t period;

    if ((st == 0) || (timer_clock_hz == 0U) || (frequency_hz == 0U))
    {
        return;
    }

    rcc_enable_timer(tim);

    tim->CR1 &= ~TIM_CR1_CEN;
    tim->DIER = 0U;
    tim->SR = 0U;
    tim->CNT = 0U;

    prescaler = timer_clock_hz / 1000000U;
    if (prescaler == 0U)
    {
        prescaler = 1U;
    }
    tim->PSC = prescaler - 1U;

    period = 1000000U / frequency_hz;
    if (period == 0U)
    {
        period = 1U;
    }
    tim->ARR = period - 1U;

    tim->EGR = TIM_EGR_UG;

    if (mode == TIMER_ONESHOT)
    {
        tim->CR1 |= TIM_CR1_OPM;
    }
    else
    {
        tim->CR1 &= ~TIM_CR1_OPM;
    }

    tim->DIER |= TIM_DIER_UIE;

    st->tim = tim;
    st->is_us_timebase = 0U;
}

void timer_us_init(TIM_TypeDef *tim, uint32_t timer_clock_hz)
{
    timer_state_t *st = find_state(tim);
    uint32_t prescaler;

    if ((st == 0) || (timer_clock_hz == 0U))
    {
        return;
    }

    rcc_enable_timer(tim);

    tim->CR1 &= ~TIM_CR1_CEN;
    tim->DIER = 0U;
    tim->SR = 0U;
    tim->CNT = 0U;

    prescaler = timer_clock_hz / 1000000U;
    if (prescaler == 0U)
    {
        prescaler = 1U;
    }
    tim->PSC = prescaler - 1U;
    tim->ARR = 0xFFFFFFFFU;
    tim->EGR = TIM_EGR_UG;
    tim->CR1 &= ~TIM_CR1_OPM;

    tim->DIER &= ~TIM_DIER_UIE;

    st->tim = tim;
    st->is_us_timebase = 1U;
    us_timebase = tim;
}

void timer_start(TIM_TypeDef *tim)
{
    rcc_enable_timer(tim);
    tim->CR1 |= TIM_CR1_CEN;
}

void timer_stop(TIM_TypeDef *tim)
{
    tim->CR1 &= ~TIM_CR1_CEN;
}

void timer_reset(TIM_TypeDef *tim)
{
    tim->CNT = 0U;
    tim->EGR = TIM_EGR_UG;
    tim->SR = 0U;
}

void timer_irq_handler(TIM_TypeDef *tim)
{
    timer_state_t *st = find_state(tim);

    if ((st == 0) || ((tim->SR & TIM_SR_UIF) == 0U))
    {
        return;
    }

    tim->SR &= ~TIM_SR_UIF;

    if (st->callback != 0)
    {
        st->callback();
    }
}

uint32_t timer_get_us(void)
{
    if (us_timebase == 0)
    {
        return 0U;
    }

    return us_timebase->CNT;
}