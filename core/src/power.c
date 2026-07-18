#include <stdint.h>

#include "mcu.h"
#include "sched.h"
#include "power.h"

#if ENABLE_STOP_MODE
volatile uint8_t tim2_irq_fired = 0U;

#define TIM_CR1_CEN   (1U << 0)
#define TIM_CR1_OPM   (1U << 3)
#define TIM_DIER_UIE  (1U << 0)
#define TIM_SR_UIF    (1U << 0)

void power_enter_stop_mode(uint32_t sleep_ticks)
{
    uint32_t saved_systick = global_systick;
    uint32_t target_systick = saved_systick + sleep_ticks;

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_CWUF;
    PWR->CR &= ~PWR_CR_PDDS;
    PWR->CR &= ~PWR_CR_LPDS;

    uint64_t us = (uint64_t)sleep_ticks * 1000000ULL / (uint64_t)TICK_HZ;
    if (us == 0U)
    {
        us = 1U;
    }

    tim2_irq_fired = 0U;

    RCC->APB1ENR |= (1U << 0);

    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->PSC = (SystemCoreClock / 1000000U) - 1U;
    TIM2->ARR = (uint32_t)(us - 1ULL);
    TIM2->CNT = 0U;
    TIM2->EGR = 1U;
    TIM2->SR = 0U;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_OPM;
    TIM2->CR1 |= TIM_CR1_CEN;

    NVIC_SetPriority(TIM2_IRQn, KERNEL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(TIM2_IRQn);

    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

    __DSB();
    __ISB();
    __WFI();

    SystemInit();
    SysTick_Config(SystemCoreClock / TICK_HZ);

    RCC->APB1ENR |= (1U << 0);
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->DIER &= ~TIM_DIER_UIE;
    TIM2->SR &= ~TIM_SR_UIF;
    NVIC_DisableIRQ(TIM2_IRQn);

    if (tim2_irq_fired)
    {
        global_systick = target_systick;
    }
    else
    {
        uint32_t elapsed_us = TIM2->CNT;
        uint32_t adv = (uint32_t)(((uint64_t)elapsed_us * (uint64_t)TICK_HZ + 500000ULL) / 1000000ULL);
        if (adv == 0U)
        {
            adv = 1U;
        }
        global_systick = saved_systick + adv;
    }
}


void enter_stop_mode(uint32_t sleep_ticks)
{
    register uint32_t r0 __asm("r0") = sleep_ticks;
    __asm volatile ("svc 8" : : "r"(r0) : "memory");
}

void svc_enter_stop_mode(uint32_t sleep_ticks)
{
    power_enter_stop_mode(sleep_ticks);
}

#endif
