#include <stdint.h>
#include <stdbool.h>

#include "timer.h"
#include "sched.h"
#include "mcu_pheripherial.h"

static bool cycle_counter_enabled = false;

static inline bool timer_is_privileged(void)
{
    return (__get_CONTROL() & 0x1U) == 0U;
}

void timer_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0U;
    cycle_counter_enabled = true;
}

uint32_t timer_get_ticks(void)
{
    return get_systick_counter();
}

uint32_t timer_get_ms(void)
{
    return timer_get_ticks();
}

uint32_t timer_get_us(void)
{
    if (cycle_counter_enabled && timer_is_privileged())
    {
        return (uint32_t)((uint64_t)DWT->CYCCNT * 1000000ULL / SystemCoreClock);
    }

    return timer_get_ms() * 1000U;
}

uint32_t timer_elapsed_ms(uint32_t start_ms)
{
    uint32_t now = timer_get_ms();
    return (now >= start_ms) ? (now - start_ms) : (UINT32_MAX - start_ms + now + 1U);
}

void timer_delay_us(uint32_t us)
{
    uint32_t start = timer_get_us();

    while ((timer_get_us() - start) < us)
    {
        __asm volatile ("nop");
    }
}

void timer_delay_ms(uint32_t ms)
{
    timer_delay_us(ms * 1000U);
}

bool timer_has_cycle_counter(void)
{
    return cycle_counter_enabled;
}
