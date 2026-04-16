#include <stdint.h>

#include "stats.h"
#include "uart.h"
#include "timer.h"

static volatile uint32_t total_ticks = 0U;
static volatile uint32_t idle_ticks = 0U;
static volatile uint32_t context_switches = 0U;

void stats_init(void)
{
    total_ticks = 0U;
    idle_ticks = 0U;
    context_switches = 0U;
}

void stats_tick(void)
{
    total_ticks++;
}

void stats_idle_enter(void)
{
    idle_ticks++;
}

void stats_context_switch(void)
{
    context_switches++;
}

void stats_report(USART_TypeDef *uart)
{
    uint32_t uptime_ms = timer_get_ms();
    uint32_t idle_pct = 0U;

    if (total_ticks > 0U)
    {
        idle_pct = (idle_ticks * 100U) / total_ticks;
    }

    uart_printf(uart,
        "[STATS] uptime=%lu ms idle=%lu/%lu (%lu%%) switches=%lu dwt=%u\n\r",
        (unsigned long)uptime_ms,
        (unsigned long)idle_ticks,
        (unsigned long)total_ticks,
        (unsigned long)idle_pct,
        (unsigned long)context_switches,
        timer_has_cycle_counter() ? 1U : 0U);
}

void stats_reset(void)
{
    total_ticks = 0U;
    idle_ticks = 0U;
    context_switches = 0U;
}

uint32_t stats_get_total_ticks(void)
{
    return total_ticks;
}

uint32_t stats_get_idle_ticks(void)
{
    return idle_ticks;
}

uint32_t stats_get_context_switches(void)
{
    return context_switches;
}
