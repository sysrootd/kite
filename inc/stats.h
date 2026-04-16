#ifndef STATS_H
#define STATS_H

#include <stdint.h>
#include "mcu_pheripherial.h"

void stats_init(void);
void stats_tick(void);
void stats_idle_enter(void);
void stats_context_switch(void);
void stats_report(USART_TypeDef *uart);
void stats_reset(void);

uint32_t stats_get_total_ticks(void);
uint32_t stats_get_idle_ticks(void);
uint32_t stats_get_context_switches(void);

#endif // STATS_H
