#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <stdbool.h>

void timer_init(void);
uint32_t timer_get_ticks(void);
uint32_t timer_get_ms(void);
uint32_t timer_get_us(void);
uint32_t timer_elapsed_ms(uint32_t start_ms);
void     timer_delay_us(uint32_t us);
void     timer_delay_ms(uint32_t ms);
bool     timer_has_cycle_counter(void);

#endif // TIMER_H
