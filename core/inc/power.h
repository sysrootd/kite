#ifndef POWER_H
#define POWER_H

#include <stdint.h>
#include "config.h"

#if ENABLE_STOP_MODE
extern volatile uint8_t tim2_irq_fired;

void power_enter_stop_mode(uint32_t sleep_ticks);
void svc_enter_stop_mode(uint32_t sleep_ticks);
#endif

#endif
