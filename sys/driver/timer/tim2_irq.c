#include "mcu.h"
#include "timer.h"

#if ENABLE_STOP_MODE
extern volatile uint8_t tim2_irq_fired;

void TIM2_IRQHandler(void)
{
    tim2_irq_fired = 1U;
    timer_irq_handler(TIM2);
}
#else

void TIM2_IRQHandler(void)
{
    timer_irq_handler(TIM2);
}

#endif
