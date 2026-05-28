#ifndef BOARD_H
#define BOARD_H

#include "gpio.h"
#include "uart.h"
#include "sched.h"
#include "adc.h"
#include "lcd.h"

#define LM35       0U
#define UP_SWITCH  8U
#define DN_SWITCH  9U
#define BUZZER     12U
#define RED_LED    13U
#define GREEN_LED  14U
#define BAUD_RATE  115200U

#define LM35_VREF_MV    1200U   
#define LM35_ADC_MAX    4095U   
#define LM35_MV_PER_DEG   10U 

void board_init(void);

#endif