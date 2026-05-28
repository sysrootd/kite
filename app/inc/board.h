#ifndef BOARD_H
#define BOARD_H

#include "gpio.h"
#include "uart.h"
#include "adc.h"
#include "lcd.h"
#include "sched.h"

#define LM35       0U
#define UP_SWITCH  8U
#define DN_SWITCH  9U
#define BUZZER     12U
#define RED_LED    13U
#define GREEN_LED  14U
#define BAUD_RATE  115200U

void board_init(void);

#endif