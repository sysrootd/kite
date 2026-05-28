#ifndef BOARD_H
#define BOARD_H

#include "gpio.h"
#include "uart.h"
#include "adc.h"
#include "lcd.h"
#include "sched.h"

#define LM35_PIN   0U
#define UP_SWITCH_PIN   8U
#define DN_SWITCH_PIN   9U
#define BUZZER_PIN  12U
#define RED_LED_PIN 13U
#define GREEN_LED_PIN   14U

#define LM35       LM35_PIN
#define UP_SWITCH  UP_SWITCH_PIN
#define DN_SWITCH  DN_SWITCH_PIN
#define BUZZER     BUZZER_PIN
#define RED_LED    RED_LED_PIN
#define GREEN_LED  GREEN_LED_PIN

#define BAUD_RATE  115200U

void board_init(void);

#endif