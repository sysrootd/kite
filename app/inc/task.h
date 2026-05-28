#ifndef TASK_H
#define TASK_H

#include "board.h"
#include "sched.h"

#define LM35_VREF_MV    1200U   
#define LM35_ADC_MAX    4095U   
#define LM35_MV_PER_DEG   10U 

#define SAMPLE_DELAY    5000U 

void tasks_init(void);

#endif