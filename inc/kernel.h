#ifndef _OS_KERNEL_H
#define _OS_KERNEL_H
#include "stm32f401.h"

void KernelLaunch(uint32_t quanta);

void KernelInit(void);

uint8_t KernelAddThreads(void(*task0)(void),void(*task1)(void),void(*task2)(void));

void SemInit(int32_t *semaphore, int32_t value);

void ThreadYield(void);

void SemPost(int32_t *semaphore);

void SemWait(volatile int32_t *semaphore);													 
//#define PERIOD		100

#endif
													 						 