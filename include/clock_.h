#ifndef CLOCK__H
#define CLOCK__H

#include "stm32l476xx.h"
#include <stdint.h>

#define SYSCLK_FREQ 80000000u
#define AHB_FREQ    80000000u
#define APB1_FREQ   80000000u
#define APB2_FREQ   80000000u

#define BAUDRATE    9600u

void SystemClock_Config_80MHz(void);
void Init_Debug_UART(void);
void delay(uint32_t time);

#endif