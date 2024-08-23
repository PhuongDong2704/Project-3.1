#ifndef TX_H
#define TX_H

#include "stm32f4xx_hal.h"
typedef enum
{
	LedBlink,
	LedOff
}_LedState;

void Tx_Data_Led_Blink(uint8_t Tx[8],uint32_t time_blink);
#endif
