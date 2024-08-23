#include "Tx.h"

void Tx_Data_Led_Blink(uint8_t Tx[8],uint32_t time_blink)
{
		Tx[1] = (uint8_t)(time_blink & 0xFF);
		Tx[2] = (uint8_t)(time_blink >> 8 & 0xFF);
		Tx[3] = (uint8_t)(time_blink >> 16 & 0xFF);
		Tx[4] = (uint8_t)(time_blink >> 24 & 0xFF);
}