#include "Rx.h"

CAN_RxHeaderTypeDef RxHeader;
uint8_t Rx[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,Rx);
}