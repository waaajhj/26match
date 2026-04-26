#ifndef __JY61P_H
#define __JY61P_H

#include "stm32f4xx_hal.h"
void Hal_Uart_Init(void);
void jy61p_ReceiveData(uint8_t RxData);
void BT24_ReceiveData(uint8_t RxData);
void delay_ms(uint32_t ms);
void JY61P_BAUD(void);
void JY61P_START(void);
extern float Roll,Pitch,Yaw;
extern int Start_Flag;
#endif
