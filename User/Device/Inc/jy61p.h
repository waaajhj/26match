#ifndef __JY61P_H
#define __JY61P_H

#include "stm32f4xx_hal.h"

/*
 * USART2 串口屏任务帧格式：0xAA + 任务号 + 0x55，共 3 字节。
 * 任务号有效范围为 1~5；任务 1 只记录状态，不置执行标志，
 * 任务 2~5 收到完整有效帧后置 serial_screen_task_ready。
 */
typedef enum
{
    SERIAL_SCREEN_TASK_NONE = 0U,
    SERIAL_SCREEN_TASK_1 = 1U,
    SERIAL_SCREEN_TASK_2 = 2U,
    SERIAL_SCREEN_TASK_3 = 3U,
    SERIAL_SCREEN_TASK_4 = 4U,
    SERIAL_SCREEN_TASK_5 = 5U
} SerialScreenTask_e;

void Hal_Uart_Init(void);
void jy61p_ReceiveData(uint8_t RxData);
void SerialScreen_ReceiveData(uint8_t rx_byte);
void BT24_ReceiveData(uint8_t RxData);
//void delay_ms(uint32_t ms);
void JY61P_BAUD(void);
void JY61P_START(void);

/*
 * 以下变量在 USART2 接收完成中断中更新，在主循环或任务函数中读取。
 * 业务代码处理完任务后，应将 serial_screen_task_ready 清零。
 */
extern volatile SerialScreenTask_e serial_screen_task;
extern volatile uint8_t serial_screen_task_ready;
extern volatile uint32_t serial_screen_packet_count;
extern volatile uint32_t serial_screen_error_count;

extern float Roll,Pitch,Yaw;
extern int Start_Flag;
#endif
