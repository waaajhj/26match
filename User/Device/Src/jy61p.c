#include "jy61p.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "maixcam.h"
#include "math.h"
static uint8_t RxBuffer[11];/*接收数据数组*/
static volatile uint8_t RxState = 0;/*接收状态标志位*/
static uint8_t RxIndex = 0;/*接受数组索引*/
float Roll,Pitch,Yaw,Yaw1,fYaw;/*角度信息，如果只需要整数可以改为整数类型*/
float LastYaw = 0, Circle;
float Lastyaw=0,F=0,quanshu=0,LastValidYaw=0;/*F是为了第一次不判断是否跳变，需要给Lastyaw赋初始值。quanshu记录圈数*/
// 蓝牙串口接收缓冲区
uint8_t rx_data[256] = {0};
int Start_Flag=0;
uint8_t  JY61P_ULOCK_CMD[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5}; //解锁
uint8_t  JY61P_BAUD_CMD [5] = {0xFF, 0xAA, 0x04, 0x06, 0x00}; //波特率修改为115200
uint8_t  JY61P_SAVE_CMD [5] = {0xFF, 0xAA, 0x00, 0x00, 0x00}; //保存
uint8_t  JY61P_XY0_CMD  [5] = {0xFF, 0xAA, 0x01, 0x08, 0x00}; //XY角度归零
uint8_t  JY61P_Z0_CMD   [5] = {0xFF, 0xAA, 0x01, 0x04, 0x00}; //Z轴归零

/*
 * 串口屏任务状态由 USART2 单字节接收中断更新。
 * 主循环处理 serial_screen_task_ready 后负责将其清零，避免重复执行同一任务。
 */
volatile SerialScreenTask_e serial_screen_task = SERIAL_SCREEN_TASK_NONE;
volatile uint8_t serial_screen_task_ready = 0U;
volatile uint32_t serial_screen_packet_count = 0U;
volatile uint32_t serial_screen_error_count = 0U;

typedef enum
{
    SERIAL_SCREEN_WAIT_HEADER = 0U,
    SERIAL_SCREEN_WAIT_TASK,
    SERIAL_SCREEN_WAIT_FOOTER
} SerialScreenRxState_e;

static SerialScreenRxState_e serial_screen_rx_state = SERIAL_SCREEN_WAIT_HEADER;
static uint8_t serial_screen_task_candidate = 0U;
//void Hal_Uart_Init(void){
//	HAL_UART_Receive_IT(&huart1, &RxData, 1);  // 接收1个字节数据
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_data, sizeof(rx_data));
//// 关闭DMA传输过半中断（HAL库默认开启，但我们只需要接收完成中断）
//    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
//	
//}
// 
/**
 * @brief       数据包处理函数
 * @param       串口接收的数据RxData
 * @retval      无
 */
void jy61p_ReceiveData(uint8_t RxData)
{
	uint8_t i,sum=0;
	
	if (RxState == 0)	//等待包头
	{
		if (RxData == 0x55)	//收到包头
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 1;
			RxIndex = 1; //进入下一状态
		}
	}
	
	else if (RxState == 1)
	{
		if (RxData == 0x53)	/*判断数据内容，修改这里可以改变要读的数据内容，0x53为角度输出*/
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 2;
			RxIndex = 2; //进入下一状态
		}
	}
	
	else if (RxState == 2)	//接收数据
	{
		RxBuffer[RxIndex++] = RxData;
		if(RxIndex == 11)	//接收完成
		{
			for(i=0;i<10;i++)
			{
				sum = sum + RxBuffer[i]; //计算校验和
			}
			if(sum == RxBuffer[10])		//校验成功
			{
				/*计算数据，根据数据内容选择对应的计算公式*/
				Roll = ((uint16_t) ((uint16_t) RxBuffer[3] << 8 | (uint16_t) RxBuffer[2])) / 32768.0f * 180.0f;
				Pitch = ((uint16_t) ((uint16_t) RxBuffer[5] << 8 | (uint16_t) RxBuffer[4])) / 32768.0f * 180.0f;
			//  Yaw = ((uint16_t) ((uint16_t) RxBuffer[7] << 8 | (uint16_t) RxBuffer[6])) / 32768.0f * 180.0f;
//				Yaw =  ((short)( RxBuffer[7]<<8 |  RxBuffer[6]))/32768.0*180;
//				if (Yaw < 0.0f) Yaw += 360.0f;
				//将角度转换成无跳变角度，防止跳变破坏稳定性
				// if(LastYaw - fYaw < -180){
				// 		Circle += 1;
				// }else if(LastYaw - fYaw > 180){
				// 		Circle -= 1;
				// }
				
				// Yaw = fYaw - 360 * Circle;
				// Yaw=(int)Yaw % 360;
				// LastYaw = fYaw;
					
			}
			RxState = 0;
			RxIndex = 0; //读取完成，回到最初状态，等待包头
		}
	}
}
/**
 * @brief 逐字节解析 USART2 串口屏任务帧。
 * @param rx_byte USART2 本次中断接收到的单字节数据。
 * @retval 无。
 * @note 调用前必须已初始化 USART2，并以单字节中断方式持续接收。
 *       帧格式固定为 0xAA + 任务号(1~5) + 0x55，共 3 字节。
 *       本函数不执行任务、不循环、不延时；任务 1 只更新状态，
 *       任务 2~5 会置 serial_screen_task_ready，交给主循环处理。
 *       非法任务号或错误包尾会被丢弃，状态机会等待下一个 0xAA 重新同步。
 */
void SerialScreen_ReceiveData(uint8_t rx_byte)
{
    switch (serial_screen_rx_state)
    {
    case SERIAL_SCREEN_WAIT_HEADER:
        if (rx_byte == 0xAAU)
        {
            serial_screen_rx_state = SERIAL_SCREEN_WAIT_TASK;
        }
        break;

    case SERIAL_SCREEN_WAIT_TASK:
        if ((rx_byte >= (uint8_t)SERIAL_SCREEN_TASK_1) &&
            (rx_byte <= (uint8_t)SERIAL_SCREEN_TASK_5))
        {
            serial_screen_task_candidate = rx_byte;
            serial_screen_rx_state = SERIAL_SCREEN_WAIT_FOOTER;
        }
        else if (rx_byte != 0xAAU)
        {
            /*
             * 连续收到 0xAA 时仍将最后一个 0xAA 视为新包头；
             * 其他非法任务号则丢弃当前半包。
             */
            serial_screen_error_count++;
            serial_screen_rx_state = SERIAL_SCREEN_WAIT_HEADER;
        }
        break;

    case SERIAL_SCREEN_WAIT_FOOTER:
        if (rx_byte == 0x55U)
        {
            serial_screen_task = (SerialScreenTask_e)serial_screen_task_candidate;
            serial_screen_packet_count++;
        }
        else
        {
            serial_screen_error_count++;

            /*
             * 错误包尾恰好为 0xAA 时，直接把它作为下一帧包头，
             * 可减少丢字节、错位后重新同步所需的时间。
             */
            serial_screen_rx_state =
                (rx_byte == 0xAAU) ? SERIAL_SCREEN_WAIT_TASK : SERIAL_SCREEN_WAIT_HEADER;
        }
        break;

    default:
        serial_screen_rx_state = SERIAL_SCREEN_WAIT_HEADER;
        break;
    }
}
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//    if (huart->Instance == USART2)
//    {
//		if(rx_data[0]=='A'){
//			Start_Flag=1;
//		}else if(rx_data[0]=='B'){
//			Start_Flag=2;
//	   }
//        // 使用DMA将接收到的数据发送回去
//        HAL_UART_Transmit_DMA(&huart2, rx_data, Size);
//        // 重新启动接收，使用Ex函数，接收不定长数据
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_data, sizeof(rx_data));
//        // 关闭DMA传输过半中断（HAL库默认开启，但我们只需要接收完成中断）
//        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
//    }
//}

 
///* 延时函数 */
//void delay_ms(uint32_t ms)  
//{  
//    uint32_t i, j;  
// 
//    for (i = ms; i > 0; i--)  
//    {  
// 
//        for (j = 800; j > 0; j--); // 这个值可能需要调整  
//    }  
//}  
/* IMU波特率修改函数 */
void JY61P_BAUD(void)
{
    HAL_UART_Transmit(&huart1,JY61P_ULOCK_CMD, sizeof(JY61P_ULOCK_CMD),10000);  //解锁
    HAL_Delay(200);//延时200ms
    HAL_UART_Transmit(&huart1,JY61P_BAUD_CMD, sizeof(JY61P_BAUD_CMD),10000);    //修改波特率为115200
    HAL_Delay(200);//延时200ms
   HAL_UART_Transmit(&huart1,JY61P_SAVE_CMD, sizeof(JY61P_SAVE_CMD),10000);    //保存
    HAL_Delay(200);//延时200ms
}
/* IMU归零函数 */
void JY61P_START(void)
{
    HAL_UART_Transmit(&huart3,JY61P_ULOCK_CMD, sizeof(JY61P_ULOCK_CMD),10000);  //解锁
    HAL_Delay(200);//延时200ms
    HAL_UART_Transmit(&huart3,JY61P_XY0_CMD, sizeof(JY61P_XY0_CMD),10000);    //XY轴归零
    HAL_Delay(200);
    HAL_UART_Transmit(&huart3,JY61P_Z0_CMD, sizeof(JY61P_Z0_CMD),10000);      //Z轴归零
    HAL_Delay(200);
    HAL_UART_Transmit(&huart3,JY61P_SAVE_CMD, sizeof(JY61P_SAVE_CMD),10000);    //保存
    HAL_Delay(200);
}

