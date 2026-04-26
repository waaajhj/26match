#include "jy61p.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "maixcam.h"
static uint8_t RxBuffer[11];/*接收数据数组*/
static volatile uint8_t RxState = 0;/*接收状态标志位*/
static uint8_t RxIndex = 0;/*接受数组索引*/
static uint8_t RxBuffer1[11];/*接收数据数组*/
static volatile uint8_t RxState1 = 0;/*接收状态标志位*/
static uint8_t RxIndex1 = 0;/*接受数组索引*/
float Roll,Pitch,Yaw,fYaw;/*角度信息，如果只需要整数可以改为整数类型*/
float LastYaw = 0, Circle;
// 蓝牙串口接收缓冲区
uint8_t rx_data[256] = {0};
int Start_Flag=0;
uint8_t  JY61P_ULOCK_CMD[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5}; //解锁
uint8_t  JY61P_BAUD_CMD [5] = {0xFF, 0xAA, 0x04, 0x06, 0x00}; //波特率修改为115200
uint8_t  JY61P_SAVE_CMD [5] = {0xFF, 0xAA, 0x00, 0x00, 0x00}; //保存
uint8_t  JY61P_XY0_CMD  [5] = {0xFF, 0xAA, 0x01, 0x08, 0x00}; //XY角度归零
 uint8_t  JY61P_Z0_CMD   [5] = {0xFF, 0xAA, 0x01, 0x04, 0x00}; //Z轴归零
uint8_t  WT101_Z0_CMD [5]={0xFF,0xAA,0x76,0x00,0x00};
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
 * @brief       数据包处理函数
 * @param       串口接收的数据RxData
 * @retval      无
 */
void WT101_ReceiveData(uint8_t RxData)
{
	uint8_t i,sum=0;
	
	if (RxState1 == 0)	//等待包头
	{
		if (RxData == 0x55)	//收到包头
		{
			RxBuffer1[RxIndex1] = RxData;
			RxState1 = 1;
			RxIndex1 = 1; //进入下一状态
		}
	}
	
	else if (RxState1 == 1)
	{
		if (RxData == 0x53)	/*判断数据内容，修改这里可以改变要读的数据内容，0x53为角度输出*/
		{
			RxBuffer1[RxIndex1] = RxData;
			RxState1 = 2;
			RxIndex1 = 2; //进入下一状态
		}
	}
	
	else if (RxState1 == 2)	//接收数据
	{
		RxBuffer1[RxIndex1++] = RxData;
		if(RxIndex1 == 11)	//接收完成
		{
			for(i=0;i<10;i++)
			{
				sum = sum + RxBuffer1[i]; //计算校验和
			}
			if(sum == RxBuffer1[10])		//校验成功
			{
				/*计算数据，根据数据内容选择对应的计算公式*/
			// 	Roll = ((uint16_t) ((uint16_t) RxBuffer[3] << 8 | (uint16_t) RxBuffer[2])) / 32768.0f * 180.0f;
			// 	Pitch = ((uint16_t) ((uint16_t) RxBuffer[5] << 8 | (uint16_t) RxBuffer[4])) / 32768.0f * 180.0f;
			//  Yaw = ((uint16_t) ((uint16_t) RxBuffer[7] << 8 | (uint16_t) RxBuffer[6])) / 32768.0f * 180.0f;
				Yaw =  ((short)( RxBuffer[7]<<8 |  RxBuffer[6]))/32768.0*180;
				if (Yaw < 0.0f) Yaw += 360.0f;
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
			RxState1 = 0;
			RxIndex1 = 0; //读取完成，回到最初状态，等待包头
		}
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
void WT101_START(void)
{
    HAL_UART_Transmit(&huart3,JY61P_ULOCK_CMD, sizeof(JY61P_ULOCK_CMD),10000);  //解锁
    HAL_Delay(200);//延时200ms
    HAL_UART_Transmit(&huart3,WT101_Z0_CMD, sizeof(WT101_Z0_CMD),10000);      //Z轴归零
    HAL_Delay(200);
    HAL_UART_Transmit(&huart3,JY61P_SAVE_CMD, sizeof(JY61P_SAVE_CMD),10000);    //保存
    HAL_Delay(200);
}

