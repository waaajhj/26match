/*-----------------------------------------------------

                    AD9834驱动程序
                    STM32HAL库
                    2019.10.16
-------------------------------------------------------*/
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//AD9834驱动代码	   
//作者：洁座
//创建日期:2019/10/16
//版本：V1.0
//版权所有，盗版必究。									  
//////////////////////////////////////////////////////////////////////////////////
#ifndef __AD9834_H
#define __AD9834_H

#include "sys.h"
#include "spi.h"
#define Triangle_Wave    0x2002
#define Sine_Wave  0x2000

/* AD9834晶振频率 */
#define AD9834_SYSTEM_COLCK     75000000UL

/* AD9834 控制引脚 */
#define AD9834_Control_Port     GPIOF
#define AD9834_FSYNC            GPIO_PIN_6
#define AD9834_RESET_SET            0x2100
#define AD9834_RESET_CLR            0x2000

#define AD9834_FSYNC_SET   HAL_GPIO_WritePin(AD9834_Control_Port ,AD9834_FSYNC,GPIO_PIN_SET)
#define AD9834_FSYNC_CLR   HAL_GPIO_WritePin(AD9834_Control_Port ,AD9834_FSYNC,GPIO_PIN_RESET)



#define FREQ_0      0
#define FREQ_1      1

/* AD9834函数声明 */
void AD9834_Write_16Bits(unsigned int data) ;
void AD9834_Select_Wave(unsigned int initdata) ;
void Init_AD9834(void) ;
void AD9834_Set_Freq(unsigned char freq_number, unsigned long freq) ;

#endif /* AD9834_H */
