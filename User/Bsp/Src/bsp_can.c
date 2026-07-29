/**
 *************************(C) COPYRIGHT 2024 DragonBot*************************
 * @file       bsp_can.c/h
 * @brief
 * @note
 * @history
 * Date            Author          Modification
 * 2023-10-26      wanghongxi      代码框架
 * 2024-08-30      ZouAjie	       优化
 @verbatim
 ==============================================================================

 ==============================================================================
 @endverbatim
 *************************(C) COPYRIGHT 2024 DragonBot*************************
 */
#include "bsp_can.h"
#include "user_lib.h"
#include "DM_Motor.h"
/*---------------------------------------------------------------------------------------------------*/
static void get_motor_measure(motor_measure_t *ptr, uint8_t *data);
static void get_shaft_angle(motor_measure_t *ptr, DJI_motor_type_e motor);
/*---------------------------------------------------------------------------------------------------*/
motor_measure_t chassis_motor[4] = {0}; // 底盘2006电机数据[0:3]
static CAN_TxHeaderTypeDef chassis_tx_message; // 底盘CAN发送结构体
static uint8_t chassis_can_send_data[8];	   // 底盘CAN发送数据
DM_Motor_t Gimbal_Motor[2] = {0};   // 达妙Yaw轴电机数据
DM_Motor_t Chassis_Motor[2] = {0}; // 达妙底盘电机数据

// 记录 CAN 发送邮箱占满或 HAL 发送失败的次数，便于在 Keil Watch 中排查丢帧。
volatile uint32_t can_tx_enqueue_error_count = 0U;
/*---------------------------------------------------------------------------------------------------*/
/**
 * @brief 发送指定数据长度的标准 CAN 数据帧。
 * @param Motor 电机类型
 * @param ID CAN报文ID
 * @param TxData 发送数据
 * @param DataLength CAN 数据长度，经典 CAN 有效范围为 1~8 字节
 * @note 本函数不阻塞等待报文发送完成；无效数据长度会直接放弃发送。
 */
void CANTransmitWithDLC(MotorType_e Motor,
                        uint16_t ID,
                        uint8_t *TxData,
                        uint8_t DataLength)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint32_t interrupt_mask;
    HAL_StatusTypeDef transmit_status;

    if (DataLength == 0U || DataLength > 8U)
    {
        return;
    }

    // 配置CAN报文头
    TxHeader.StdId = ID;                  // 标准ID
    TxHeader.ExtId = 0x00;                // 扩展ID（未使用）
    TxHeader.IDE = CAN_ID_STD;            // 标准帧
    TxHeader.RTR = CAN_RTR_DATA;          // 数据帧
    TxHeader.DLC = DataLength;
    TxHeader.TransmitGlobalTime = DISABLE;

    // 根据电机类型选择对应的CAN句柄
    CAN_HandleTypeDef *hcan;
    switch (Motor)
    {
    case DJI1_Motor_Type:
        hcan = &hcan2;
        break;
    case DM_Motor_Type:
        hcan = &hcan1;
        break;       
    default:
        return; // 无效电机类型，直接返回
    }

    /*
     * 底盘指令在主循环发送，球杆指令在 TIM4 中断发送。
     * 短暂关闭中断可防止两个上下文同时进入 HAL_CAN_AddTxMessage()
     * 并选择到同一个发送邮箱；完成寄存器写入后立即恢复原中断状态。
     */
    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    transmit_status = HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    if (transmit_status != HAL_OK)
    {
        can_tx_enqueue_error_count++;
    }
    __set_PRIMASK(interrupt_mask);
}

/**
 * @brief 发送原工程使用的 8 字节标准 CAN 数据帧。
 * @note 保留该接口以兼容现有电机控制代码。
 */
void CANTransmit(MotorType_e Motor, uint16_t ID, uint8_t *TxData)
{
    CANTransmitWithDLC(Motor, ID, TxData, 8U);
}
/**
 * @brief  CAN过滤器初始化
 * @param  None
 */
void CAN_Config(void)
{
	CAN_FilterTypeDef can_filter_st;								   // CAN滤波器结构体
	can_filter_st.FilterActivation = ENABLE;						   // 使能滤波器
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;				   // ID屏蔽模式
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;				   // 32位滤波器
	can_filter_st.FilterIdHigh = 0x0000;							   // 32位ID
	can_filter_st.FilterIdLow = 0x0000;								   // 32位ID
	can_filter_st.FilterMaskIdHigh = 0x0000;						   // 32位屏蔽ID
	can_filter_st.FilterMaskIdLow = 0x0000;							   // 32位屏蔽ID
	can_filter_st.FilterBank = 0;									   // 过滤器组0
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;				   // 过滤器组0关联到FIFO0
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);					   // 配置过滤器
	HAL_CAN_Start(&hcan1);											   // 开启CAN1
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 开启中断
	can_filter_st.SlaveStartFilterBank = 14;						   // 从过滤器组14开始
	can_filter_st.FilterBank = 14;									   // 过滤器组14
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);					   // 配置过滤器
	HAL_CAN_Start(&hcan2);											   // 开启CAN2
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // 开启中断
}

/**
 * @brief      hal库CAN回调函数,接收电机数据,并将数据解包后存入相应数组
 * @param[in]  hcan:CAN句柄指针
 * @retval     none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHead;
	uint8_t Rxdata[8];
	if (hcan == &hcan2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHead, Rxdata);
		switch (RxHead.StdId) // 根据StdID筛选出底盘M2006电机
		{
		case CAN_CHASSIS_M1_ID:
		case CAN_CHASSIS_M2_ID:
		case CAN_CHASSIS_M3_ID:
		case CAN_CHASSIS_M4_ID:
		{
			static uint8_t i = 0;
			i = RxHead.StdId - CAN_CHASSIS_M1_ID;		  // 获取电机ID
			get_motor_measure(&chassis_motor[i], Rxdata); // 获取电机原始数据
			get_shaft_angle(&chassis_motor[i], M2006);	  // 获取输出轴角度
			break;
		}
		default:
			break;
		}
	}else if(hcan == &hcan1){
		 HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHead, Rxdata);
        switch (RxHead.StdId)
        {
        case DM_YAW_RX_ID:
            DMMotorDecode(&Gimbal_Motor[0], Rxdata);
            break;
		case DM_PITCH_RX_ID:
            DMMotorDecode(&Gimbal_Motor[1], Rxdata);
            break;
		case DM_Chassis1_RX_ID:
			DMMotorDecode(&Chassis_Motor[0], Rxdata);
			break;
		case DM_Chassis2_RX_ID:
			DMMotorDecode(&Chassis_Motor[1], Rxdata);
			break;
        default:
            break;
        }
	}
}

/**
 * @brief      解包电机数据
 * @param[in]  *ptr:电机数据地址
 * @param[in]  *data:原始数据
 * @retval     none
 */
static void get_motor_measure(motor_measure_t *ptr, uint8_t *data)
{
	ptr->last_ecd = ptr->ecd;
	ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);
	ptr->speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
	ptr->given_current = (uint16_t)(data[4] << 8 | data[5]);
	ptr->temperate = data[6];
}

/**
 * @brief      电机输出轴角度换算
 * @param[in]  *ptr: 电机数据地址
 * @param[in]  motor: 电机类型
 * @retval     none
 */
static void get_shaft_angle(motor_measure_t *ptr, DJI_motor_type_e motor)
{
	// 上电获取编码器补偿量
	if (ptr->init_cnt < 10)
	{
		ptr->offset_ecd = ptr->ecd;
		ptr->init_cnt++;
	}
	else
	{
		switch (motor) // 根据不同电机类型计算输出轴角度
		{
		case M2006:
			// 记电机轴圈数
			if (ptr->ecd - ptr->last_ecd > 4096)
				ptr->round_cnt--;
			else if (ptr->ecd - ptr->last_ecd < -4096)
				ptr->round_cnt++;
			if (ptr->round_cnt == 36 || ptr->round_cnt == -36)
				ptr->round_cnt = 0;
			// 算当前电机轴总编码器值
			ptr->total_angle = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
			// 输出轴角度 = -PI到PI的循环限幅(电机轴总编码器值 * 转换比)
			ptr->shaft_angle = rad_format(ptr->total_angle * M2006_CONVERSION_RATIO);
			break;
		case M3508:
			// 记电机轴圈数
			if (ptr->ecd - ptr->last_ecd > 4096)
				ptr->round_cnt--;
			else if (ptr->ecd - ptr->last_ecd < -4096)
				ptr->round_cnt++;
			if (ptr->round_cnt == 3591 || ptr->round_cnt == -3591)
				ptr->round_cnt = 0;
			// 算当前电机轴总编码器值
			ptr->total_angle = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
			// 输出轴角度 = -PI到PI的循环限幅(电机轴总编码器值 * 转换比)
			ptr->shaft_angle = rad_format(ptr->total_angle * M3508_CONVERSION_RATIO);
			break;
		case M6020:
			ptr->shaft_angle = rad_format(ptr->ecd * M6020_CONVERSION_RATIO);
			break;
		}
	}
}

/**
 * @brief      发送底盘电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]  motor1: (0x201) 2006电机控制电流, 范围 [-10000,10000]
 * @param[in]  motor2: (0x202) 2006电机控制电流, 范围 [-10000,10000]
 * @param[in]  motor3: (0x203) 2006电机控制电流, 范围 [-10000,10000]
 * @param[in]  motor4: (0x204) 2006电机控制电流, 范围 [-10000,10000]
 * @retval     none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box;						   // 发送邮箱
	chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID; // 发送ID
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = motor1 >> 8;
	chassis_can_send_data[1] = motor1;
	chassis_can_send_data[2] = motor2 >> 8;
	chassis_can_send_data[3] = motor2;
	chassis_can_send_data[4] = motor3 >> 8;
	chassis_can_send_data[5] = motor3;
	chassis_can_send_data[6] = motor4 >> 8;
	chassis_can_send_data[7] = motor4;
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
/* 获取达妙电机状态, 1使能, 0失能及其他状态 */
uint8_t GetMotorState(DM_Motor_TX_ID_e DMMotorID)
{
    switch (DMMotorID)
    {
    case DM_YAW_TX_ID:
        return Gimbal_Motor[0].measure.State;
    case DM_PITCH_TX_ID:
        return Gimbal_Motor[1].measure.State;
    case DM_Chassis1_TX_ID:
        return Chassis_Motor[0].measure.State;
    case DM_Chassis2_TX_ID:
        return Chassis_Motor[1].measure.State;
    default:
        break;
    }
    return 0;
}
/**
 * @brief      返回云台4310电机数据指针
 * @param[in]  i: 电机编号,范围[0,1]
 * @retval     云台电机数据指针
 */
const DM_Motor_t *get_gimbal_motor_measure_point(uint8_t i)
{
    return &Gimbal_Motor[(i & 0x01)];
}


/**
 * @brief      返回底盘2006电机数据指针
 * @param[in]  i: 电机编号,范围[0,3]
 * @retval     底盘电机数据指针
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
	return &chassis_motor[(i & 0x03)];
}


