#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stdint.h"
#include "can.h"
#include "DM_Motor.h"

#define CHASSIS_CAN hcan1 // 底盘电机CAN

// M2006电机轴总编码器转换到输出轴角度(rad)的比例=1/36/8192*2*PI
#define M2006_CONVERSION_RATIO 2.130528872063391e-5f
// M3508电机轴总编码器转换到输出轴角度(rad)的比例=187/3591/8192*2*PI
#define M3508_CONVERSION_RATIO 3.994074176199038e-5f
// M6020电机轴总编码器转换到输出轴角度(rad)的比例=2*PI/8192
#define M6020_CONVERSION_RATIO 7.669903939428206e-4f
// M3508转子转速(rpm)转换为输出轴角速度(rad/s)的比例, 2*PI*187/3591/60
#define M3508_MOTOR_RPM2OMG_SEN 0.0054532426f
// M2006转子转速(rpm)转换为输出轴角速度(rad/s)的比例, 2*PI*1/36/60
#define M2006_MOTOR_RPM2OMG_SEN 0.0029088821f
// 无减速箱电机转子转速(rpm)转换为输出轴角速度(rad/s)的比例, 2*PI/60
#define MOTOR_RPM2OMG_SEN 0.10471975512f
/* 底盘CAN收发ID */
typedef enum
{
	CAN_CHASSIS_ALL_ID = 0x200, // 控制所有底盘电机的ID
	CAN_CHASSIS_M1_ID = 0x201,
	CAN_CHASSIS_M2_ID = 0x202,
	CAN_CHASSIS_M3_ID = 0x203,
	CAN_CHASSIS_M4_ID = 0x204,
} chassis_can_id_e;

/* 云台CAN收发ID */
typedef enum
{
	CAN_GIMBAL_ALL_ID = 0x1FF, // 0x1FE电流控, 0x1FF电压控
	CAN_GIMBAL_YAW_ID = 0x205,
	CAN_GIMBAL_PITCH_ID = 0x206,
} gimbal_can_id_e;

/* 发射CAN收发ID */
typedef enum
{
	CAN_SHOOT_ALL_ID = 0x200,	  // 控制所有发射机构电机的ID
	CAN_SHOOT_FRIC_M1_ID = 0x201, // 摩擦轮1电机ID
	CAN_SHOOT_FRIC_M2_ID = 0x202, // 摩擦轮2电机ID
	CAN_SHOOT_TRIGGER_ID = 0x203, // 拨弹电机ID
} shoot_can_id_e;

/* 电机类型, 用于不同电机的输出轴角度获取 */
typedef enum
{
	M2006 = 0x00,
	M3508 = 0x01,
	M6020 = 0x02,
} DJI_motor_type_e;
typedef enum
{
    DJI1_Motor_Type = 0x00,
    DM_Motor_Type = 0x01,
} MotorType_e;

/*3508电机的数据结构体*/
typedef struct
{
	/*----------------------原始值----------------------*/
	uint16_t ecd;		   // 编码器值(Encoder)，范围0~8191
	int16_t speed_rpm;	   // 电机转速，单位RPM
	int16_t given_current; // 转矩电流
	uint8_t temperate;	   // 电机温度
						   /*-----------------输出轴相对角度值-----------------*/
	uint8_t init_cnt;	   // 初始化的计数值
	uint16_t last_ecd;	   // 上一次的编码器值
	uint16_t offset_ecd;   // 电机校准值
	int32_t round_cnt;	   // 电机屁股圈数计数
	int32_t total_angle;   // 电机屁股实际转动角度(编码值)
	float shaft_angle;	   // 输出轴角度(rad)
} motor_measure_t;
extern motor_measure_t chassis_motor[4];
void CAN_Config(void);
void CANTransmit(MotorType_e Motor, uint16_t ID, uint8_t *TxData);
void CANTransmitWithDLC(MotorType_e Motor,
                        uint16_t ID,
                        uint8_t *TxData,
                        uint8_t DataLength);
void TriggerMotorCalibrate(void);
uint8_t GetMotorState(DM_Motor_TX_ID_e DMMotorID);
/*---------------电机电压电流控制函数---------------*/
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch);
void CAN_cmd_shoot(int16_t fric1, int16_t fric2, int16_t trigger);
/*----------------获取电机数据结构体----------------*/
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
const DM_Motor_t *get_gimbal_motor_measure_point(uint8_t i);
//const motor_measure_t *get_gimbal_yaw_motor_measure_point(void);
//const motor_measure_t *get_gimbal_pitch_motor_measure_point(void);
//const motor_measure_t *get_trigger_motor_measure_point(void);
//const motor_measure_t *get_fric_motor_measure_point(uint8_t i);

#endif
