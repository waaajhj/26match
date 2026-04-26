#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

#include "stdint.h"

// 电机转速滤波系数
#define DM_OMEGA_LPF_RC 0.15f

#define P_MIN -12.5663706144f
#define P_MAX 12.5663706144f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

/* 电机控制模式 */
typedef enum
{
    MIT_MODE = 0x000,
    POS_MODE = 0x100,
    SPEED_MODE = 0x200,
} MotorControl_e;

/* 电机状态 */
typedef enum
{
    MOTOR_DISABLE = 0x00, // 电机失能
    MOTOR_ENABLE = 0x01,  // 电机使能
} MotorMode_e;

/* 达妙电机控制帧ID(CAN ID/Slave ID) */
typedef enum
{
    DM_YAW_TX_ID = 0x01,
    DM_PITCH_TX_ID = 0x02,
} DM_Motor_TX_ID_e;

/* 达妙电机反馈帧ID(Master ID) */
typedef enum
{
    DM_YAW_RX_ID = 0x11,
    DM_PITCH_RX_ID = 0x12,
} DM_Motor_RX_ID_e;

/* 关节电机数据 */
typedef struct
{
    /* 原始值 */
    struct
    {
        uint8_t ID;     // 电机线圈温度
        uint8_t State;  // 错误状态
        float PosTemp;  // 电机位置(P_MIN,P_MAX)
        float Velocity; // 电机转速
        uint8_t T_Mos;
        uint8_t T_Coil;
    } measure;
    float LastPosTemp; // 上次电机位置(P_MIN,P_MAX)
    float LastOmega;   // 电机输出轴上次角速度(rad/s)
    /* 解析后的反馈值 */
    float Position; // 输出轴角度(-PI,PI)(rad)
    float Omega;    // 电机输出轴角速度(rad/s)
    float Torque;   // 反馈扭矩
} DM_Motor_t;

void DMMotorDecode(DM_Motor_t *Motor, uint8_t *RxData);
void DMMotorEnable(DM_Motor_TX_ID_e MotorID, MotorControl_e Mode);
void DMMotorDisable(DM_Motor_TX_ID_e MotorID, MotorControl_e Mode);
void DMMotorZeroSet(DM_Motor_TX_ID_e MotorID, MotorControl_e Mode);
void DMMotorClearErrors(DM_Motor_TX_ID_e MotorID, MotorControl_e Mode);
void MitControl(DM_Motor_TX_ID_e MotorID, float Pos, float Vel, float Kp, float Kd, float Tor);
void DM_MitControl(DM_Motor_TX_ID_e MotorID, MotorMode_e State, float Pos, float Vel, float Kp, float Kd, float Tor);

#endif
