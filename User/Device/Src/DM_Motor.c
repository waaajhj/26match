/**
 *************************(C) COPYRIGHT 2025 DragonBot*************************
 * @file	DM_Motor.c/h
 * @brief	达妙电机反馈数据解析函数与力矩控制函数
 * @history
 * Date            Author          Modification
 * 2024-8-30       ZouAjie	       代码框架
 * @attention
 ==============================================================================
 使用MIT模式, 除转矩外全为0, 达到力矩控制效果
 达妙电机的数据解析在反馈帧ID(Master ID)下
 控制达妙电机应使用控制器ID(CAN ID), 最终发送的ID要根据模式加偏移值
 ==============================================================================
 *************************(C) COPYRIGHT 2025 DragonBot*************************
*/
#include "DM_Motor.h"
#include "bsp_can.h"
#include "user_lib.h"

/* Converts a float to an unsigned int, given range and number of bits */
static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/* converts unsigned int to float, given range and number of bits */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 解析DM8009电机反馈值
 * @param Motor 指向Joint_Motor_t结构体的指针，包含电机相关信息和反馈数据
 * @param RxData 指向包含反馈数据的数组指针
 **/
void DMMotorDecode(DM_Motor_t *Motor, uint8_t *RxData)
{
    /*---------------------------------解析原始值---------------------------------*/
    int p_int, v_int, t_int;
    Motor->measure.ID = (RxData[0]) & 0xF;
    Motor->measure.State = (RxData[0]) >> 4;
    p_int = (RxData[1] << 8) | RxData[2];
    v_int = (RxData[3] << 4) | (RxData[4] >> 4);
    t_int = ((RxData[4] & 0xF) << 8) | RxData[5];
    Motor->measure.PosTemp = uint_to_float(p_int, P_MIN, P_MAX, 16);
    Motor->measure.Velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);
    Motor->Torque = uint_to_float(t_int, T_MIN, T_MAX, 12);
    Motor->measure.T_Mos = RxData[6];
    Motor->measure.T_Coil = RxData[7];
    Motor->Omega = DM_OMEGA_LPF_RC * Motor->LastOmega + (1.0f - DM_OMEGA_LPF_RC) * Motor->measure.Velocity;
    Motor->LastOmega = Motor->Omega;
    /*-------------------------------解析输出轴角度-------------------------------*/
    Motor->Position = rad_format(Motor->measure.PosTemp);
}

/**
 * @brief 达妙电机使能函数
 * @param MotorID 电机ID，指定目标电机
 **/
void DMMotorEnable(DM_Motor_TX_ID_e MotorID, MotorControl_e Mode)
{
    uint8_t TxData[8];
    uint16_t ID = MotorID + Mode;
    TxData[0] = 0xFF;
    TxData[1] = 0xFF;
    TxData[2] = 0xFF;
    TxData[3] = 0xFF;
    TxData[4] = 0xFF;
    TxData[5] = 0xFF;
    TxData[6] = 0xFF;
    TxData[7] = 0xFC;
    CANTransmit(DM_Motor_Type, ID, TxData);
}

/**
 * @brief 达妙电机失能函数
 * @param MotorID 电机ID，指定目标电机
 **/
void DMMotorDisable(DM_Motor_TX_ID_e MotorID, MotorControl_e Mode)
{
    uint8_t TxData[8];
    uint16_t ID = MotorID + Mode;
    TxData[0] = 0xFF;
    TxData[1] = 0xFF;
    TxData[2] = 0xFF;
    TxData[3] = 0xFF;
    TxData[4] = 0xFF;
    TxData[5] = 0xFF;
    TxData[6] = 0xFF;
    TxData[7] = 0xFD;
    CANTransmit(DM_Motor_Type, ID, TxData);
}

/**
 * @brief 达妙电机保存零点函数
 * @param MotorID 电机ID
 **/
void DMMotorZeroSet(DM_Motor_TX_ID_e MotorID, MotorControl_e Mode)
{
    uint8_t TxData[8];
    uint16_t ID = MotorID + Mode;
    TxData[0] = 0xFF;
    TxData[1] = 0xFF;
    TxData[2] = 0xFF;
    TxData[3] = 0xFF;
    TxData[4] = 0xFF;
    TxData[5] = 0xFF;
    TxData[6] = 0xFF;
    TxData[7] = 0xFE;
    CANTransmit(DM_Motor_Type, ID, TxData);
}

/**
 * @brief 达妙电机清除错误
 * @param MotorID 电机ID
 **/
void DMMotorClearErrors(DM_Motor_TX_ID_e MotorID, MotorControl_e Mode)
{
    uint8_t TxData[8];
    uint16_t ID = MotorID + Mode;
    TxData[0] = 0xFF;
    TxData[1] = 0xFF;
    TxData[2] = 0xFF;
    TxData[3] = 0xFF;
    TxData[4] = 0xFF;
    TxData[5] = 0xFF;
    TxData[6] = 0xFF;
    TxData[7] = 0xFB;
    CANTransmit(DM_Motor_Type, ID, TxData);
}

/**
 * @brief 达妙电机Mit模式控制函数
 * @note Kp=0, Kd!=0, Vel可直接控电机角速度
         Kp!=0, Kd!=0, Pos可直接控电机位置
         Kp=0, Kd=0, Tor可实现纯力矩控制
 * @param MotorID 电机ID，指定目标电机
 * @param Pos 位置设定值(rad)
 * @param Vel 速度设定值(rad/s)
 * @param Kp 位置比例系数
 * @param Kd 位置微分系数
 * @param Tor 力矩设定值(NM)
 **/
void MitControl(DM_Motor_TX_ID_e MotorID, float Pos, float Vel, float Kp, float Kd, float Tor)
{
    uint8_t TxData[8];
    uint16_t PosTmp, VelTmp, KpTmp, KdTmp, TorTmp;
    uint16_t ID = MotorID + MIT_MODE;

    PosTmp = float_to_uint(Pos, P_MIN, P_MAX, 16);
    VelTmp = float_to_uint(Vel, V_MIN, V_MAX, 12);
    KpTmp = float_to_uint(Kp, KP_MIN, KP_MAX, 12);
    KdTmp = float_to_uint(Kd, KD_MIN, KD_MAX, 12);
    TorTmp = float_to_uint(Tor, T_MIN, T_MAX, 12);

    TxData[0] = (PosTmp >> 8);
    TxData[1] = PosTmp;
    TxData[2] = (VelTmp >> 4);
    TxData[3] = ((VelTmp & 0xF) << 4) | (KpTmp >> 8);
    TxData[4] = KpTmp;
    TxData[5] = (KdTmp >> 4);
    TxData[6] = ((KdTmp & 0xF) << 4) | (TorTmp >> 8);
    TxData[7] = TorTmp;

    CANTransmit(DM_Motor_Type, ID, TxData);
}

/**
 * @brief 达妙电机MIT模式控制函数，并切换使能使能状态
 * @param MotorID 电机ID，指定目标电机
 * @param State 电机使能状态
 * @param Pos 位置设定值(rad)
 * @param Vel 速度设定值(rad/s)
 * @param Kp 位置比例系数
 * @param Kd 位置微分系数
 * @param Tor 力矩设定值(NM)
 **/
void DM_MitControl(DM_Motor_TX_ID_e MotorID, MotorMode_e State, float Pos, float Vel, float Kp, float Kd, float Tor)
{
    if (GetMotorState(MotorID) >= 2)
    {
        DMMotorClearErrors(MotorID, MIT_MODE);
    }
    else
    {
        if (GetMotorState(MotorID) == State)
            MitControl(MotorID, DM_pos_limit(Pos), Vel, Kp, Kd, Tor);
        else if (State == MOTOR_ENABLE)
            DMMotorEnable(MotorID, MIT_MODE);
        else
            DMMotorDisable(MotorID, MIT_MODE);
    }
}
float DM_pos_limit(float pos)//电机限位函数
{
    if (pos < DM_POS_LIMIT_MIN)
        return DM_POS_LIMIT_MIN;
    else if (pos > DM_POS_LIMIT_MAX)
        return DM_POS_LIMIT_MAX;
    else
        return pos;
}