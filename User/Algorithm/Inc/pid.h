#ifndef _PID_H
#define _PID_H

#include "stdint.h"
#include "stdlib.h"


// PID 优化环节使能标志位,通过位与可以判c断启用的优化环节;也可以改成位域的形式
typedef enum
{
    PID_IMPROVE_NONE = 0x00,                // 0000 0000
    PID_Integral_Limit = 0x01,              // 0000 0001
    PID_Derivative_On_Measurement = 0x02,   // 0000 0010
    PID_Trapezoid_Intergral = 0x04,         // 0000 0100
    PID_Proportional_On_Measurement = 0x08, // 0000 1000
    PID_OutputFilter = 0x10,                // 0001 0000
    PID_ChangingIntegrationRate = 0x20,     // 0010 0000
    PID_DerivativeFilter = 0x40,            // 0100 0000
    PID_ErrorHandle = 0x80,                 // 1000 0000
} PID_Improvement_e;

/* PID 报错类型枚举*/
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U	// 堵转错误
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/* PID结构体 */
typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float MaxIout;
    float DeadBand;

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit;     // 积分限幅
    float CoefA;             // 变速积分 For Changing Integral
    float CoefB;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
    float Derivative_LPF_RC; // 微分滤波器系数

    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Set;

    uint32_t DWT_CNT;
    float dt;
	_Bool first_flag;

    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

/* 用于PID初始化的结构体*/
typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float MaxIout;  // 积分限幅(针对全向轮代码yaw角度环需要于是添加一条)
    float DeadBand; // 死区

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit; // 积分限幅
    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;
} PID_Init_Config_s;
/**
 * @brief 经典位置式 PID 的参数和运行状态。
 *
 * Current、Target 及输出的单位由具体控制对象决定，同一个实例不得在多个
 * 控制对象之间混用，否则积分和上次误差会互相影响。
 */
typedef struct __PID_Position_Struct
{
  float Kp;           // 比例系数
  float Ki;           // 积分系数
  float Kd;           // 微分系数
  float Error;        // 当前误差，等于Target-Current
  float Integral;     // 当前误差累加值，未乘Ki
  float IntegralOutput;      // 当前积分项输出，等于Ki*Integral
  float IntegralOutputLimit; // 积分项输出绝对限幅，单位与最终输出一致
  float Differential; // 当前误差变化量，等于Error-Error_Last1，未乘Kd
  float Output;       // 经过输出限幅后的最终PID输出
  float Error_Last1;  // 上一次误差，用于计算离散微分项
} PID_Position_Struct;
typedef struct
{
  float Kp, Ki, Kd;     //ϵ��
  float Integral;    //����(�ۻ�)
  float Error_Last1; //�ϴ����
} PID_Init_Config_a;
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);
float PIDCalculate(PIDInstance *pid, float set, float measure);
float PID_Angle_Position(PID_Position_Struct *PID, float Current, float Target,float limit);
void PIDClear(PIDInstance *pid);
float calculate_angle_error(float current, float target);
void PID_Init(void);
float PID_Position(PID_Position_Struct *PID, float Current, float Target,float limit);
extern PIDInstance Motor1SpeedPID;
extern PIDInstance Motor2SpeedPID;
extern PIDInstance Motor3SpeedPID;
extern PIDInstance Motor4SpeedPID;
extern PID_Position_Struct PID_straight;
extern PID_Position_Struct PID_sensor1;
extern PID_Position_Struct PID_sensor2;
extern PID_Position_Struct PID_sensor3;
extern PID_Position_Struct PID_YAW;
extern PID_Position_Struct PID_DM_Pitch_Position;
#endif

