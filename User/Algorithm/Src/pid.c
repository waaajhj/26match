/**
 *************************(C) COPYRIGHT 2024 DragonBot*************************
 * @file	pid.c/h
 * @brief	位置式PID控制器定义, 包含PID优化环节
 * @history
 * Date          Author         Modification
 * 2020-05-01    wanghongxi     代码框架
 * 2024-08-30    ZouAjie	    优化代码
 * @attention
 ==============================================================================
 初始化
 PID_Init_Config_s xxx_config = {
     .MaxOut = 2000.0f,
     .IntegralLimit = 1000.0f,
     .Kp = 1000.0f,
     .Ki = 20.0f,
     .Kd = 0.1f,
     .Derivative_LPF_RC = 0.008f,
     .Improve = PID_Integral_Limit | PID_DERIVATIVE_FILTER};
 PIDInit(&xxx_pid, &xxx_config);
 ==============================================================================
 *************************(C) COPYRIGHT 2024 DragonBot*************************
*/
#include "pid.h"
#include "arm_math.h"
#include "bsp_dwt.h"
PIDInstance Motor1SpeedPID;
PIDInstance Motor2SpeedPID;
PIDInstance Motor3SpeedPID;
PIDInstance Motor4SpeedPID;
PID_Position_Struct PID_straight;
PID_Position_Struct PID_YAW;
PID_Position_Struct PID_sensor1;
PID_Position_Struct PID_sensor2;
PID_Position_Struct PID_sensor3;
/* ----------------------------下面是pid优化环节的实现---------------------------- */
// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// 线性变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (fabsf(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (fabsf(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}

// 积分限幅
static void f_Integral_Limit(PIDInstance *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (fabsf(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0) // 积分还在累积
        {
            pid->ITerm = 0; // 当前积分项置零
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(PIDInstance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 输出滤波
static void f_Output_Filter(PIDInstance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// 电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    // PID设定或输出值较小就不进行堵转检测
    if (pid->Output < pid->MaxOut * 0.001f || fabsf(pid->Set) < 0.0001f)
        return;

    if ((fabsf(pid->Set - pid->Measure) / fabsf(pid->Set)) > 0.95f)
    {
        // 电机堵转计数
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // 电机堵转计数值大于500次, 控制任务一般2-3ms, 即堵转1-1.5s
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */
/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 * @param pid: PID实例
 * @param config: PID初始化设置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    // config的数据和pid的部分数据是连续且相同的的,所以可以直接用memcpy
    memset(pid, 0, sizeof(PIDInstance));
    // 利用其内存连续的结构体的特性, 将剩余内存设置为0
    memcpy(pid, config, sizeof(PID_Init_Config_s));
}
void PID_Init(void){
    PID_straight.Kp = 60.0f;   
    PID_straight.Ki = 0.0f;
    PID_straight.Kd = 20.0f;
    PID_YAW.Kp = 100.0f;
    PID_YAW.Ki = 0.0f;
    PID_YAW.Kd = 0.0f;
    PID_sensor1.Kp = 60.0f;
    PID_sensor1.Ki = 0.0f;
    PID_sensor1.Kd = 1.0f;
    PID_sensor2.Kp = 70.0f;
    PID_sensor2.Ki = 0.0f;
    PID_sensor2.Kd = 0.0f;
	  PID_sensor3.Kp = 30.0f;
    PID_sensor3.Ki = 0.0f;
    PID_sensor3.Kd = 0.0f;

}
/**
 * @brief PID计算, 并反回PID输出
 * @param pid: PID实例指针
 * @param measure: 反馈值
 * @param set: 设定值
 */
float PIDCalculate(PIDInstance *pid, float set, float measure)
{
    // 第一次进入对齐系统时间, 直接退出, 防止ITerm在第一次进入时突变无法快速衰减
    if (pid->first_flag == 0)
    {
        pid->first_flag = 1;
        DWT_GetDeltaT((void *)&pid->DWT_CNT);
        return 0.0f;
    }

    // 堵转检测
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = DWT_GetDeltaT((void *)&pid->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分

    // 保存上次的测量值和误差,计算当前error
    pid->Measure = measure;
    pid->Set = set;
    pid->Err = pid->Set - pid->Measure;

    // 如果在死区外,则计算PID
    if (fabsf(pid->Err) > pid->DeadBand)
    {
        // 基本的pid计算,使用位置式
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         // 累加积分
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // 保存当前数据,用于下次计算
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}
float calculate_angle_error(float current, float target) {
    float error = target - current;
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }
	// if (error > 0) {
	// 	motor1.direction = turn_left;
	// } else if (error < 0) {
	// 	motor1.direction = turn_right;
	// }
    return error;
}
float PID_Position(PID_Position_Struct *PID, float Current, float Target,float limit)
{
  float err,                                                                                                        //���
      out,                                                                                                          //���
      differential;                                                                                                 //΢��
  err = (float)Target - (float)Current;                                                                             //�������
  PID->Integral += err;                                                                                             //���»���
  differential = (float)err - (float)PID->Error_Last1;                                                              //����΢��
  out = (float)PID->Kp * (float)err + (float)PID->Ki * (float)PID->Integral + (float)PID->Kd * (float)differential; //����PID
  PID->Error_Last1 = err;       //�������
   if(out>=limit){
	  out=limit;
  }
   if(out<=-limit){
	  out=-limit;
  }
  return out;
}
float PID_Angle_Position(PID_Position_Struct *PID, float Current, float Target,float limit)
{
  float err,                                                                                                        //���
      out,                                                                                                          //���
      differential;                                                                                                 //΢��
  err = calculate_angle_error(Current, Target);                                                                             //�������
  PID->Integral += err;                                                                                             //���»���
  differential = (float)err - (float)PID->Error_Last1;                                                              //����΢��
  out = (float)PID->Kp * (float)err + (float)PID->Ki * (float)PID->Integral + (float)PID->Kd * (float)differential; //����PID
  PID->Error_Last1 = err;       //�������
   if(out>=limit){
	  out=limit;
  }
   if(out<=-limit){
	  out=-limit;
  }
  return out;
}

/**
 * @brief 清除PID, 将除初始化的其他数据置零
 * @param pid: PID实例
 */
void PIDClear(PIDInstance *pid)
{
    pid->Measure = pid->Last_Measure = pid->Err = pid->Last_Err = pid->Last_ITerm = 0.0f;
    pid->Pout = pid->Iout = pid->Dout = pid->ITerm = 0.0f;
    pid->Output = pid->Last_Output = pid->Last_Dout = pid->Set = pid->dt = 0.0f;
    pid->DWT_CNT = 0;
    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
    pid->first_flag = 0;
}
