#ifndef TASK_H_
#define TASK_H_
#include "stm32f4xx_hal.h"

void DM_Pitch_ReturnZero(void);

/**
 * @brief 根据球的位置误差计算达妙Pitch电机的目标角度。
 * @param target_position 球的目标位置，单位与视觉位置数据一致。
 * @param current_position 球的当前位置，单位与视觉位置数据一致。
 * @note 调用前必须完成 PID_Init()、CAN 初始化、电机使能和回零。
 *       函数会发送MIT位置控制指令，并限制电机目标角度。
 */
void position_control(float target_position, float current_position);

/**
 * @brief 打开或关闭由 TIM4 驱动的球杆位置控制。
 */
void BallBalanceControl_Start(float target_position);
void BallBalanceControl_Stop(void);

/**
 * @brief 底盘运行时间的启动、实时刷新和停止接口。
 */
void ChassisMotionTime_Start(void);
void ChassisMotionTime_Update(void);
void ChassisMotionTime_Stop(void);

//任务函数
void task_1(void);
void task_2(void);
void task_3(void);
void task_4(void);
void task_5(void);
void task_switch(void);
static void ChassisTrack2_Run(void);
static void ChassisTrack_Run(void);

// 以下状态量用于 Keil Watch 调试 TIM4 球杆控制。
extern volatile uint8_t ball_balance_control_enabled;
extern volatile float ball_balance_target_position;
extern volatile uint32_t ball_balance_control_count;
extern volatile uint32_t ball_balance_no_new_frame_count;

// 底盘实时计时状态，单位为毫秒。
extern volatile uint8_t chassis_motion_timing_active;
extern volatile uint32_t chassis_motion_elapsed_ms;
#endif /* TASK_H_ */
