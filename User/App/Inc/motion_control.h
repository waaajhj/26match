#ifndef MOTION_CONTROL_H_
#define MOTION_CONTROL_H_

#include "stm32f4xx_hal.h"

// 视觉坐标中的球杆中心目标，单位pixel。
#define BALL_BALANCE_DEFAULT_TARGET_POSITION 287.5f

/**
 * @brief 控制达妙Pitch电机返回保存的电机零点。
 * @note 本函数包含阻塞循环，位置连续10次进入[-0.05, 0.05] rad后退出。
 */
void DM_Pitch_ReturnZero(void);

/**
 * @brief 根据小球位置计算球杆目标角，并发送达妙MIT位置控制指令。
 * @param target_position 小球目标位置，单位pixel。
 * @param current_position 小球当前位置，单位pixel。
 * @note 调用前必须完成PID、CAN、电机使能和回零；函数本身无阻塞循环。
 */
void position_control(float target_position, float current_position);

/**
 * @brief 打开或关闭由TIM4调度的球杆位置控制。
 * @param target_position 小球目标位置，单位pixel。
 */
void BallBalanceControl_Start(float target_position);
void BallBalanceControl_Stop(void);

/**
 * @brief 底盘运动过程及OLED运行时间显示接口。
 * @note ChassisTrack_Run()和ChassisTrack2_Run()包含阻塞式运动循环，
 *       运行期间TIM4仍可中断并执行球杆控制。
 */
void ChassisTrack_Run(void);
void ChassisTrack2_Run(void);
void ChassisMotionTime_Start(void);
void ChassisMotionTime_Update(void);
void ChassisMotionTime_Stop(void);

// 以下状态量由TIM4回调更新，用于Keil Watch或LinkScope调试球杆控制。
extern volatile uint8_t ball_balance_control_enabled;
extern volatile float ball_balance_target_position;
extern volatile uint32_t ball_balance_control_count;
extern volatile uint32_t ball_balance_no_new_frame_count;
extern volatile float ball_balance_raw_velocity_pixel_s;
extern volatile float ball_balance_filtered_velocity_pixel_s;
extern volatile float ball_balance_velocity_kv;
extern volatile float ball_balance_velocity_feedback_angle_rad;
extern volatile float ball_balance_rod_target_angle_rad;

// 底盘实时计时状态，单位为毫秒。
extern volatile uint8_t chassis_motion_timing_active;
extern volatile uint32_t chassis_motion_elapsed_ms;

#endif /* MOTION_CONTROL_H_ */
