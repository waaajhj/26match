#ifndef MOTION_CONTROL_H_
#define MOTION_CONTROL_H_

#include "stm32f4xx_hal.h"

/*
 * 球杆视觉坐标，单位pixel。
 * 相机有效范围为12~457 pixel；任务目标采用装机后的实测标定值：
 * +5 cm为128 pixel、中心0点为228 pixel、-5 cm为322 pixel。
 * 本机构定义的正方向对应像素减小，重新安装相机后应重新标定三点。
 */
#define BALL_BALANCE_VISUAL_START_PIXEL 12.0f
#define BALL_BALANCE_VISUAL_END_PIXEL 457.0f
#define BALL_BALANCE_VISUAL_LENGTH_CM 25.0f
#define BALL_BALANCE_DEFAULT_TARGET_POSITION 228.0f
// #define BALL_BALANCE_FIVE_CM_OFFSET_PIXEL \
//     (((BALL_BALANCE_VISUAL_END_PIXEL - BALL_BALANCE_VISUAL_START_PIXEL) / \
//       BALL_BALANCE_VISUAL_LENGTH_CM) * 5.0f)
#define BALL_BALANCE_POSITIVE_5CM_TARGET_POSITION 128.0f
    // (BALL_BALANCE_DEFAULT_TARGET_POSITION + BALL_BALANCE_FIVE_CM_OFFSET_PIXEL)
#define BALL_BALANCE_NEGATIVE_5CM_TARGET_POSITION 322.0f
    // (BALL_BALANCE_DEFAULT_TARGET_POSITION - BALL_BALANCE_FIVE_CM_OFFSET_PIXEL)

// 正5 cm和中心使用±25 pixel，最终-5 cm稳定判断使用±15 pixel。
#define BALL_BALANCE_INTERMEDIATE_TOLERANCE_PIXEL 25.0f
#define BALL_BALANCE_FINAL_TOLERANCE_PIXEL 15.0f

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
 * @param tolerance_pixel 当前目标的到达判断半宽，单位pixel。
 */
void BallBalanceControl_Start(float target_position,
                              float tolerance_pixel);
void BallBalanceControl_Stop(void);
void BallBalanceControl_SetTarget(float target_position,
                                  float tolerance_pixel);

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

// 以下共享状态用于任务控制，也可在Keil Watch或LinkScope中观察。
extern volatile uint8_t ball_balance_control_enabled;
extern volatile uint8_t ball_balance_target_in_range_count;
extern volatile float ball_balance_target_position;
extern volatile float ball_balance_target_tolerance_pixel;
extern volatile float ball_balance_raw_velocity_pixel_s;
extern volatile float ball_balance_filtered_velocity_pixel_s;
extern volatile float ball_balance_velocity_kv;
extern volatile float ball_balance_velocity_feedback_angle_rad;
extern volatile float ball_balance_rod_target_angle_rad;

// 底盘实时计时状态，单位为毫秒。
extern volatile uint8_t chassis_motion_timing_active;
extern volatile uint32_t chassis_motion_elapsed_ms;

#endif /* MOTION_CONTROL_H_ */
