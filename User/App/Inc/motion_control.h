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
 * @brief 任务3单段控制参数。
 * @note Kp、Ki用于位置环，Kv用于滤波后视觉速度反馈。
 */
typedef struct
{
    float Kp;
    float Ki;
    float Kv;
} Task3SegmentParam_t;

typedef enum
{
    TASK_3_SEGMENT_NEAR = 0,
    TASK_3_SEGMENT_LOW_PIXEL_MIDDLE,
    TASK_3_SEGMENT_LOW_PIXEL_FAR,
    TASK_3_SEGMENT_HIGH_PIXEL_MIDDLE,
    TASK_3_SEGMENT_HIGH_PIXEL_FAR
} Task3Segment_e;

/**
 * @brief 任务3五段位置控制配置和运行状态。
 * @note enabled是TIM4选择任务3策略的任务标志。底盘正向加速时小球趋向
 *       低像素侧，因此正的acceleration_feedforward_gain产生正球杆角，
 *       用来推动小球返回高像素侧。
 */
typedef struct
{
    volatile uint8_t enabled;
    volatile Task3Segment_e active_segment;
    float near_error_limit_pixel;
    float middle_error_limit_pixel;
    float velocity_filter_time_constant_s;       // 任务3视觉速度滤波时间常数
    float near_velocity_deadband_pixel_s;        // 近段速度反馈死区(pixel/s)
    Task3SegmentParam_t near;
    Task3SegmentParam_t low_pixel_middle;
    Task3SegmentParam_t low_pixel_far;
    Task3SegmentParam_t high_pixel_middle;
    Task3SegmentParam_t high_pixel_far;
    Task3SegmentParam_t normal;
    volatile float chassis_acceleration_raw_rad_s2; // S曲线指令加速度(rad/s^2)
    volatile float chassis_acceleration_rad_s2;     // 送入前馈的滤波加速度
    float acceleration_filter_alpha;                // 指令加速度滤波系数[0,1]
    float acceleration_feedforward_gain;
    float acceleration_feedforward_limit_rad;
    volatile float acceleration_feedforward_angle_rad;
} Task3SegmentedControl_t;

/**
 * @brief 控制达妙Pitch电机返回保存的电机零点。
 * @note 本函数包含阻塞循环，位置连续10次进入[-0.05, 0.05] rad后退出。
 */
void DM_Pitch_ReturnZero(void);

/**
 * @brief 根据小球位置计算球杆目标角，并发送达妙MIT位置控制指令。
 * @param target_position 小球目标位置，单位pixel。
 * @param current_position 小球当前位置，单位pixel。
 * @param feedforward_angle_rad 额外前馈球杆角，单位rad；任务2传入0。
 * @note 调用前必须完成PID、CAN、电机使能和回零；函数本身无阻塞循环。
 */
void position_control(float target_position,
                      float current_position,
                      float feedforward_angle_rad);

/**
 * @brief 打开或关闭由TIM4调度的球杆位置控制。
 * @param target_position 小球目标位置，单位pixel。
 * @param tolerance_pixel 任务2目标的到达判断半宽，单位pixel；
 *        任务3传入0，TIM4不会进行到达判断。
 */
void BallBalanceControl_Start(float target_position,
                              float tolerance_pixel);
void BallBalanceControl_Stop(void);
void BallBalanceControl_SetTarget(float target_position,
                                  float tolerance_pixel);
void Task3SegmentedControl_Enable(void);
void Task3SegmentedControl_Disable(void);
void Task3ChassisCommandAccelerationSet(float acceleration_rad_s2);

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
extern volatile Task3SegmentedControl_t task3_segmented_control;

// 底盘实时计时状态，单位为毫秒。
extern volatile uint8_t chassis_motion_timing_active;
extern volatile uint32_t chassis_motion_elapsed_ms;

#endif /* MOTION_CONTROL_H_ */
