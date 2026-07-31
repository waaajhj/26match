#ifndef MOTION_CONTROL_H_
#define MOTION_CONTROL_H_

#include "stm32f4xx_hal.h"

/*
 * 球杆视觉坐标，单位pixel。
 * 相机有效范围为12~457 pixel；任务目标采用装机后的实测标定值：
 * +5 cm为128 pixel、中心0点为227 pixel、-5 cm为322 pixel。
 * 本机构定义的正方向对应像素减小，重新安装相机后应重新标定三点。
 */
#define BALL_BALANCE_VISUAL_START_PIXEL 15.0f
#define BALL_BALANCE_VISUAL_END_PIXEL 460.0f
#define BALL_BALANCE_VISUAL_LENGTH_CM 25.0f
#define BALL_BALANCE_DEFAULT_TARGET_POSITION 227.0f
// #define BALL_BALANCE_FIVE_CM_OFFSET_PIXEL \
//     (((BALL_BALANCE_VISUAL_END_PIXEL - BALL_BALANCE_VISUAL_START_PIXEL) / \
//       BALL_BALANCE_VISUAL_LENGTH_CM) * 5.0f)
#define BALL_BALANCE_POSITIVE_5CM_TARGET_POSITION 135.0f
    // (BALL_BALANCE_DEFAULT_TARGET_POSITION + BALL_BALANCE_FIVE_CM_OFFSET_PIXEL)
#define BALL_BALANCE_NEGATIVE_5CM_TARGET_POSITION 338.0f
    // (BALL_BALANCE_DEFAULT_TARGET_POSITION - BALL_BALANCE_FIVE_CM_OFFSET_PIXEL)

// 正5 cm和中心使用±25 pixel，最终-5 cm稳定判断使用±15 pixel。
#define BALL_BALANCE_INTERMEDIATE_TOLERANCE_PIXEL 20.0f // +5 cm阶段到达阈值，按当前任务时间要求设置
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
    TASK_2_SEGMENT_LOW_PIXEL_NEAR = 0,
    TASK_2_SEGMENT_HIGH_PIXEL_NEAR,
    TASK_2_SEGMENT_MIDDLE,
    TASK_2_SEGMENT_FAR
} Task2Segment_e;

/**
 * @brief 任务2从+5 cm直达-5 cm时使用的分段控制参数。
 * @note 参数独立于任务3，后续可通过LinkScope单独调整，不会改变任务3效果。
 */
typedef struct
{
    volatile uint8_t enabled;
    volatile Task2Segment_e active_segment;
    float near_error_limit_pixel;
    float middle_error_limit_pixel;
    float velocity_filter_time_constant_s;
    float near_velocity_deadband_pixel_s;
    Task3SegmentParam_t low_pixel_near;
    Task3SegmentParam_t high_pixel_near;
    Task3SegmentParam_t middle;
    Task3SegmentParam_t far;
    Task3SegmentParam_t normal;
} Task2SegmentedControl_t;

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
    float target_offset_pixel;                  // 任务3内部目标零偏(pixel)，不修改公共中心坐标
    float velocity_filter_time_constant_s;       // 任务3视觉速度滤波时间常数
    float near_velocity_deadband_pixel_s;        // 近段速度反馈死区(pixel/s)
    float startup_velocity_kv;                   // 加速阶段向低像素滑动时的速度反馈增益(rad/(pixel/s))
    float transition_high_brake_start_pixel;     // 加速及目标渐入阶段的高像素软边界起点(pixel)
    float transition_high_brake_gain_rad_per_pixel; // 超过高像素软边界后的附加制动增益(rad/pixel)
    float transition_high_brake_limit_rad;       // 高像素软边界附加制动角限幅(rad)
    Task3SegmentParam_t near;
    Task3SegmentParam_t low_pixel_middle;
    Task3SegmentParam_t low_pixel_far;
    Task3SegmentParam_t high_pixel_middle;
    Task3SegmentParam_t high_pixel_far;
    Task3SegmentParam_t normal;
    float pitch_motor_kp;                         // 任务3达妙Pitch电机MIT位置刚度
    float pitch_motor_kd;                         // 任务3达妙Pitch电机MIT速度阻尼
    volatile float chassis_acceleration_raw_rad_s2; // S曲线指令加速度(rad/s^2)
    volatile float chassis_acceleration_rad_s2;     // 送入前馈的滤波加速度
    float acceleration_filter_alpha;                // 指令加速度滤波系数[0,1]
    float acceleration_release_filter_alpha;        // 正向加速前馈退出系数[0,1]
    float brake_release_filter_alpha;               // 刹车前馈退出滤波系数[0,1]
    float acceleration_feedforward_gain;
    float acceleration_feedforward_limit_rad;       // 正向加速前馈角限幅(rad)
    float acceleration_brake_feedforward_limit_rad; // 刹车减速前馈角限幅(rad)
    volatile float acceleration_feedforward_angle_rad;
} Task3SegmentedControl_t;

// 视觉约42 Hz时600个样本可记录约14 s，覆盖任务2、3或4主要运动过程。
#define TASK3_DEBUG_SAMPLE_CAPACITY 600U

/**
 * @brief 任务2、3和4单片机内部调试样本，每个视觉新包记录一次。
 * @note 角度单位为rad，速度单位为pixel/s，加速度单位为rad/s^2；
 *       数据由TIM4回调更新，只用于离线调参，不参与控制计算。任务2样本中的
 *       motion_active保存Task2Stage_e阶段值，任务3/4样本中表示底盘是否运动。
 */
typedef struct
{
    uint32_t tick_ms;
    uint32_t elapsed_ms;
    uint32_t packet_count;
    uint32_t can_error_count;
    uint16_t point_x;
    uint8_t segment;
    uint8_t motion_active;
    float effective_target_pixel;
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float pid_error;
    float pid_integral_output;
    float pid_differential;
    float pid_output;
    float raw_velocity_pixel_s;
    float filtered_velocity_pixel_s;
    float velocity_kv;
    float velocity_feedback_angle_rad;
    float rod_target_angle_rad;
    float raw_acceleration_rad_s2;
    float filtered_acceleration_rad_s2;
    float feedforward_angle_rad;
    float motor_target_angle_rad;
    float motor_position_rad;
    float motor_velocity_rad_s;
    float motor_torque_nm;
    float chassis_motor_1_velocity_rad_s;
    float chassis_motor_1_acceleration_rad_s2;
    float chassis_motor_2_velocity_rad_s;
    float chassis_motor_2_acceleration_rad_s2;
    float chassis_track_bias;
    float chassis_track_output;
} Task3DebugSample_t;

/**
 * @brief 任务2、3和4复用的内部记录器状态及连续样本缓存。
 * @note task_id为2、3或4，用于标识当前数据来源；各任务不会同时运行，因此
 *       复用缓存可避免额外占用约78 KB RAM。overflow表示缓存写满、样本被截断。
 */
typedef struct
{
    volatile uint16_t sample_count;
    volatile uint8_t recording;
    volatile uint8_t complete;
    volatile uint8_t overflow;
    volatile uint8_t task_id;
    uint8_t reserved[2];
    Task3DebugSample_t samples[TASK3_DEBUG_SAMPLE_CAPACITY];
} Task3DebugRecorder_t;

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
void Task2SegmentedControl_Enable(void);
void Task2SegmentedControl_Disable(void);
void Task3SegmentedControl_Enable(void);
void Task3SegmentedControl_Disable(void);
void Task3ChassisCommandAccelerationSet(float acceleration_rad_s2);

/**
 * @brief 清空并启动任务3片上高速数据记录器。
 * @note 仅在任务3控制和底盘即将启动时调用；函数不含循环、延时和通信，
 *       缓存写满时会停止记录并置overflow，不会改变电机输出。
 */
void Task3DebugRecorder_Start(void);

/**
 * @brief 清空共享RAM缓存并启动任务2片上数据记录器。
 * @note 仅在任务2控制即将启动时调用；函数不含循环、延时和通信，缓存写满后
 *       自动停止，不会改变PID参数、电机指令或任务状态。
 */
void Task2DebugRecorder_Start(void);

/**
 * @brief 清空共享RAM缓存并启动任务4片上数据记录器。
 * @note 仅在任务4底盘和球杆控制即将启动时调用；函数不含循环、延时和通信，
 *       缓存写满或底盘停止后结束记录，不会改变控制参数和电机输出。
 */
void Task4DebugRecorder_Start(void);

/**
 * @brief 底盘运动过程及OLED运行时间显示接口。
 * @note ChassisTrack_Run()和ChassisTrack2_Run()包含阻塞式运动循环，
 *       运行期间TIM4仍可中断并执行球杆控制。
 */
void ChassisTrack_Run(void);
void ChassisTrack2_Run(void);
void ChassisTrack3_Run(void);
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
extern volatile Task2SegmentedControl_t task2_segmented_control;
extern volatile Task3SegmentedControl_t task3_segmented_control;
extern volatile Task3DebugRecorder_t task3_debug_recorder;

// 底盘实时计时状态，单位为毫秒。
extern volatile uint8_t chassis_motion_timing_active;
extern volatile uint32_t chassis_motion_elapsed_ms;

#endif /* MOTION_CONTROL_H_ */
