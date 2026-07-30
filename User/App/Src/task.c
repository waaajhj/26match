#include "task.h"
#include "main.h"
#include "pid.h"
#include "sensor.h"
#include "OLED.h"
#include "jy61p.h"
#include "arm.h"
#include "chassis.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "maixcam.h"
#include "usart.h"
#include "tim.h"
#include "OLED.h"
//定义小车行驶速度
#define speed_target 10.0f
// 视觉坐标中的摆杆中心点，后续完成像素/厘米标定后可按实测值修改。
#define BALL_BALANCE_DEFAULT_TARGET_POSITION 287.5f
//视觉起点像素点120，终点450
#define BALL_BALANCE_VISUAL_START_PIXEL 122.0f
#define BALL_BALANCE_VISUAL_END_PIXEL 453.0f
// 球杆平衡时电机的中立位置，以达妙电机保存的零点为基准，单位 rad。
#define BALL_BALANCE_MOTOR_ZERO_ANGLE_RAD 0.0f

// PID最多允许球杆相对中立位置倾斜20°，单位rad。
#define BALL_BALANCE_MAX_TILT_ANGLE_RAD 0.44906585f

// OLED显示底盘运行时间的刷新周期；100 ms可兼顾实时性与软件I2C开销。
#define CHASSIS_TIME_OLED_REFRESH_MS 100U

/*
 * TIM4 球杆控制状态。
 * 这些变量保留为全局量，便于在 Keil Watch 窗口中确认任务和中断是否正常运行。
 */
volatile uint8_t ball_balance_control_enabled = 0U;
volatile float ball_balance_target_position =BALL_BALANCE_DEFAULT_TARGET_POSITION;
volatile uint32_t ball_balance_control_count = 0U;
volatile uint32_t ball_balance_no_new_frame_count = 0U;

// TIM4 每次只处理一帧新视觉数据，避免对同一坐标重复执行 PID 微分计算。
static uint32_t ball_balance_last_packet_count = 0U;

// 新灰度模块的单字节查询命令；末尾 0 供现有 usart6_send() 计算长度。
static uint8_t grayscale_query_command[] = "\x61";

/*
 * 底盘运行计时状态。
 * elapsed_ms保留停车时的最终结果，也可在Keil Watch中辅助验证OLED显示。
 */
volatile uint8_t chassis_motion_timing_active = 0U;
volatile uint32_t chassis_motion_elapsed_ms = 0U;
static uint32_t chassis_motion_start_ms = 0U;
static uint32_t chassis_oled_last_refresh_ms = 0U;

/**
 * @brief 从底盘开始加速时启动计时，并初始化OLED时间显示。
 */
void ChassisMotionTime_Start(void)
{
    uint32_t now;

    // 先写固定字符，再从第一条底盘控制指令之前开始精确计时。
    OLED_ShowString(1, 1, "TIME:");
    OLED_ShowNum(1, 6, 0U, 5);
    OLED_ShowString(1, 11, "ms");

    now = read_time();
    chassis_motion_start_ms = now;
    chassis_oled_last_refresh_ms = now;
    chassis_motion_elapsed_ms = 0U;
    chassis_motion_timing_active = 1U;
}

/**
 * @brief 在主循环控制代码中周期调用，最多每100 ms刷新一次OLED。
 * @note 本函数不能放入TIM4中断；OLED使用软件I2C，刷新时间较长。
 */
void ChassisMotionTime_Update(void)
{
    uint32_t now;

    if (chassis_motion_timing_active == 0U)
    {
        return;
    }

    now = read_time();
    chassis_motion_elapsed_ms = now - chassis_motion_start_ms;

    if ((now - chassis_oled_last_refresh_ms) >=
        CHASSIS_TIME_OLED_REFRESH_MS)
    {
        // 只更新数字区域，避免反复刷新固定字符增加软件I2C开销。
        OLED_ShowNum(1, 6, chassis_motion_elapsed_ms, 5);
        chassis_oled_last_refresh_ms = now;
    }
}

/**
 * @brief 在减速停车完成后停止计时，并保留最终时间。
 */
void ChassisMotionTime_Stop(void)
{
    if (chassis_motion_timing_active == 0U)
    {
        return;
    }

    chassis_motion_elapsed_ms =
        read_time() - chassis_motion_start_ms;
    OLED_ShowNum(1, 6, chassis_motion_elapsed_ms, 5);
    chassis_motion_timing_active = 0U;
}

/**
 * @brief 启动 TIM4 球杆位置控制。
 * @param target_position 视觉坐标系中的目标位置。
 * @note TIM4 已在 main() 中启动，本函数只打开控制开关并复位本位置环状态。
 */
void BallBalanceControl_Start(float target_position)
{
    ball_balance_target_position = target_position;
    ball_balance_last_packet_count = point_packet_rx_count;
    PID_DM_Pitch_Position.Error = 0.0f;
    PID_DM_Pitch_Position.Integral = 0.0f;
    PID_DM_Pitch_Position.IntegralOutput = 0.0f;
    PID_DM_Pitch_Position.Differential = 0.0f;
    PID_DM_Pitch_Position.Output = 0.0f;
    PID_DM_Pitch_Position.Error_Last1 = 0.0f;
    ball_balance_control_enabled = 1U;
}

/**
 * @brief 停止 TIM4 球杆闭环，并让球杆回到中立角度。
 */
void BallBalanceControl_Stop(void)
{
    ball_balance_control_enabled = 0U;
    PID_DM_Pitch_Position.Error = 0.0f;
    PID_DM_Pitch_Position.Integral = 0.0f;
    PID_DM_Pitch_Position.IntegralOutput = 0.0f;
    PID_DM_Pitch_Position.Differential = 0.0f;
    PID_DM_Pitch_Position.Output = 0.0f;
    PID_DM_Pitch_Position.Error_Last1 = 0.0f;
    DM_MitControl(DM_PITCH_TX_ID, MOTOR_ENABLE,
                  BALL_BALANCE_MOTOR_ZERO_ANGLE_RAD,
                  0.0f, 2.0f, 0.1f, 0.0f);
}

/**
 * @brief 控制达妙 Pitch 电机返回电机零点。
 * @note 本函数会阻塞，直至位置连续 10 次位于 [-0.05, 0.05] rad。
 *       未收到有效 Pitch 反馈时不会退出，并会每 10 ms 重发一次回零命令。
 */
void DM_Pitch_ReturnZero(void)
{
    const DM_Motor_t *pitch_motor = get_gimbal_motor_measure_point(1U);
    uint8_t position_stable_count = 0U;

    while (position_stable_count < 10U)
    {
        float pitch_position = pitch_motor->Position;
        uint8_t feedback_valid =
            (pitch_motor->measure.ID == (uint8_t)DM_PITCH_TX_ID);

        if (feedback_valid &&
            pitch_position >= -0.05f &&
            pitch_position <= 0.05f)
        {
            position_stable_count++;
        }
        else
        {
            position_stable_count = 0U;
        }

        if (position_stable_count < 10U)
        {
            DM_MitControl(DM_PITCH_TX_ID, MOTOR_ENABLE,
                          0.0f, 0.0f, 4.0f, 0.1f, 0.0f);
            HAL_Delay(10U);
        }
    }
}
/**
 * @brief 根据球的位置误差计算达妙Pitch电机的目标角度。
 * @param target_position 球的目标位置，单位与视觉位置数据一致。
 * @param current_position 球的当前位置，单位与视觉位置数据一致。
 * @note 调用前必须完成 PID_Init()、CAN 初始化、电机使能和回零。
 *       建议以固定周期调用；本函数每次调用会发送一帧MIT位置控制指令。
 */
void position_control(float target_position, float current_position)
{
    float angle_offset = PID_Position(&PID_DM_Pitch_Position,
                                      current_position,
                                      target_position,
                                      BALL_BALANCE_MAX_TILT_ANGLE_RAD);

    /*
     * 最终目标角度先叠加平衡零点，再经过达妙绝对电控限位，
     * 因此同时受到相对倾角±20°和绝对角度[-1.1, 0.65] rad的保护。
     */
    float motor_target_angle =
        DM_pos_limit(BALL_BALANCE_MOTOR_ZERO_ANGLE_RAD + angle_offset);

    DM_MitControl(DM_PITCH_TX_ID, MOTOR_ENABLE,
                  motor_target_angle, 0.0f, 5.0f, 0.1f, 0.0f);
}

/**
 * @brief 执行一次底盘循迹：平滑加速、循迹至路口、平滑停车。
 * @note 本函数在主循环上下文运行，球杆控制可由 TIM4 中断并行执行。
 */
static void ChassisTrack_Run(void)
{
    ChassisMotionTime_Start();
    S_regulate_track(0, speed_target, 800); // 加速到目标速度
    while (scan_cross_nostop(line) != 0)
    {
        track_dynamic_Speed(speed_target);
        ChassisMotionTime_Update();
        delay_ms(3);
    }
    // S_regulate_Ctl(speed_target, 0.0f, 100); // 减速停车
	DM_SpeedControl(DM_Chassis1_TX_ID,MOTOR_ENABLE,0);
    DM_SpeedControl(DM_Chassis2_TX_ID,MOTOR_ENABLE,0);
    ChassisMotionTime_Stop();
}
static void ChassisTrack2_Run(void)
{
    ChassisMotionTime_Start();
    S_regulate_track(0, speed_target, 800); // 加速到目标速度
    uint32_t time_on;

    time_on = read_time();
    while (read_time() <= time_on + time) {
        track_dynamic_Speed(speed_target);
        ChassisMotionTime_Update();
        delay_ms(5);
    }
    S_regulate_Ctl(speed_target, 0.0f, 300); // 减速停车
    ChassisMotionTime_Stop();
}

/**
 * @brief 任务1：只运行底盘循迹，不启用球杆闭环。
 */
void task_1(void)
{
    BallBalanceControl_Stop();
    ChassisTrack_Run();
    serial_screen_task = SERIAL_SCREEN_TASK_NONE; // 状态复位
}

/**
 * @brief 任务2：底盘保持静止，只运行 TIM4 球杆闭环。
 */
void task_2(void)
{
    // DM_SpeedControl(DM_Chassis1_TX_ID, MOTOR_ENABLE, 0.0f);
    // DM_SpeedControl(DM_Chassis2_TX_ID, MOTOR_ENABLE, 0.0f);
    BallBalanceControl_Start(BALL_BALANCE_DEFAULT_TARGET_POSITION);
    serial_screen_task = SERIAL_SCREEN_TASK_NONE; // 防止主循环重复启动任务
}

/**
 * @brief 任务3：主循环执行底盘循迹，TIM4 同时执行球杆闭环。
 */
void task_3(void)
{
    BallBalanceControl_Start(BALL_BALANCE_DEFAULT_TARGET_POSITION);
    ChassisTrack2_Run();
    serial_screen_task = SERIAL_SCREEN_TASK_NONE; // 状态复位
}

/**
 * @brief 根据串口屏发送的任务号执行任务。
 */
void task_switch(void)
{
    switch (serial_screen_task)
    {
        case SERIAL_SCREEN_TASK_1:
            task_1();
            break;
        case SERIAL_SCREEN_TASK_2:
            task_2();
            break;
        case SERIAL_SCREEN_TASK_3:
            task_3();
            break;
        case SERIAL_SCREEN_TASK_4:
            break;
        case SERIAL_SCREEN_TASK_5:
            break;
        default:
            break;
    }
}

/**
 * @brief HAL 定时器周期回调。
 * @note TIM3 保留原有的灰度模块查询；TIM4 只在收到新视觉帧时更新球杆控制。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3)
    {
        // 每 10 ms 查询一次 12 路灰度模块。
        usart6_send(grayscale_query_command);
        return;
    }

    if (htim == &htim4)
    {
        uint32_t current_packet_count;
        float current_position;

        if (ball_balance_control_enabled == 0U)
        {
            return;
        }

        current_packet_count = point_packet_rx_count;
        if ((current_packet_count == 0U) ||
            (current_packet_count == ball_balance_last_packet_count))
        {
            // 没有新视觉数据时保持上一条电机指令，不重复计算位置式PID。
            ball_balance_no_new_frame_count++;
            return;
        }

        /*
         * UART4 在完整包校验成功后先更新 point_packet，再增加接收计数。
         * TIM4 因此可用计数变化判断当前坐标是否为一帧新数据。
         */
        current_position = (float)point_packet.centerpoint_x;
        ball_balance_last_packet_count = current_packet_count;
        position_control(ball_balance_target_position, current_position);
        ball_balance_control_count++;
    }
}
