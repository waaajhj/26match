#include "motion_control.h"

#include "main.h"
#include "pid.h"
#include "sensor.h"
#include "OLED.h"
#include "chassis.h"
#include "DM_Motor.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "maixcam.h"
#include "tim.h"

#include <math.h>

// 底盘循迹目标速度，单位沿用当前达妙底盘速度接口。
#define speed_target 10.0f

// 球杆水平时电机的中立位置，单位rad，以达妙电机保存零点为基准。
#define BALL_BALANCE_MOTOR_ZERO_ANGLE_RAD 0.0f

// PID最多请求球杆相对水平位置倾斜12°，单位rad。
#define BALL_BALANCE_MAX_TILT_ANGLE_RAD 0.20943951f

/*
 * 视觉速度反馈参数。
 * 速度单位为pixel/s，反馈输出单位为rad；正速度表示小球坐标向右增大。
 * 60 ms一阶低通用于削弱视觉坐标抖动，速度反馈最多贡献±2°。
 */
#define BALL_VELOCITY_FILTER_TIME_CONSTANT_S 0.060f
#define BALL_VELOCITY_FEEDBACK_LIMIT_RAD 0.03490659f
#define BALL_VELOCITY_MAX_VALID_INTERVAL_MS 100U

/*
 * 四连杆杆长，单位mm：D、C为固定铰点，AD为电机主动杆，
 * AB为中间连杆，BC为绕C点转动的球杆。
 */
#define FOUR_BAR_AD_LENGTH_MM 40.0f
#define FOUR_BAR_AB_LENGTH_MM 90.0f
#define FOUR_BAR_BC_LENGTH_MM 180.0f
#define FOUR_BAR_CD_LENGTH_MM 200.0f

/*
 * 由上述杆长和“AD、BC同时水平”的零位支路计算得到的球杆几何极限。
 * 反解前先限幅，避免目标角超出四杆闭环可达范围后acosf()无解。
 */
#define BALL_ROD_GEOMETRY_MIN_ANGLE_RAD -0.28702193f
#define BALL_ROD_GEOMETRY_MAX_ANGLE_RAD 0.16151227f

// OLED显示底盘运行时间的刷新周期；100 ms兼顾实时性与软件I2C开销。
#define CHASSIS_TIME_OLED_REFRESH_MS 100U

/*
 * TIM4球杆控制状态。
 * volatile变量由主循环任务和定时器回调共同访问，也便于LinkScope调试。
 */
volatile uint8_t ball_balance_control_enabled = 0U;
volatile uint8_t ball_balance_target_in_range_count = 0U;
volatile float ball_balance_target_position =
    BALL_BALANCE_DEFAULT_TARGET_POSITION;
volatile float ball_balance_target_tolerance_pixel =
    BALL_BALANCE_INTERMEDIATE_TOLERANCE_PIXEL;
volatile float ball_balance_raw_velocity_pixel_s = 0.0f;
volatile float ball_balance_filtered_velocity_pixel_s = 0.0f;
// 速度反馈增益，单位rad/(pixel/s)，可在LinkScope中实时修改。
volatile float ball_balance_velocity_kv = 0.00038f;
volatile float ball_balance_velocity_feedback_angle_rad = 0.0f;
volatile float ball_balance_rod_target_angle_rad = 0.0f;

// TIM4每次只处理一帧新视觉数据，避免对同一坐标重复执行位置环计算。
static uint32_t ball_balance_last_packet_count = 0U;
static uint8_t ball_balance_velocity_initialized = 0U;
static float ball_balance_velocity_last_position = 0.0f;
static uint32_t ball_balance_velocity_last_tick_ms = 0U;

// 新灰度模块的单字节查询命令；末尾0供现有usart6_send()计算长度。
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
 * @brief 在底盘控制循环中周期调用，最多每100 ms刷新一次OLED。
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
 * @brief 启动TIM4球杆位置控制。
 * @param target_position 视觉坐标系中的目标位置，单位pixel。
 * @param tolerance_pixel 当前目标的到达判断半宽，单位pixel。
 * @note TIM4已在main()中启动，本函数只打开控制开关并复位位置环状态。
 */
void BallBalanceControl_Start(float target_position,
                              float tolerance_pixel)
{
    ball_balance_target_position = target_position;
    ball_balance_target_tolerance_pixel = tolerance_pixel;
    ball_balance_target_in_range_count = 0U;
    ball_balance_last_packet_count = point_packet_rx_count;
    PID_DM_Pitch_Position.Error = 0.0f;
    PID_DM_Pitch_Position.Integral = 0.0f;
    PID_DM_Pitch_Position.IntegralOutput = 0.0f;
    PID_DM_Pitch_Position.Differential = 0.0f;
    PID_DM_Pitch_Position.Output = 0.0f;
    PID_DM_Pitch_Position.Error_Last1 = 0.0f;
    ball_balance_raw_velocity_pixel_s = 0.0f;
    ball_balance_filtered_velocity_pixel_s = 0.0f;
    ball_balance_velocity_feedback_angle_rad = 0.0f;
    ball_balance_rod_target_angle_rad = 0.0f;
    ball_balance_velocity_initialized = 0U;
    ball_balance_velocity_last_position = 0.0f;
    ball_balance_velocity_last_tick_ms = HAL_GetTick();
    ball_balance_control_enabled = 1U;
}

/**
 * @brief 停止TIM4球杆闭环，并让球杆回到中立角度。
 */
void BallBalanceControl_Stop(void)
{
    ball_balance_control_enabled = 0U;
    ball_balance_target_in_range_count = 0U;
    PID_DM_Pitch_Position.Error = 0.0f;
    PID_DM_Pitch_Position.Integral = 0.0f;
    PID_DM_Pitch_Position.IntegralOutput = 0.0f;
    PID_DM_Pitch_Position.Differential = 0.0f;
    PID_DM_Pitch_Position.Output = 0.0f;
    PID_DM_Pitch_Position.Error_Last1 = 0.0f;
    ball_balance_raw_velocity_pixel_s = 0.0f;
    ball_balance_filtered_velocity_pixel_s = 0.0f;
    ball_balance_velocity_feedback_angle_rad = 0.0f;
    ball_balance_rod_target_angle_rad = 0.0f;
    ball_balance_velocity_initialized = 0U;
    DM_MitControl(DM_PITCH_TX_ID, MOTOR_ENABLE,
                  BALL_BALANCE_MOTOR_ZERO_ANGLE_RAD,
                  0.0f, 2.0f, 0.1f, 0.0f);
}

/**
 * @brief 在控制运行期间修改小球目标位置。
 * @param target_position 新目标位置，单位pixel。
 * @param tolerance_pixel 新目标的到达判断半宽，单位pixel。
 * @note 本函数由主循环调用；会清除旧目标的到达标志和积分，
 *       但保留视觉速度滤波状态，避免切换目标时失去速度阻尼。
 */
void BallBalanceControl_SetTarget(float target_position,
                                  float tolerance_pixel)
{
    ball_balance_target_position = target_position;
    ball_balance_target_tolerance_pixel = tolerance_pixel;
    ball_balance_target_in_range_count = 0U;
    PID_DM_Pitch_Position.Integral = 0.0f;
    PID_DM_Pitch_Position.IntegralOutput = 0.0f;
}

/**
 * @brief 控制达妙Pitch电机返回电机零点。
 * @note 本函数会阻塞，直至位置连续10次位于[-0.05, 0.05] rad。
 *       未收到有效Pitch反馈时不会退出，并会每10 ms重发一次回零命令。
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
 * @brief 将BC球杆相对水平位置的目标角度反解为AD电机相对零点的目标角度。
 * @param ball_rod_angle BC球杆目标角度，单位rad；逆时针为正。
 * @retval AD电机相对水平零点的目标角度，单位rad；逆时针为正。
 * @note 使用AD=40、AB=90、BC=180、CD=200 mm的四杆闭环几何。
 *       输入超出当前装配支路的几何范围时会先限幅；本函数无循环和阻塞。
 */
static float BallRodAngleToMotorAngle(float ball_rod_angle)
{
    const float zero_horizontal_distance =
        FOUR_BAR_BC_LENGTH_MM - FOUR_BAR_AD_LENGTH_MM;
    const float fixed_point_c_x =
        (zero_horizontal_distance * zero_horizontal_distance +
         FOUR_BAR_CD_LENGTH_MM * FOUR_BAR_CD_LENGTH_MM -
         FOUR_BAR_AB_LENGTH_MM * FOUR_BAR_AB_LENGTH_MM) /
        (2.0f * zero_horizontal_distance);
    const float fixed_point_c_y =
        sqrtf(FOUR_BAR_CD_LENGTH_MM * FOUR_BAR_CD_LENGTH_MM -
              fixed_point_c_x * fixed_point_c_x);
    float point_b_x;
    float point_b_y;
    float distance_db;
    float acos_input;

    if (ball_rod_angle > BALL_ROD_GEOMETRY_MAX_ANGLE_RAD)
    {
        ball_rod_angle = BALL_ROD_GEOMETRY_MAX_ANGLE_RAD;
    }
    else if (ball_rod_angle < BALL_ROD_GEOMETRY_MIN_ANGLE_RAD)
    {
        ball_rod_angle = BALL_ROD_GEOMETRY_MIN_ANGLE_RAD;
    }

    /*
     * 以D为原点，图示CD朝右上方；BC水平时B位于C左侧。
     * 球杆正向逆时针转动时，B点沿C点左下方运动。
     */
    point_b_x =
        fixed_point_c_x - FOUR_BAR_BC_LENGTH_MM * cosf(ball_rod_angle);
    point_b_y =
        fixed_point_c_y - FOUR_BAR_BC_LENGTH_MM * sinf(ball_rod_angle);
    distance_db = sqrtf(point_b_x * point_b_x + point_b_y * point_b_y);

    /*
     * A点同时位于以D为圆心、AD为半径的圆和以B为圆心、
     * AB为半径的圆上。选择经过图示水平零位的装配支路。
     */
    acos_input =
        (FOUR_BAR_AB_LENGTH_MM * FOUR_BAR_AB_LENGTH_MM -
         FOUR_BAR_AD_LENGTH_MM * FOUR_BAR_AD_LENGTH_MM -
         distance_db * distance_db) /
        (2.0f * FOUR_BAR_AD_LENGTH_MM * distance_db);

    // 浮点舍入可能使几何极限处略超出[-1, 1]，必须夹紧后再反余弦。
    if (acos_input > 1.0f)
    {
        acos_input = 1.0f;
    }
    else if (acos_input < -1.0f)
    {
        acos_input = -1.0f;
    }

    return atan2f(point_b_y, point_b_x) - acosf(acos_input);
}

/**
 * @brief 根据相邻视觉帧计算并滤波小球速度，再转换为球杆阻尼角。
 * @param current_position 当前小球位置，单位pixel。
 * @retval 速度反馈角，单位rad；方向与小球运动方向相反。
 * @note 首帧以及帧间隔大于100 ms时速度置零，仅重置速度估计，
 *       不会关闭球杆控制，也不会让球杆自动回中。
 */
static float BallVelocityFeedbackUpdate(float current_position)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t interval_ms;
    float interval_s;
    float filter_alpha;
    float feedback_angle;

    if (ball_balance_velocity_initialized == 0U)
    {
        ball_balance_velocity_last_position = current_position;
        ball_balance_velocity_last_tick_ms = now_ms;
        ball_balance_velocity_initialized = 1U;
        ball_balance_raw_velocity_pixel_s = 0.0f;
        ball_balance_filtered_velocity_pixel_s = 0.0f;
        return 0.0f;
    }

    interval_ms = now_ms - ball_balance_velocity_last_tick_ms;
    if ((interval_ms == 0U) ||
        (interval_ms > BALL_VELOCITY_MAX_VALID_INTERVAL_MS))
    {
        // 帧间隔异常时丢弃本次速度，防止恢复视觉后出现速度尖峰。
        ball_balance_raw_velocity_pixel_s = 0.0f;
        ball_balance_filtered_velocity_pixel_s = 0.0f;
    }
    else
    {
        interval_s = (float)interval_ms * 0.001f;
        ball_balance_raw_velocity_pixel_s =
            (current_position - ball_balance_velocity_last_position) /
            interval_s;

        // alpha随实际视觉帧间隔变化，帧率波动时滤波强度更稳定。
        filter_alpha =
            interval_s /
            (BALL_VELOCITY_FILTER_TIME_CONSTANT_S + interval_s);
        ball_balance_filtered_velocity_pixel_s +=
            filter_alpha *
            (ball_balance_raw_velocity_pixel_s -
             ball_balance_filtered_velocity_pixel_s);
    }

    ball_balance_velocity_last_position = current_position;
    ball_balance_velocity_last_tick_ms = now_ms;

    // 对测量速度取负反馈，避免目标位置变化产生微分冲击。
    feedback_angle =
        -ball_balance_velocity_kv *
        ball_balance_filtered_velocity_pixel_s;
    if (feedback_angle > BALL_VELOCITY_FEEDBACK_LIMIT_RAD)
    {
        feedback_angle = BALL_VELOCITY_FEEDBACK_LIMIT_RAD;
    }
    else if (feedback_angle < -BALL_VELOCITY_FEEDBACK_LIMIT_RAD)
    {
        feedback_angle = -BALL_VELOCITY_FEEDBACK_LIMIT_RAD;
    }

    return feedback_angle;
}

/**
 * @brief 根据球的位置误差计算达妙Pitch电机的目标角度。
 * @param target_position 球的目标位置，单位pixel。
 * @param current_position 球的当前位置，单位pixel。
 * @note 调用前必须完成PID、CAN初始化、电机使能和回零。
 *       本函数每次调用发送一帧MIT位置控制指令，不包含循环或延时。
 */
void position_control(float target_position, float current_position)
{
    float position_loop_output =
        PID_Position(&PID_DM_Pitch_Position,
                     current_position,
                     target_position,
                     BALL_BALANCE_MAX_TILT_ANGLE_RAD);
    float velocity_feedback_angle =
        BallVelocityFeedbackUpdate(current_position);
    float ball_rod_target_angle =
        position_loop_output + velocity_feedback_angle;
    float motor_angle_offset;
    float motor_target_angle;

    // 位置环与速度阻尼相加后再次限幅，保证球杆总请求角度不超过±12°。
    if (ball_rod_target_angle > BALL_BALANCE_MAX_TILT_ANGLE_RAD)
    {
        ball_rod_target_angle = BALL_BALANCE_MAX_TILT_ANGLE_RAD;
    }
    else if (ball_rod_target_angle < -BALL_BALANCE_MAX_TILT_ANGLE_RAD)
    {
        ball_rod_target_angle = -BALL_BALANCE_MAX_TILT_ANGLE_RAD;
    }

    ball_balance_velocity_feedback_angle_rad = velocity_feedback_angle;
    ball_balance_rod_target_angle_rad = ball_rod_target_angle;
    motor_angle_offset = BallRodAngleToMotorAngle(ball_rod_target_angle);

    /*
     * PID输出表示BC球杆目标角度；经四杆反解得到AD电机角度后，
     * 再叠加电机零点并通过达妙绝对电控限位[-1.1, 0.65] rad。
     */
    motor_target_angle =
        DM_pos_limit(BALL_BALANCE_MOTOR_ZERO_ANGLE_RAD +
                     motor_angle_offset);

    DM_MitControl(DM_PITCH_TX_ID, MOTOR_ENABLE,
                  motor_target_angle, 0.0f, 2.0f, 0.1f, 0.0f);
}

/**
 * @brief 执行一次底盘循迹：平滑加速、循迹至路口并停车。
 * @note 本函数在主循环上下文运行并包含阻塞循环；
 *       球杆控制可由TIM4中断并行执行。
 */
void ChassisTrack_Run(void)
{
    ChassisMotionTime_Start();
    S_regulate_track(0, speed_target, 800); // 加速到目标速度
    while (scan_cross_nostop(line) != 0)
    {
        track_dynamic_Speed(speed_target);
        ChassisMotionTime_Update();
        delay_ms(3);
    }

    // 到达路口后直接发送零速度，保留当前任务1的停车行为。
    DM_SpeedControl(DM_Chassis1_TX_ID, MOTOR_ENABLE, 0.0f);
    DM_SpeedControl(DM_Chassis2_TX_ID, MOTOR_ENABLE, 0.0f);
    ChassisMotionTime_Stop();
}

/**
 * @brief 按全局time设定的时长执行底盘循迹并平滑停车。
 * @note 本函数在主循环上下文运行并包含阻塞循环；
 *       time单位为ms，球杆控制可由TIM4中断并行执行。
 */
void ChassisTrack2_Run(void)
{
    uint32_t time_on;

    ChassisMotionTime_Start();
    S_regulate_track(0, speed_target, 800); // 加速到目标速度
    time_on = read_time();

    while (read_time() <= time_on + 1000)
    {
        track_dynamic_Speed(speed_target);
        ChassisMotionTime_Update();
        delay_ms(3);
    }

    // S_regulate_Ctl(speed_target, 0.0f, 300); // 减速停车
    ChassisMotionTime_Stop();
}

/**
 * @brief HAL定时器周期回调。
 * @note TIM3每10 ms查询一次灰度模块；TIM4只在收到新视觉帧时更新球杆控制。
 *       回调中不进行OLED刷新，避免软件I2C阻塞中断。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3)
    {
        usart6_send(grayscale_query_command);
        return;
    }

    if (htim == &htim4)
    {
        uint32_t current_packet_count;
        float current_position;
        float current_target_position;
        float current_target_tolerance_pixel;

        if (ball_balance_control_enabled == 0U)
        {
            return;
        }

        current_packet_count = point_packet_rx_count;
        if ((current_packet_count == 0U) ||
            (current_packet_count == ball_balance_last_packet_count))
        {
            // 没有新视觉数据时保持上一条电机指令，不重复计算位置环。
            return;
        }

        /*
         * UART4在完整包校验成功后先更新point_packet，再增加接收计数。
         * TIM4因此可用计数变化判断当前坐标是否为一帧新数据。
         */
        current_position = (float)point_packet.centerpoint_x;
        current_target_position = ball_balance_target_position;
        current_target_tolerance_pixel =
            ball_balance_target_tolerance_pixel;
        ball_balance_last_packet_count = current_packet_count;

        position_control(current_target_position, current_position);

        // 连续计数离开当前目标对应的像素阈值后立即清零。
        if ((current_position >=
             current_target_position - current_target_tolerance_pixel) &&
            (current_position <=
             current_target_position + current_target_tolerance_pixel))
        {
            if (ball_balance_target_in_range_count < 255U)
            {
                ball_balance_target_in_range_count++;
            }
        }
        else
        {
            ball_balance_target_in_range_count = 0U;
        }

    }
}
