#include "task.h"

#include "motion_control.h"
#include "jy61p.h"
#include "pid.h"

// 正5 cm阶段提高位置P并降低速度阻尼，兼顾到达速度和目标精度。
#define TASK_2_POSITIVE_KP_SCALE 1.65f // 降低+5 cm转向前惯性，同时保持快速到达
#define TASK_2_POSITIVE_KV 0.00020f

// 任务2阶段只在主循环修改，可在Keil Watch中观察当前目标阶段。
volatile Task2Stage_e task_2_stage = TASK_2_STAGE_IDLE;
static float task_2_original_kp = 0.0f;
static float task_2_original_kv = 0.0f;
static uint8_t task_2_positive_kp_enabled = 0U;

/**
 * @brief 恢复任务2启动前的位置P和速度反馈系数。
 * @note 仅正5 cm阶段临时修改参数，任务切换或进入第二阶段时调用。
 */
static void Task2_RestoreOriginalParams(void)
{
    if (task_2_positive_kp_enabled != 0U)
    {
        PID_DM_Pitch_Position.Kp = task_2_original_kp;
        ball_balance_velocity_kv = task_2_original_kv;
        task_2_positive_kp_enabled = 0U;
    }
}

/**
 * @brief 任务1：只运行底盘循迹，不启用球杆闭环。
 */
void task_1(void)
{
    Task2SegmentedControl_Disable();
    Task3SegmentedControl_Disable();
    Task2_RestoreOriginalParams();
    task_2_stage = TASK_2_STAGE_IDLE;
    BallBalanceControl_Stop();
    ChassisTrack_Run();
    serial_screen_task = SERIAL_SCREEN_TASK_NONE;
}

/**
 * @brief 任务2：底盘保持静止，只运行TIM4球杆闭环。
 */
void task_2(void)
{
    Task2SegmentedControl_Disable();
    Task3SegmentedControl_Disable();
    Task2_RestoreOriginalParams();
    task_2_original_kp = PID_DM_Pitch_Position.Kp;
    task_2_original_kv = ball_balance_velocity_kv;
    PID_DM_Pitch_Position.Kp =
        task_2_original_kp * TASK_2_POSITIVE_KP_SCALE;
    ball_balance_velocity_kv = TASK_2_POSITIVE_KV;
    task_2_positive_kp_enabled = 1U;

    // 从任务2发令时开始记录视觉、PID、速度反馈和电机状态到共享RAM缓存。
    Task2DebugRecorder_Start();
    BallBalanceControl_Start(
        BALL_BALANCE_POSITIVE_5CM_TARGET_POSITION,
        BALL_BALANCE_INTERMEDIATE_TOLERANCE_PIXEL);
    task_2_stage = TASK_2_STAGE_TO_POSITIVE_5CM;
    // 防止主循环重复启动同一任务。
    serial_screen_task = SERIAL_SCREEN_TASK_NONE;
}

/**
 * @brief 在主循环中推进任务2的目标点序列。
 * @note TIM4负责更新连续到达计数；本函数无阻塞行为。
 *       到达+5 cm后直接切换到-5 cm，避免在中心停顿影响总时间。
 *       切换后启用任务2专用分段PID，并持续闭环保持最终位置。
 */
void task_2_update(void)
{
    if (ball_balance_target_in_range_count == 0U)
    {
        return;
    }

    switch (task_2_stage)
    {
        case TASK_2_STAGE_TO_POSITIVE_5CM:
            // 直接进入-5 cm阶段，省略中心目标以缩短任务时间。
            Task2_RestoreOriginalParams();
            Task2SegmentedControl_Enable();
            BallBalanceControl_SetTarget(
                BALL_BALANCE_NEGATIVE_5CM_TARGET_POSITION,
                BALL_BALANCE_FINAL_TOLERANCE_PIXEL);
            task_2_stage = TASK_2_STAGE_TO_NEGATIVE_5CM;
            break;

        case TASK_2_STAGE_TO_NEGATIVE_5CM:
            // 最终目标不再切换，位置环和速度反馈持续稳定小球。
            break;

        default:
            break;
    }
}

/**
 * @brief 加载任务3的球杆分段控制参数。
 * @note 本函数无参数和返回值；调用前必须先关闭任务3分段控制，避免TIM4在
 *       参数写入过程中读取到一半新一半旧的配置。本函数不包含循环、延时或
 *       通信，不会启动电机；角度限幅和刹车限幅均保留任务3的安全设置。
 */
static void Task3_BallControlParamsApply(void)
{
    // 分段阈值、视觉速度滤波和五段位置/速度反馈参数。
    task3_segmented_control.near_error_limit_pixel = 30.0f;
    task3_segmented_control.middle_error_limit_pixel = 100.0f;
    task3_segmented_control.velocity_filter_time_constant_s = 0.040f;
    task3_segmented_control.near_velocity_deadband_pixel_s = 15.0f;
    // 任务3运动全过程采用非对称速度阻尼，减小低像素回摆后的高侧反弹。
    task3_segmented_control.low_direction_velocity_kv_enabled = 1U;

    task3_segmented_control.near.Kp = 0.00028f;
    task3_segmented_control.near.Ki = 0.0f;
    task3_segmented_control.near.Kv = 0.00038f; // 实测0.00034未减小振幅，恢复原近段速度阻尼
    task3_segmented_control.low_pixel_middle.Kp = 0.00026f;
    task3_segmented_control.low_pixel_middle.Ki = 0.0f;
    task3_segmented_control.low_pixel_middle.Kv = 0.00030f;
    task3_segmented_control.low_pixel_far.Kp = 0.00028f;
    task3_segmented_control.low_pixel_far.Ki = 0.0f;
    task3_segmented_control.low_pixel_far.Kv = 0.00024f;
    task3_segmented_control.high_pixel_middle.Kp = 0.00026f;
    task3_segmented_control.high_pixel_middle.Ki = 0.0f;
    task3_segmented_control.high_pixel_middle.Kv = 0.00024f;
    task3_segmented_control.high_pixel_far.Kp = 0.00028f;
    task3_segmented_control.high_pixel_far.Ki = 0.0f;
    task3_segmented_control.high_pixel_far.Kv = 0.00020f;

    /*
     * 两轮片上采样表明-10偏向高像素而-18会过度拉向低像素；
     * 任务3采用折中零偏-14，且不改变任务2目标。
     */
    task3_segmented_control.target_offset_pixel = -14.0f;
    // 加速时向低像素滑动使用较小Kv，减小电机目标大幅反向。
    task3_segmented_control.startup_velocity_kv = 0.00030f;
    // 提前撤除仍向高像素侧推球的正向前馈，给高速回摆保留制动距离。
    task3_segmented_control.startup_feedforward_cutoff_start_pixel = 232.0f;
    task3_segmented_control.startup_feedforward_cutoff_velocity_pixel_s = 10.0f;
    // 高像素超过240 pixel且仍向高侧运动时，提前介入任务3过渡期软边界。
    task3_segmented_control.transition_high_brake_start_pixel = 240.0f;
    // 实测0.0016会激起更大的反向回摆，恢复匀速段更平稳的0.0013。
    task3_segmented_control.transition_high_brake_gain_rad_per_pixel = 0.0013f;
    task3_segmented_control.transition_high_brake_limit_rad = 0.01221730f; // 最大附加制动0.7°
    // 实测0.35会因前馈残留扩大高侧回摆，恢复效果更好的0.45退出系数。
    task3_segmented_control.acceleration_release_filter_alpha = 0.45f;
    // 3 s缓加速后高侧仍偏大，任务3单独降低前馈增益，将启动峰值角约降至1.32°。
    task3_segmented_control.acceleration_feedforward_gain = 0.00330f;
    // 恢复实测低像素侧保护更好的任务3正向启动前馈限幅+4.5°。
    task3_segmented_control.acceleration_feedforward_limit_rad = 0.07853982f;
    task3_segmented_control.acceleration_brake_feedforward_limit_rad =
        0.10471976f;
    task3_segmented_control.acceleration_filter_alpha = 1.0f;
    task3_segmented_control.brake_release_filter_alpha = 0.65f;
    task3_segmented_control.pitch_motor_kp = 3.5f;
    // 实测Kd=0.2会扩大快速目标变化时的跟随滞后，任务3恢复原MIT速度阻尼0.1。
    task3_segmented_control.pitch_motor_kd = 0.1f;
}

/**
 * @brief 加载任务4专用的球杆分段控制参数，初始数值完整复制自任务3。
 * @note 本函数无参数和返回值；调用前必须先关闭任务3分段控制，避免TIM4在
 *       参数写入过程中读取到不完整配置。本函数不包含循环、延时或通信，
 *       不会启动电机；任务4后续调参只修改本函数，不会影响任务3。
 */
static void Task4_BallControlParamsApply(void)
{
    // 任务4独立保存分段阈值、视觉速度滤波和五段位置/速度反馈参数。
    task3_segmented_control.near_error_limit_pixel = 30.0f;
    task3_segmented_control.middle_error_limit_pixel = 100.0f;
    task3_segmented_control.velocity_filter_time_constant_s = 0.040f;
    task3_segmented_control.near_velocity_deadband_pixel_s = 15.0f;
    // 任务4仅同步启动保护，匀速阶段暂不启用任务3新加入的非对称阻尼。
    task3_segmented_control.low_direction_velocity_kv_enabled = 0U;

    task3_segmented_control.near.Kp = 0.00028f;
    task3_segmented_control.near.Ki = 0.0f;
    task3_segmented_control.near.Kv = 0.00038f;
    task3_segmented_control.low_pixel_middle.Kp = 0.00026f;
    task3_segmented_control.low_pixel_middle.Ki = 0.0f;
    task3_segmented_control.low_pixel_middle.Kv = 0.00030f;
    task3_segmented_control.low_pixel_far.Kp = 0.00028f;
    task3_segmented_control.low_pixel_far.Ki = 0.0f;
    task3_segmented_control.low_pixel_far.Kv = 0.00024f;
    task3_segmented_control.high_pixel_middle.Kp = 0.00026f;
    task3_segmented_control.high_pixel_middle.Ki = 0.0f;
    task3_segmented_control.high_pixel_middle.Kv = 0.00024f;
    task3_segmented_control.high_pixel_far.Kp = 0.00028f;
    task3_segmented_control.high_pixel_far.Ki = 0.0f;
    task3_segmented_control.high_pixel_far.Kv = 0.00020f;

    task3_segmented_control.target_offset_pixel = -14.0f;
    task3_segmented_control.startup_velocity_kv = 0.00030f;
    // 任务4当前同步任务3的启动前馈撤除策略，但仍在本函数中独立保存数值。
    task3_segmented_control.startup_feedforward_cutoff_start_pixel = 235.0f;
    task3_segmented_control.startup_feedforward_cutoff_velocity_pixel_s = 15.0f;
    task3_segmented_control.transition_high_brake_start_pixel = 240.0f;
    task3_segmented_control.transition_high_brake_gain_rad_per_pixel = 0.0013f;
    task3_segmented_control.transition_high_brake_limit_rad = 0.01221730f;
    task3_segmented_control.acceleration_release_filter_alpha = 0.45f;
    task3_segmented_control.acceleration_feedforward_gain = 0.00330f;
    task3_segmented_control.acceleration_feedforward_limit_rad = 0.07853982f;
    task3_segmented_control.acceleration_brake_feedforward_limit_rad =
        0.10471976f;
    task3_segmented_control.acceleration_filter_alpha = 1.0f;
    task3_segmented_control.brake_release_filter_alpha = 0.65f;
    task3_segmented_control.pitch_motor_kp = 3.5f;
    task3_segmented_control.pitch_motor_kd = 0.1f;
}

/**
 * @brief 任务3：主循环执行底盘循迹，TIM4同时执行球杆闭环。
 */
void task_3(void)
{
    Task2SegmentedControl_Disable();
    Task2_RestoreOriginalParams();
    Task3SegmentedControl_Disable();
    Task3_BallControlParamsApply();
    Task3SegmentedControl_Enable();
    // 仅任务3启用内部高速记录，任务2和任务4不会写入该缓存。
    Task3DebugRecorder_Start();
    task_2_stage = TASK_2_STAGE_IDLE;
    BallBalanceControl_Start(
        BALL_BALANCE_DEFAULT_TARGET_POSITION,
        0.0f); // 任务3无到达阈值，TIM4始终执行五段闭环。
    ChassisTrack2_Run();
    serial_screen_task = SERIAL_SCREEN_TASK_NONE;
}
void task_4(void)
{ 
    Task2SegmentedControl_Disable();
    Task2_RestoreOriginalParams();
    Task3SegmentedControl_Disable();
    // 当前数值复制自任务3，但由任务4专用函数独立维护，后续调参互不影响。
    Task4_BallControlParamsApply();
    Task3SegmentedControl_Enable();
    // 任务4与任务2/3复用RAM采样区，新任务启动时覆盖上一轮数据。
    Task4DebugRecorder_Start();
    task_2_stage = TASK_2_STAGE_IDLE;
    BallBalanceControl_Start(
        BALL_BALANCE_DEFAULT_TARGET_POSITION,
        0.0f); // 任务4无到达阈值，TIM4始终执行五段闭环。
    ChassisTrack3_Run();
    serial_screen_task = SERIAL_SCREEN_TASK_NONE;

}

/**
 * @brief 根据串口屏发送的任务号执行对应任务。
 * @note 本函数在主循环调用；任务1和任务3包含底盘运动阻塞过程，
 *       但运行期间TIM4仍可中断并执行球杆控制。
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
            task_4();
            break;
        case SERIAL_SCREEN_TASK_5:
            break;
        default:
            break;
    }
}
