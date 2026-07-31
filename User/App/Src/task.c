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
 * @brief 任务3：主循环执行底盘循迹，TIM4同时执行球杆闭环。
 */
void task_3(void)
{
    Task2SegmentedControl_Disable();
    Task2_RestoreOriginalParams();
    Task3SegmentedControl_Disable();
    /*
     * 两轮片上采样表明-10偏向高像素而-18会过度拉向低像素；
     * 任务3采用折中零偏-14，继续由片上数据校准且不改变任务2目标。
     */
    task3_segmented_control.target_offset_pixel = -14.0f;
    // 仅底盘0~2.5 s加速阶段使用较小Kv，降低首次回摆，随后自动恢复分段Kv。
    task3_segmented_control.startup_velocity_kv = 0.00030f;
    // 任务3启动前馈以0.60系数退出，缩短补偿残留并抑制加速结束后的高像素侧超调。
    task3_segmented_control.acceleration_release_filter_alpha = 0.60f;
    // 利用低像素侧剩余余量把启动前馈限幅降到+4°，减少前馈退出后的机械储能回摆。
    task3_segmented_control.acceleration_feedforward_limit_rad = 0.06981317f;
    // 实测Kd=0.2会扩大快速目标变化时的跟随滞后，任务3恢复原MIT速度阻尼0.1。
    task3_segmented_control.pitch_motor_kd = 0.1f;
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
    // 任务4暂不沿用任务3的中心零偏，避免任务3调参改变其他任务效果。
    task3_segmented_control.target_offset_pixel = 0.0f;
    // 任务4保持全程原近段Kv，不沿用任务3的加速阶段柔化参数。
    task3_segmented_control.startup_velocity_kv =
        task3_segmented_control.near.Kv;
    // 任务4保持原前馈退出方式，避免任务3调参改变任务4效果。
    task3_segmented_control.acceleration_release_filter_alpha = 1.0f;
    // 任务4恢复原正向前馈限幅+5°，不沿用任务3参数。
    task3_segmented_control.acceleration_feedforward_limit_rad = 0.08726646f;
    // 任务4保持原MIT速度阻尼，避免任务3电机调参改变其他任务效果。
    task3_segmented_control.pitch_motor_kd = 0.1f;
    Task3SegmentedControl_Enable();
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
