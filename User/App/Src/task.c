#include "task.h"

#include "motion_control.h"
#include "jy61p.h"
#include "pid.h"

// 正5 cm阶段按倍率提高当前Kp，之后恢复原值。
#define TASK_2_POSITIVE_KP_SCALE 1.47f

// 任务2阶段只在主循环修改，可在Keil Watch中观察当前目标阶段。
volatile Task2Stage_e task_2_stage = TASK_2_STAGE_IDLE;
static float task_2_original_kp = 0.0f;
static uint8_t task_2_positive_kp_enabled = 0U;

/**
 * @brief 恢复任务2启动前的位置环Kp。
 * @note 仅正5 cm阶段临时提高Kp，任务切换或进入第二阶段时调用。
 */
static void Task2_RestoreOriginalKp(void)
{
    if (task_2_positive_kp_enabled != 0U)
    {
        PID_DM_Pitch_Position.Kp = task_2_original_kp;
        task_2_positive_kp_enabled = 0U;
    }
}

/**
 * @brief 任务1：只运行底盘循迹，不启用球杆闭环。
 */
void task_1(void)
{
    Task2_RestoreOriginalKp();
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
    Task2_RestoreOriginalKp();
    task_2_original_kp = PID_DM_Pitch_Position.Kp;
    PID_DM_Pitch_Position.Kp =
        task_2_original_kp * TASK_2_POSITIVE_KP_SCALE;
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
 *       切换到-5 cm后不再更换目标，TIM4持续运行闭环保持平衡。
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
            // 从中心返回-5 cm的第二阶段恢复当前原始PID参数。
            Task2_RestoreOriginalKp();
            BallBalanceControl_SetTarget(
                BALL_BALANCE_DEFAULT_TARGET_POSITION,
                BALL_BALANCE_INTERMEDIATE_TOLERANCE_PIXEL);
            task_2_stage = TASK_2_STAGE_TO_CENTER;
            break;

        case TASK_2_STAGE_TO_CENTER:
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
    Task2_RestoreOriginalKp();
    task_2_stage = TASK_2_STAGE_IDLE;
    BallBalanceControl_Start(
        BALL_BALANCE_DEFAULT_TARGET_POSITION,
        BALL_BALANCE_INTERMEDIATE_TOLERANCE_PIXEL);
    // ChassisTrack2_Run();
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
            break;
        case SERIAL_SCREEN_TASK_5:
            break;
        default:
            break;
    }
}
