#include "task.h"

#include "motion_control.h"
#include "jy61p.h"

/**
 * @brief 任务1：只运行底盘循迹，不启用球杆闭环。
 */
void task_1(void)
{
    BallBalanceControl_Stop();
    ChassisTrack_Run();
    serial_screen_task = SERIAL_SCREEN_TASK_NONE;
}

/**
 * @brief 任务2：底盘保持静止，只运行TIM4球杆闭环。
 */
void task_2(void)
{
    BallBalanceControl_Start(BALL_BALANCE_DEFAULT_TARGET_POSITION);
    // 防止主循环重复启动同一任务。
    serial_screen_task = SERIAL_SCREEN_TASK_NONE;
}

/**
 * @brief 任务3：主循环执行底盘循迹，TIM4同时执行球杆闭环。
 */
void task_3(void)
{
    BallBalanceControl_Start(BALL_BALANCE_DEFAULT_TARGET_POSITION);
    ChassisTrack2_Run();
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
