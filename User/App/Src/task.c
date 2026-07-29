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

// 球杆平衡时电机的中立位置，以达妙电机保存的零点为基准，单位 rad。
#define BALL_BALANCE_MOTOR_ZERO_ANGLE_RAD 0.0f

// PID 最多允许球杆相对中立位置倾斜 5°，防止视觉误差异常时目标角度过大。
#define BALL_BALANCE_MAX_TILT_ANGLE_RAD 0.08726646f

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
 * @brief 根据球的位置误差控制达妙 Pitch 电机的目标角度。
 * @param target_position 球的目标位置，单位与视觉位置数据一致。
 * @param current_position 球的当前位置，单位与视觉位置数据一致。
 * @note 调用前必须完成 PID_Init()、CAN 初始化、电机使能和回零。
 *       建议以固定周期调用；本函数每次调用会发送一帧 MIT 位置控制指令。
 */
void position_control(float target_position, float current_position)
{
    /*
     * PID 输出表示球杆相对中立位置的目标倾角，并限制在 ±5°。
     * 保留原控制方向的负号：current_position 大于 target_position 时，
     * PID 内部误差为负，取反后电机目标角度增大，即逆时针转动。
     */
    float angle_offset = -PID_Position(&PID_DM_Pitch_Position,
                                       current_position,
                                       target_position,
                                       BALL_BALANCE_MAX_TILT_ANGLE_RAD);

    /*
     * 最终目标角度先叠加平衡零点，再经过达妙绝对电控限位，
     * 因此同时受到相对倾角 ±5° 和绝对角度 [-0.7, 1.04] rad 的保护。
     */
    float motor_target_angle =
        DM_pos_limit(BALL_BALANCE_MOTOR_ZERO_ANGLE_RAD + angle_offset);

    DM_MitControl(DM_PITCH_TX_ID, MOTOR_ENABLE,
                  motor_target_angle, 0.0f, 2.0f, 0.1f, 0.0f);
}
