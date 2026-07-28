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

