#ifndef TASK_H_
#define TASK_H_
#include "stm32f4xx_hal.h"

void DM_Pitch_ReturnZero(void);

/**
 * @brief 根据球的位置误差控制达妙 Pitch 电机的目标角度。
 * @param target_position 球的目标位置，单位与视觉位置数据一致。
 * @param current_position 球的当前位置，单位与视觉位置数据一致。
 * @note 调用前必须完成 PID_Init()、CAN 初始化、电机使能和回零。
 *       函数会发送 MIT 位置控制指令，电机相对中立位置最多倾斜 ±5°。
 */
void position_control(float target_position, float current_position);

#endif /* TASK_H_ */
