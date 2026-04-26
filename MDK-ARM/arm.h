#ifndef __ARM_H
#define __ARM_H
#include "stm32f4xx_hal.h"
void Angle1(float angle);
void Angle2(float angle);
void Angle3(float angle);
void Angle4(float angle);
void Arm_Init(void);
void Arm_up(void);
void Arm_down(void);
void Arm_catch(void);
void Arm_place_3p(void);
void Arm_placeRing(void);
void Catch_craft(void);
void Arm_High(void);
void Arm_placeRing2(void);
extern int flag_catch;
#endif
