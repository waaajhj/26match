#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H
#include "bsp_can.h"
#include "pid.h"
// 底盘电机到底盘中心的距离，单位m
#define MOTOR_DISTANCE_TO_CENTER 0.28f
extern float x,y,z,wz1;
#define front_wz 1000

void motor_speed_control(int16_t *speeds );
void motor_angle_control(float angle);
void Kinematic_Analysis2(float Vx,float Vy,float Vz);
void calculate_motor_speeds(float V,float W );
void motor_stop(void);
void  motor_angle_control_S(float angle,float speed);
#endif