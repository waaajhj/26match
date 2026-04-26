#include "DJI_Motor.h"
#include "sensor.h"
#include "jy61p.h"
// ==============================================================================
//  自车体正方向的右前轮开始，逆时针电机编号依次从0递增
// ==============================================================================
float x,y,z;
//void Kinematic_Analysis2(float Vx,float Vy,float Vz)
//{
//	int16_t speeds[4];
//    calculate_motor_speeds(Vx, Vy, Vz, speeds);
//    motor_speed_control(speeds);
//}
void calculate_motor_speeds(float V,float W)
{
    int16_t speeds[4];
    
    speeds[0] = -V+W; // Speed_A
    speeds[1] = V+W; // Speed_B
    speeds[2] = V+W; // Speed_C
    speeds[3] = -V+W; // Speed_D
    motor_speed_control(speeds);
//    speeds[0] = -Vy - Vx - common_part_3; // Speed_A
//    speeds[1] = -Vy + Vx - common_part_3; // Speed_B
//    speeds[2] = Vy + Vx - common_part_3; // Speed_C
//    speeds[3] = Vy - Vx - common_part_3; // Speed_D
}
void motor_stop(void)
{
	while(chassis_motor[0].speed_rpm>100||chassis_motor[0].speed_rpm<-100
	&&chassis_motor[1].speed_rpm>100||chassis_motor[1].speed_rpm<-100
	&chassis_motor[2].speed_rpm>100||chassis_motor[2].speed_rpm<-100
	&&chassis_motor[3].speed_rpm>100||chassis_motor[3].speed_rpm<-100){
		calculate_motor_speeds(0,0);
	}
    CAN_cmd_chassis(0,0,0,0);
}
void motor_speed_control(int16_t *speeds ){
	
	PIDCalculate(&Motor1SpeedPID, speeds[0], chassis_motor[0].speed_rpm);
    PIDCalculate(&Motor2SpeedPID, speeds[1], chassis_motor[1].speed_rpm);
    PIDCalculate(&Motor3SpeedPID, speeds[2], chassis_motor[2].speed_rpm);
    PIDCalculate(&Motor4SpeedPID, speeds[3], chassis_motor[3].speed_rpm);
	CAN_cmd_chassis(Motor1SpeedPID.Output,Motor2SpeedPID.Output,Motor3SpeedPID.Output,Motor4SpeedPID.Output);
	HAL_Delay(3);
}
void  motor_angle_control(float angle){
    float wz1,yaw_caculate;
    while(Yaw<=angle-2.5||Yaw>=angle+2.5){
        wz1=PID_Angle_Position(&PID_straight, Yaw, angle, 3000);
        if(wz1>0){
           calculate_motor_speeds(0,-(wz1+front_wz));
        }
        else if(wz1<0){
            calculate_motor_speeds(0,front_wz-wz1);
        }
    }
    motor_stop();
}
void  motor_angle_control_S(float angle,float speed){
    float wz1,yaw_caculate;
    while(Yaw<=angle-2.5||Yaw>=angle+2.5){
        wz1=PID_Angle_Position(&PID_straight, Yaw, angle, 3000);
        if(wz1>0){
           calculate_motor_speeds(speed,-(wz1+front_wz));
        }
        else if(wz1<0){
            calculate_motor_speeds(speed,front_wz-wz1);
        }
    }
    motor_stop();
}

