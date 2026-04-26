#include "route.h"
#include "task.h"
#include "main.h"
#include "pid.h"
#include "sensor.h"
#include "oled.h"
#include "jy61p.h"
#include "arm.h"
#include "chassis.h"
#include "DJI_Motor.h"
#include "bsp_dwt.h"
#include "maixcam.h"
// 起始部分函数
// 开始：通过手势启动
// 结束：成功通过梯形山到达路口
// 退出速度为（4000）
uint8_t light_flag = 1;
void begin_part(void)
{
	down_low_stair();//下1号台
	// if(light_flag){set_color(GREEN);}
	
	while(scan_stair()){track_dynamic_Speed(8000);}//循迹到长桥
	
	// if(light_flag){set_color(RED);}
	climb_moutain();//过长桥	
//	motor_stop();
	// if(light_flag){set_color(GREEN);}
  
	while(scan_stair()){track_dynamic_Speed(8000);}

	// if(light_flag){set_color(BLACK);}
	updown_low_stair();//上下2号台
	// if(light_flag){set_color(GREEN);}
	
	while(scan_cross(line,8000,200)){track_dynamic_Speed(8000);}//循迹到路口
	motor_stop();
	delay_ms(200);
//S_regulate_track(8000, 4000, 100);
	turn_right(45);//右转45
	S_regulate_track(0,6000,300);//起步
	while(scan_stair()){track_dynamic_Speed(6000);}//循迹到障碍
////	
////	// if(light_flag){set_color(RED);}
	obstacle_stair();//过梯形山
//	// if(light_flag){set_color(GREEN);}
    while(scan_cross(line,8000,250)){track_dynamic_Speed(8000);}//循迹到路口
		motor_stop();
	  delay_ms(200);
    turn_right(135);//右转135
}
// 去往3号平台函数
// 开始：通过begin_part
// 结束：右转面向门
// 退出速度为（0）
uint8_t circuit_flag = 0,circuit_flag2 = 0;

void goto_3_part(void)
{
	S_regulate_track(0,5000,300);//起步
	S_regulate_track(5000,8000,300);//起步
	S_regulate_track(8000,15000,500);//起步
	track_highSpeed_count(500);//高速循迹一段
	S_regulate_track(15000,10000,300);//起步
	while(scan_stair()){track_dynamic_Speed(10000);}
	
	// if(light_flag){set_color(RED);}
	updown_low_stair();//上下4号台
	// if(light_flag){set_color(GREEN);}
	
	S_regulate_track(8000,13000,300);
//	track_highSpeed_count(300);
	S_regulate_track(13000,8000,300);
	while(scan_cross(line,8000,400)){track_dynamic_Speed(8000);};
	motor_stop();
	// while(scan_stair()){track_dynamic_Speed(10000);}
	
	// // if(light_flag){set_color(RED);}
	// updown_low_stair();//上下3号台
	// if(light_flag){set_color(GREEN);}
	
	// if(circuit_flag != 4){
	// 	S_regulate_track(10000,15000,500);
	// 	S_regulate_track(15000,10000,300);
	// 	while(scan_cross(line,10000,300)){track_dynamic_Speed(10000);};//循迹到路口
	// }
	// else{
	// 	S_regulate_track(10000,15000,500);
	// 	track_highSpeed_count(1200);//高速循迹一段
	// 	S_regulate_track(15000,10000,300);
	// 	while(scan_cross(line,10000,250)){track_dynamic_Speed(10000);};//循迹到路口
	// }
}
// 穿门函数（前半程）
// 开始：通过goto_31_part或者goto_43_part
// 结束：到达路口正对梯形山
// 退出速度为（0）
// 现在只有1门开的情况
void across_door_goto(void)
{
if(circuit_flag == 0)
{
	// if(light_flag){set_color(RED);}
	//看1号门
	turn_right(93);
	delay_ms(100);
	vision_start();
	delay_ms(100);
	//确定红绿灯(1红2绿)
	if(vision_choose_cross()){
		circuit_flag = 1;
		vision_stop();
		delay_ms(100);
		// if(light_flag){set_color(GREEN);}
		S_regulate_track(0,8000,500);//起步
		while(scan_stair()){track_dynamic_Speed(8000);};//循迹到门
		
		door();//过门
		turn_right(90);//右转90
		// S_regulate_track_left(0,12000,400);
		return;//退出
	}
//	vision_stop();
//	delay_ms(100);
	//换2号门
	turn_left(50);
	delay_ms(100);
	
	vision_start();
	delay_ms(100);
	//确定红绿灯(1红2绿)
	if(vision_choose_cross()){
		circuit_flag = 2;
		vision_stop();
		delay_ms(100);
//		if(light_flag){set_color(GREEN);}
		S_regulate_track(0,8000,500);//起步
		while(scan_stair()){track_dynamic_Speed(8000);}//循迹到门
		
		door_nostop();//过门
		
		S_regulate_track(door_nostop_exit_speed,13000,500);//起步
		S_regulate_track(13000,10000,300);//起步
		while(scan_cross(line,10000,200)){track_dynamic_Speed(10000);}//循迹到路口
		delay_ms(100);
		turn_right(140);//左转90
		S_regulate_track_left(0,12000,400);
		S_regulate_track_left(12000,8000,200);
		while(scan_cross(line,8000,200)){track_dynamic_Speed(8000);}	
		return;//退出
	}
	circuit_flag = 4;
	vision_stop();
	delay_ms(100);
//	if(light_flag){set_color(GREEN);}
	
	//换4号门
	turn_left(40);
	delay_ms(100);
	
	S_regulate_track(0,10000,800);//起步
	S_regulate_track(10000,15000,500);//起步
	S_regulate_track(15000,10000,500);//起步
	while(scan_cross(line,10000,200)){track_dynamic_Speed(10000);}//循迹到路口
	
	turn_right(90);
	S_regulate_track(0,8000,500);//起步
	while(scan_stair()){track_dynamic_Speed(8000);}//循迹到门		
	
	door();//过门
	
	turn_right(90);//右转90
	S_regulate_track(0,12000,500);
	S_regulate_track(12000,8000,300);
	while(scan_cross(line,8000,200)){track_dynamic_Speed(8000);}
}
// 第二圈
else if(circuit_flag == 1){
		//看1号门
		// if(light_flag){set_color(GREEN);}
		turn_right(93);
		S_regulate_track(0,8000,500);//起步
		while(scan_stair()){track_dynamic_Speed(8000);};//循迹到门
		
		door();//过门
		turn_right(90);//右转90
		// S_regulate_track_left(0,12000,400);
		return;//退出
		
	}
else if(circuit_flag == 2){
		turn_right(40);
	    delay_ms(100);
		S_regulate_track(0,8000,500);//起步
		while(scan_stair()){track_dynamic_Speed(8000);}//循迹到门
		
		door_nostop();//过门
		
		S_regulate_track(door_nostop_exit_speed,13000,500);//起步
		S_regulate_track(13000,10000,300);//起步
		while(scan_cross(line,10000,200)){track_dynamic_Speed(10000);}//循迹到路口
		delay_ms(100);
		turn_right(140);//左转90
		S_regulate_track_left(0,12000,400);
		S_regulate_track_left(12000,8000,200);
		while(scan_cross(line,8000,200)){track_dynamic_Speed(8000);}	
		return;//退出
}
else if(circuit_flag == 4){
	S_regulate_track(0,10000,800);//起步
	S_regulate_track(10000,15000,500);//起步
	S_regulate_track(15000,10000,500);//起步
	while(scan_cross(line,10000,200)){track_dynamic_Speed(10000);}//循迹到路口
	
	turn_right(90);
	S_regulate_track(0,8000,500);//起步
	while(scan_stair()){track_dynamic_Speed(8000);}//循迹到门		
	
	door();//过门
	
	turn_right(90);//右转90
	S_regulate_track(0,12000,500);
	S_regulate_track(12000,8000,300);
	while(scan_cross(line,8000,200)){track_dynamic_Speed(8000);}
		// S_regulate_track_left(0,12000,500);
		// S_regulate_track_left(12000,8000,300);
		// while(scan_cross(line,8000,200)){track_dynamic_Speed(8000);}
		// turn_left(130);//左转90
		
		return;//退出
}
	
}
// 穿门函数（后半程）
// 开始：通过goto_65_part
// 结束：到达路口正对三连障碍
// 退出速度为（0）
// 现在只有4门开的情况
void across_door_return(void)
{
if(circuit_flag2 == 0)
{
	// if(light_flag){set_color(RED);}
	//看4号门
//	while(scan_cross(line,8000,200)){track_dynamic_Speed(8000);}//循迹到路口
	turn_right(90);
	delay_ms(100);
	vision_start();
	delay_ms(100);
	//确定红绿灯(1红2绿)
	if(vision_choose_cross()){
		circuit_flag2 = 4;
		vision_stop();
		delay_ms(100);
		// if(light_flag){set_color(GREEN);}
		S_regulate_track(0,8000,500);//起步
	    while(scan_stair()){track_dynamic_Speed(8000);};//循迹到门
		door();
        turn_right(90);
	    S_regulate_track(0,10000,300);//起步
		// S_regulate_track_left(0,12000,400);
		return;//退出
	}
//	vision_stop();
//	delay_ms(100);
	//换2号门
	turn_left(50);
	S_regulate_track(0,8000,300);//起步
	while(scan_cross(line,8000,300)){track_dynamic_Speed(8000);}//循迹到路口
	motor_stop();
	delay_ms(100);
	vision_start();
	delay_ms(100);
	//确定红绿灯(1红2绿)
	if(vision_choose_cross()){
	circuit_flag2 = 2;
	vision_stop();
	delay_ms(100);
//		if(light_flag){set_color(GREEN);}
	S_regulate_track(0,6000,150);//起步
	while(scan_stair()){track_dynamic_Speed(6000);};//循迹到门
	door();
    turn_right(135);
	S_regulate_track_left(0,10000,300);//起步
	S_regulate_track(10000,13000,300);//起步
	track_highSpeed_count(100);//高速循迹一段
	S_regulate_track(13000,10000,300);//起步
	while(scan_cross_nostop(line)){track_dynamic_Speed(10000);}//循迹到路口
		return;//退出
	}
	circuit_flag2 = 3;
	vision_stop();
	delay_ms(100);
//	if(light_flag){set_color(GREEN);}
	
	//换3号门
	turn_right(120);
	delay_ms(100);
	
	S_regulate_track(0,6000,150);//起步
	while(scan_stair()){track_dynamic_Speed(6000);};//循迹到门
	door();
	
	turn_right(40);//右转40
	S_regulate_track_left(0,10000,300);//起步
	
}



else if(circuit_flag2 == 3){
//	S_regulate_track(0,10000,500);
//	S_regulate_track(0,8000,300);//起步
	turn_right(45);
	S_regulate_track(0,8000,300);//起步
	while(scan_cross(line,8000,150)){track_dynamic_Speed(8000);}//循迹到路口
	turn_right(120);
	delay_ms(100);
	
	S_regulate_track(0,6000,150);//起步
	while(scan_stair()){track_dynamic_Speed(6000);};//循迹到门
	door();
	
	turn_right(40);//右转40
	S_regulate_track_left(0,10000,300);//起步
	return;//退出
}
else if(circuit_flag2 == 2){
	turn_right(45);//右转90
	delay_ms(100);
	S_regulate_track(0,8000,400);
	S_regulate_track(8000,13000,500);
	S_regulate_track(13000,8000,300);
	while(scan_stair()){track_dynamic_Speed(8000);}//循迹到门

	door();
		
	turn_right(135);
	S_regulate_track_left(0,10000,300);//起步
	S_regulate_track(10000,13000,300);//起步
	track_highSpeed_count(100);//高速循迹一段
	S_regulate_track(13000,10000,300);//起步
	while(scan_cross_nostop(line)){track_dynamic_Speed(10000);}//循迹到路口
	return;//退出
}
else if(circuit_flag2 == 4){
	
    turn_right(90);
	
	while(scan_stair()){track_dynamic_Speed(8000);};//循迹到门

	door();

	turn_right(90);
	float record_angle = Yaw;
	S_regulate_walkspin(0,10000,record_angle,400);//起步
//	S_regulate_track(0,10000,300);//起步
	
	return;//退出
}
}
// 去往6号平台函数
// 开始：通过门
// 结束：走出双线圆环，左转面对双线部分
// 退出速度为（0）
void goto_6_part(void){
	float record_angle = Yaw;
	Arm_down();
    S_regulate_track(0,6000,300);
	
	S_regulate_track(6000,6000,1000);
	while (scan_2Line())
	{
		track_dynamic_Speed(5000);
	}
S_regulate_track2Line(6000, 6000, 1000);
PID_sensor2.Kp = 70.0f;
	while(scan_right_cross_2L()){track_dynamic_Speed_2Line(6000);}
	S_regulate_track2Line(5000, 5000, 50);
	walk_spin_right(100);
	S_regulate_track2Line(0,4000,200);
    S_regulate_track2Line(4000,6000,100);
    while(scan_stair()){track_dynamic_Speed_2Line(6000);}
	updown_low_stair6();
	while(scan_right_cross_2L()){track_dynamic_Speed_2Line(6000);}
	S_regulate_track2Line(6000,0,600);
	turn_right(100);
	track_2L_count(1000);
	while(!(scan_line())){track_dynamic_Speed_2Line(6000);}
	
	S_regulate_track(6000,6000,200);
//	Arm_up();
	S_regulate_track(6000,8000,200);
	while(scan_left_cross(line,8000,50)){track_dynamic_Speed(8000);}
	walk_spin_left(90);	
		

}
// 去往6号平台函数
// 开始：通过门
// 结束：走出双线圆环，转身
// 退出速度为（0）
void goto_6_part2(void){
	float record_angle = Yaw;
    S_regulate_track(0,6000,400);
	while(scan_cross_nostop(line)){
		track_dynamic_Speed(6000);
	}
	Arm_down();
	while (scan_2Line())
	{
		track_dynamic_Speed(5000);
	}
//	motor_stop();
	while(scan_right_cross_2L()){track_dynamic_Speed_2Line(6000);}
	record_angle = Yaw;
	S_regulate_track2Line(6000, 0,700);
	turn_right(90);
	S_regulate_track2Line(0,4000,200);
    S_regulate_track2Line(4000,6000,100);
    while(scan_stair()){track_dynamic_Speed_2Line(6000);}
	updown_low_stair6();
	while(scan_right_cross_2L()){track_dynamic_Speed_2Line(6000);}
	S_regulate_track2Line(6000,0,600);
	motor_stop();
	delay_ms(500);
  turn_right(100);
	track_2L_count(1000);
	while(!(scan_line())){track_dynamic_Speed_2Line(6000);}
	Arm_up();
	S_regulate_track(6000,0,500);
	spin180();	
	S_regulate_track(0,8000,1000);
	circuit_flag = 1;
	while(scan_cross(line,8000,200)){track_dynamic_Speed(8000);}//循迹到路口
	turn_left(90);
	// while(scan_cross(line,8000,200)){track_dynamic_Speed(8000);}
		

}
void goto_5_part(void){ 
	S_regulate_track(0,8000,200);
    S_regulate_track(8000,10000,200);
	// track_highSpeed_count(1000);
//	S_regulate_track(13000,10000,300);
	while(scan_stair()){track_dynamic_Speed(10000);}
	updown_low_stair();
	circuit_flag = 4;
	while(scan_right_cross(line,8000,200)){track_dynamic_Speed(8000);
	}
}
void goto_4_part_return(void){ 
	
//	S_regulate_track(0,8000,300);
	while(scan_stair()){track_dynamic_Speed(10000);}
	updown_low_stair();	
	S_regulate_track(8000,10000,300);
	while(scan_cross_nostop(line)){track_dynamic_Speed(10000);}
	S_regulate_track(10000,10000,200);
	S_regulate_track(10000,8000,200);
	while(scan_cross(line,8000,300)){track_dynamic_Speed(10000);}
	turn_right(135);
	S_regulate_track(0,8000,200);
	while(scan_stair()){track_dynamic_Speed(8000);}
	teeterboard_270();
	while(scan_cross(line,8000,200)){track_dynamic_Speed(8000);}
	turn_right(40);
	S_regulate_track(0,8000,300);
	while(scan_stair()){track_dynamic_Speed(8000);}
	up_low_stair();
	motor_stop();
}
void test_2L(void){
//	Arm_down();
//	S_regulate_track2Line(0,8000,500);
	while(scan_stair()){track_dynamic_Speed_2Line(8000);}
	float record_angle = Yaw;
	obstacle_goto();
	while(scan_left_cross_2L()){track_dynamic_Speed_2Line(7000);}
	record_angle = Yaw;
//	motor_stop();
//	delay_ms(500);
	S_regulate_track2Line(5000, 5000, 50);
	walk_spin_left(90);
	S_regulate_track2Line(5000, 8000, 200);
	while(scan_stair()){track_dynamic_Speed_2Line(8000);}
	obstacle_goto();
	S_regulate_track2Line(4000, 8000, 200);
	S_regulate_track2Line(8000, 13000, 200);
	track_highSpeed_count(500);
	S_regulate_track2Line(13000, 8000, 200);
	while(scan_stair()){track_dynamic_Speed_2Line(8000);}
	obstacle_three_goto();
	S_regulate_track2Line(4000, 8000, 200);
	while(scan_stair()){track_dynamic_Speed_2Line(8000);}
	updown_middle_stair();
	while(scan_right_cross_2L()){track_dynamic_Speed_2Line(8000);}
	S_regulate_track2Line(5000, 5000, 50);
	walk_spin_right(85);
	while(scan_stair()){track_dynamic_Speed_2Line(6000);}
	Arm_up();
	teeterboard_270();
	while(scan_cross(line,6000,150)){track_dynamic_Speed_2Line(6000);}
	turn_right(90);
}