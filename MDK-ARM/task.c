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
#include "usart.h"
// 上低台通用函数
// 功能：
//    实现上低台的操作，上台速度为10000
void up_low_stair(void){
	uint16_t i;
	float angle;
	float record_angle;
	
	i=8000;
	
	while(Pitch > 5 && Pitch < 355){
		i-=500;
		if(i<=4000){i=4000;}
		
//		walkspin_dynamic(record_angle,i);
		track_dynamic_Speed(i);
	}
	record_angle = Yaw;
	S_regulate_walkspin(4000,0,record_angle,350);
	angle = ((record_angle + 180) > 360)? (record_angle - 180) : (record_angle + 180);
	
	motor_stop();
	motor_angle_control(record_angle);
	delay_ms(500);
	spin180();
}
// 下低台通用函数
// 功能：
//    实现下低台的操作，落地速度为10000
void down_low_stair(void){
	float angle;
	angle = Yaw;
	uint16_t i=0;
	while(!(Pitch > 5 && Pitch < 355)){
		i+=200;
		if(i>4000){i=4000;}
		walkspin_dynamic(angle,i);
	}
//	OLED_ShowNum(0,0,RollX,3);
	while(!(scan_line())){walkspin_dynamic(angle,i);}
	S_regulate_track(4000,8000,300);
	Arm_down();
	S_regulate_track(8000,10000,200);
	
}
// 上下低台通用函数
// 功能：
//    实现上下低台的操作，上台、落地速度为10000
void updown_low_stair(void){
	uint16_t i;
	float angle;
	float record_angle;
	
	i=8000;
	
//	motor_stop();
//	delay_ms(500);
	
	while(Pitch > 5 && Pitch < 355){
		i-=500;
		if(i<=4000){i=4000;}
//		walkspin_track_dynamic(record_angle, i);
//		walkspin_dynamic(record_angle,i);
		track_dynamic_Speed(i);
	}
	record_angle = Yaw;
	S_regulate_walkspin(4000,2500,record_angle,150);
	
	S_regulate_walkspin(2500,0,record_angle,100);
	motor_stop();
	delay_ms(500);
	record_angle = Yaw;
//	motor_angle_control(record_angle);
	angle = ((record_angle + 180) > 360)? (record_angle - 180) : (record_angle + 180);
	
//	motor_stop();
	
	spin180();
//	for(i=0;i<3;i++){
//		motor_angle_control(angle);
//	}
	delay_ms(100);
	angle=Yaw;
	i=0;
	while(!(Pitch > 5 && Pitch < 355)){
		i+=200;
		if(i>3000){i=3000;}
		walkspin_dynamic(angle,i);
	}
	while(!(scan_line())){walkspin_dynamic(angle,i);}
	S_regulate_track(3000,8000,300);
//	S_regulate_track(7000,8000,200);
}
// 上下2号低台特殊函数
// 功能：
//    实现上下低台的操作，上台、落地速度为10000，会闪两下
void updown_low_stair2(void){
	uint16_t i;
	float angle;
	float record_angle;
	record_angle = Yaw;
	i=10000;
	while(Pitch > 10 && Pitch < 350){
		i-=100;
		if(i<=7000){i=7000;}
		
		walkspin_dynamic(record_angle,i);
	}
	
	S_regulate_walkspin(7000,6000,record_angle,450);
	S_regulate_walkspin(6000,0,record_angle,300);
	
	CAN_cmd_chassis(0,0,0,0);
	
	// twinkle();
	
	angle = ((record_angle + 180) > 360)? (record_angle - 180) : (record_angle + 180);
    
	motor_stop();
	delay_ms(100);
	spin180();
	
	delay_ms(100);
	i=0;
	while(!(Pitch > 10 && Pitch < 350)){
		i+=200;
		if(i>5000){i=5000;}
		walkspin_dynamic(angle,i);
	}
	//OLED_ShowNum(0,0,RollX,3);
	while(!(scan_line())){walkspin_dynamic(angle,i);}
	S_regulate_track(5000,8000,300);
}
// 上下低台通用函数
// 功能：
//    实现上下低台的操作，上台、落地速度为10000
void updown_low_stair6(void){
	uint16_t i;
	float angle;
	float record_angle;
	
	i=6000;
	
	while(Pitch > 5 && Pitch < 355){
		i-=500;
		if(i<=4000){i=4000;}
		
		track_dynamic_Speed_2Line(i);
	}
	record_angle = Yaw;
	S_regulate_walkspin(4000,2500,record_angle,350);
	S_regulate_walkspin(2500,0,record_angle,100);
	angle = ((record_angle + 180) > 360)? (record_angle - 180) : (record_angle + 180);
	
	motor_stop();
	delay_ms(100);
	spin180();
	motor_angle_control(angle);
	delay_ms(100);
	i=0;
	while(!(Pitch > 5 && Pitch < 355)){
		i+=200;
		if(i>4000){i=4000;}
		walkspin_dynamic(angle,i);
	}
//	while(Pitch > 5 && Pitch < 355){
//		track_dynamic_Speed_2Line(6000);
//	}
	S_regulate_track2Line(4000,7000,300);
//	S_regulate_track2Line(7000,8000,200);
}
// 上下中台
// 落地速度为6000
void updown_middle_stair(void){
	
   	uint16_t i;
	float angle;
	float record_angle;
	
	i=6000;
	
	while(Pitch > 5 && Pitch < 355){
		i-=500;
		if(i<=4000){i=4000;}
		
		track_dynamic_Speed_2Line(i);
	}
	record_angle = Yaw;
	S_regulate_walkspin(4000,0,record_angle,350);
	angle = ((record_angle + 180) > 360)? (record_angle - 180) : (record_angle + 180);
	
	motor_stop();
	delay_ms(100);
	spin180();
	motor_angle_control(angle);
	delay_ms(100);
	i=0;
	while(!(Pitch > 5 && Pitch < 355)){
		i+=200;
		if(i>3500){i=3500;}
		walkspin_dynamic(angle,i);
	}
	PID_sensor2.Kp = 30.0f;
	while(Pitch > 5 && Pitch < 355){
		track_dynamic_Speed_2Line(4000);
	}
	PID_sensor2.Kp = 70.0f;
	S_regulate_track2Line(4000,6000,200);
}
// 上下高台
// 落地速度为10000
void updown_high_stair(void) {
    uint16_t i;
	float angle;
	float record_angle;
	record_angle = Yaw;
	//减速
	
	S_regulate_track(10000,7000,300);
	
	while(Pitch > 10 && Pitch < 350){
		track_dynamic_Speed(7000);
	}
	S_regulate_track(7000,3000,500);
	S_regulate_track(3000,6000,500);
	
	i=6000;
	uint16_t record_time = read_time();
	while(Pitch > 10 && Pitch < 350){
		i-=10;
		if(i<=4000){i=4000;}

		track_dynamic_Speed(i);
	}
	
	record_angle = Yaw;
	S_regulate_walkspin(4000,0,record_angle,200);
	angle = ((record_angle + 180) > 360)? (record_angle - 180) : (record_angle + 180);
	
	motor_stop();
	delay_ms(100);
	spin180();

	angle-=3;
	if(angle < 0){angle +=359;}
	delay_ms(100);
	i=0;
	while(!(Pitch > 20 && Pitch < 340)){
		i+=100;
		if(i>3000){i=3000;}
		walkspin_dynamic(angle,i);
	}
	while(!(scan_line())){
		i+=100;
		if(i>3000){i=3000;}
		walkspin_dynamic(angle,i);
	}
	i=3000;
	//倾斜时
	while(Pitch > 15 && Pitch < 345){
		i+=50;
		if(i<=7000){i=7000;}
		track_dynamic_Speed(i);
	}
	S_regulate_track(7000,2000,500);
	//S_regulate_track(0,4000,00);
	while(!(Pitch > 20 && Pitch < 340)){
		track_dynamic_Speed(2000);
	}

	i=2000;
	while(Pitch > 8 && Pitch < 352){
		i+=30;
		if(i>=8000){i=8000;}
		track_dynamic_Speed(i);
	}
	S_regulate_track(8000,10000,300);
}
// 爬长桥
// 落地速度为10000
void climb_moutain(void){
	static float moutain_angle;
	uint16_t angle_limit_flag = 0;
	uint16_t angle_limit = 20;
	uint8_t remove_limit = 0;
	moutain_angle = Yaw;
	int i;
	//float record_angle = moutain_angle;
	S_regulate_track2Line(8000,4000,200);
//	S_regulate_track2Line(4000,6000,300);
//	S_regulate_moutain(8000,4000,moutain_angle,500);
//	S_regulate_moutain(4000,8000,moutain_angle,300);
	while(Pitch > 5 && Pitch < 355){
//		moutain_angle-=updown_stair_error_2L();
//		if(moutain_angle > 360){moutain_angle -= 360;}	
//		else if(moutain_angle < 0){moutain_angle += 360;}
		track_dynamic_Speed_2Line(4000);
//		walkspin_dynamic(moutain_angle, 8000);
	}
	S_regulate_track2Line(4000,8000,300);
		while(!(Pitch > 5 && Pitch < 355)){
		track_dynamic_Speed_2Line(8000);
	}
		moutain_angle = Yaw;
	S_regulate_moutain(8000,3000,moutain_angle,200);
     // 通过陀螺仪判断下台时间
//     while(!(Pitch > 200 && Pitch < 350)){
//			 track_dynamic_Speed_2Line(8000);
////         walkspin_dynamic(moutain_angle,8000);
////         // 色标判断校正
////         if (RF == 0 && angle_limit_flag < angle_limit){
////             moutain_angle += 0.2f;
////	 		angle_limit_flag++;
////	 		remove_limit = 0;
////             if(moutain_angle > 360){moutain_angle -= 360;}
////         }
////         else if(LF == 0 && angle_limit_flag < angle_limit){
////             moutain_angle -= 0.2f;
////	 		angle_limit_flag++;
////			remove_limit = 0;
////             if(moutain_angle < 0){moutain_angle += 360;}
////         }
////	 	if(LF == 0 || RF == 0){
////	 		remove_limit++;
////	 		if(remove_limit >= 20){
////	 			remove_limit =0;
////	 			angle_limit_flag = 0;
////	 		}
////		}
//     }
		 
		 while(!(scan_line())){track_dynamic_Speed_2Line(3000);}
		 Arm_up();
	   S_regulate_track(3000,8000,500);
	
}
float z0 = 4;
float z1 = 3.0;
float z2 = 2.0;
float z3 = 1.0;
float z4 = 0.5;
float z5 = 0.1;
float a0w = 2.0;
float a1w = 1.8;
float a2w = 1.0;
float a3w = 0.3;
float a4w = 0.2;
float a5w = 0.0;

// 角度环灰度误差计算函数（新的）
float updown_stair_error_new(void){
	float error_angle;
	
	if (line[5] == scan_flag) { error_angle = -a5w; }
    else if (line[6] == scan_flag) { error_angle = a5w; }
    else if (line[4] == scan_flag) { error_angle = -a4w; }
    else if (line[7] == scan_flag) { error_angle = a4w; }
    else if (line[3] == scan_flag) { error_angle = -a3w; }
    else if (line[8] == scan_flag) { error_angle = a3w; }
    else if (line[2] == scan_flag) { error_angle = -a2w; }
    else if (line[9] == scan_flag) { error_angle = a2w; }
    else if (line[1] == scan_flag) { error_angle = -a1w; }
    else if (line[10] == scan_flag) { error_angle = a1w; }
    else if (line[0] == scan_flag) { error_angle = -a0w; }
    else if (line[11] == scan_flag) { error_angle = a0w; }
    else {  
		error_angle = 0;
    }
	return error_angle;
}
// 角度环灰度误差计算函数（双线）
float updown_stair_error_2L(void){
	float error_angle;
	float k=0.03f;
	bais=bais_judgment_2Line();
	error_angle = bais * k;
	return error_angle;
}
// 过三连障碍函数
void obstacle_three_goto(void){
	float record_angle = Yaw;
	uint32_t time_on;
    uint16_t i =3500;
    time_on = read_time();
    while (read_time() <= time_on + 2800) {
        i+=50;
		if(i > 3500){i=3500;}
		walkspin_track_dynamic_2L(record_angle, i);
    }
	// while(1){
	// 	if(abs((int)spin_error(record_angle)) > 20){
	// 		break;
	// 	}
	// 	track_dynamic_newT(3500);
	// }
}
// 过三连障碍函数
void obstacle_goto(void){
	float record_angle = Yaw;
	uint32_t time_on;
    uint16_t i =3500;
    time_on = read_time();
    while (read_time() <= time_on + 800) {
        i+=50;
		if(i > 4000){i=4000;}
		walkspin_track_dynamic_2L(record_angle, i);
//		walkspin_dynamic(record_angle,i);
//		track_dynamic_Speed_2Line(i);
    }
	// while(1){
	// 	if(abs((int)spin_error(record_angle)) > 20){
	// 		break;
	// 	}
	// 	track_dynamic_newT(3500);
	// }
}
// 过梯形山
void obstacle_stair(void){
	float record_angle;
	record_angle = Yaw;
	while(Pitch > 5 && Pitch < 355){
//		record_angle -= updown_stair_error_new();
//		if(record_angle > 360){record_angle -= 360;}	
//		else if(record_angle < 0){record_angle += 360;}
//		
//		walkspin_dynamic(record_angle,4000);
		walkspin_track_dynamic(record_angle, 4000);
	}
	S_regulate_track(4000,6000,200);
	S_regulate_track(6000,8000,200);

}
// 过跷跷板
// 针对零初始速度下的过跷跷板
void teeterboard(float angle,float correct_angle){
	
	S_regulate_teeterboard_nocorrect(0,7500,angle,600);
	
	S_regulate_teeterboard(7500,0,angle,600);
	
	S_regulate_teeterboard(0,3000,angle,300);
	
	walk_spin_left(correct_angle);
	
}
// 过跷跷板
// 针对第一个跷跷板
void teeterboard_270(void){
	float record_angle;
	uint16_t angle_limit_flag = 0;
	float stride = 1.2;
	uint16_t angle_limit = 12;
	record_angle = Yaw;
	if(record_angle > 359){record_angle -= 359;}
	//前进到倾斜
	while(!(Pitch > 20 && Pitch < 200)){
		walkspin_dynamic(record_angle,3000);
        // 色标判断校正
        // if (RF == 0 && angle_limit_flag < angle_limit){
        //     record_angle += 0.3f*stride;
		// 	if(stride > 0.1f){stride-=0.05f;}
		// 	angle_limit_flag++;
        //     if(record_angle > 360){record_angle -= 360;}
        // }
//        else if(LF == 0 && angle_limit_flag < angle_limit){
//            record_angle -= 0.3f*stride;
//			angle_limit_flag++;
//			if(stride > 0.1f){stride-=0.05f;}
//            if(record_angle < 0){record_angle += 360;}
//        }
		angle_limit_flag = 0;
	}
	angle_limit_flag = 0;
	stride = 1.5;
	while(Pitch > 2 && Pitch < 352){
		walkspin_dynamic(record_angle,3000);
        // 色标判断校正
        // if (RF == 0 && angle_limit_flag < angle_limit){
        //     record_angle += 0.3f*stride;
		// 	if(stride > 0.1f){stride-=0.05f;}
		// 	angle_limit_flag++;
        //     if(record_angle > 360){record_angle -= 360;}
        // }
        // else if(LF == 0 && angle_limit_flag < angle_limit){
        //     record_angle -= 0.3f*stride;
		// 	if(stride > 0.1f){stride-=0.05f;}
		// 	angle_limit_flag++;
        //     if(record_angle < 0){record_angle += 360;}
        // }
		angle_limit_flag = 0;
	}
	S_regulate_track2Line(3000,6000,300);	
	
}// 过跷跷板
// 针对第二个过跷跷板
void teeterboard_90(float correct_angle){
	float record_angle;
	uint16_t angle_limit_flag = 0;
	float stride = 1.5;
	uint16_t angle_limit = 12;
	record_angle = Yaw;
	if(record_angle > 359){record_angle -= 359;}
	//前进到倾斜
	while(!(Pitch > 20 && Pitch < 200)){
		walkspin_dynamic(record_angle,3000);
        // 色标判断校正
        // if (RF == 0 && angle_limit_flag < angle_limit){
        //     record_angle += 0.3f*stride;
		// 	if(stride > 0.1f){stride-=0.05f;}
		// 	angle_limit_flag++;
        //     if(record_angle > 360){record_angle -= 360;}
        // }
        // else if(LF == 0 && angle_limit_flag < angle_limit){
        //     record_angle -= 0.3f*stride;
		// 	angle_limit_flag++;
		// 	if(stride > 0.1f){stride-=0.05f;}
        //     if(record_angle < 0){record_angle += 360;}
        // }
		angle_limit_flag = 0;
	}
	angle_limit_flag = 0;
	stride = 1.5;
	while(Pitch > 7 && Pitch < 352){
		walkspin_dynamic(record_angle,3000);
        // 色标判断校正
        // if (RF == 0 && angle_limit_flag < angle_limit){
        //     record_angle += 0.3f*stride;
		// 	if(stride > 0.1f){stride-=0.05f;}
		// 	angle_limit_flag++;
        //     if(record_angle > 360){record_angle -= 360;}
        // }
        // else if(LF == 0 && angle_limit_flag < angle_limit){
        //     record_angle -= 0.3f*stride;
		// 	if(stride > 0.1f){stride-=0.05f;}
		// 	angle_limit_flag++;
        //     if(record_angle < 0){record_angle += 360;}
        // }
		angle_limit_flag = 0;
	}
	turn_left(correct_angle);
	S_regulate_track(0,8000,500);
	
}

// 8000速度下的过跷跷板
void teeterboard_move(void){
	float teeter_angle;
	teeter_angle = Yaw;
	
	S_regulate_teeterboard(8000,4000,teeter_angle,400);
	S_regulate_teeterboard(4000,0,teeter_angle,1200);
	S_regulate_teeterboard(0,2000,teeter_angle,300);
	
}
uint16_t door_exit_speed = 0;
// 过门函数
// 给后半段加速
void door(void){
	uint16_t i;
	uint16_t record_time = read_time();
	for(i=8000;i>=3000;i-=50){
		track_dynamic_Speed(i);
	}
	door_exit_speed=3000;
	while(scan_cross(line,door_exit_speed,200) || read_time() - record_time < 1000 ){
		door_exit_speed+=50;
		if(door_exit_speed>=8000){door_exit_speed=8000;}
		track_dynamic_Speed(door_exit_speed);
	}
}
// 过门函数
// 给后半段加速
uint16_t door_nostop_exit_speed = 0;
void door_nostop(void){
	uint16_t i;
	uint16_t record_time = read_time();
	for(i=8000;i>=3000;i-=50){
		track_dynamic_Speed(i);
	}
	door_nostop_exit_speed=3000;
	while(scan_cross_nostop(line) || read_time() - record_time < 1000 ){
		door_nostop_exit_speed+=50;
		if(door_nostop_exit_speed>=10000){door_nostop_exit_speed=10000;}
		track_dynamic_Speed(door_nostop_exit_speed);
	}
}
//过悬崖函数
void cliff(float left_correct_angle){
	uint16_t re_time = read_time();
	S_regulate_track(6000,2500,200);
	while(!((Pitch > 200 && Pitch <= 359) || Pitch == 0 || Pitch == 1)){		//低头退出
		if((read_time()-re_time) > 1800){break;}//超时退出
		walkspin_dynamic(left_correct_angle,2500);
	}
	
	uint16_t max_correct_flag=0;
	left_correct_angle += 10;
	if(left_correct_angle > 359){left_correct_angle -= 359;}
	uint16_t i=2500;
	while(!(scan_line()) || max_correct_flag < 40){
		if(max_correct_flag <= 125){
			left_correct_angle += 0.4f;
			max_correct_flag++;
		}
		
		if(left_correct_angle > 359){left_correct_angle -= 359;}
		i+=100;
		if(i>8000){i=8000;}
		walkspin_dynamic(left_correct_angle,i);
	}
}
// void slope(void){
// 	static float slope_angle;
// 	slope_angle = YawZ;
// 	uint16_t angle_limit_flag = 0;
// 	uint16_t angle_limit = 20;
// 	uint8_t remove_limit = 0;
	
// 	float r1=slope_angle - 15;
// 	if(r1 < 0){r1-=359;}
	
// 	S_regulate_walkspin(8000,6000,r1,300);
// 	S_regulate_moutain(6000,8000,slope_angle,300);
	
// 	while(!(scan_line())){
//         walkspin_dynamic(slope_angle,8000);
//         // 色标判断校正
//         if (RF == 0 && angle_limit_flag < angle_limit){
//             slope_angle += 0.2f;
// 			angle_limit_flag++;
// 			remove_limit = 0;
//             if(slope_angle > 360){slope_angle -= 360;}
//         }
//         else if(LF == 0 && angle_limit_flag < angle_limit){
//             slope_angle -= 0.2f;
// 			angle_limit_flag++;
// 			remove_limit = 0;
//             if(slope_angle < 0){slope_angle += 360;}
//         }
// 		if(LF == 0 || RF == 0){
// 			remove_limit++;
// 			if(remove_limit >= 20){
// 				remove_limit =0;
// 				angle_limit_flag = 0;
// 			}
// 		}
//     }
// 	S_regulate_track(8000,5000,300);
// 	while(1){
// 		if(abs((int)spin_error(slope_angle)) > 20){
// 			break;
// 		}
// 		track_dynamic_Speed(5000);
// 	}
// }
// void cliff_new(float left_correct_angle){
// 	uint16_t re_time = read_time();
// 	S_regulate_track(6000,2500,200);
// 	while(!((Pitch > 200 && Pitch <= 359) || Pitch == 0)){		//低头退出
// 		if((read_time()-re_time) > 1800){break;}//超时退出
// 		walkspin_dynamic(left_correct_angle,2500);
// 	}
	
// 	while(1){
// 		if(abs((int)spin_error(left_correct_angle)) > 20){
// 			break;
// 		}
// 		track_dynamic_Speed(4000);
// 	}
// }
void vision_init(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
    GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	delay_ms(1000);
	
}
uint8_t vision_start_command[]="\xAA\xBB\x01\x00\xFF";
uint8_t vision_stop_command[]="\xAA\xBB\x00\x00\xFF";

void vision_start(void){
	// RGB_ON();
	
	
	delay_ms(1000);
	uint8_t start[]="\xFF\xBB\x01\x01\xAA";
	usart2_send(start);
	
}
void vision_stop(void){
	// RGB_OFF();
	
	HAL_UART_AbortReceive_IT(&huart4);
	
	uint8_t stop[]="\xFF\xBB\x02\x02\xAA";
	usart2_send(stop);
}

// 路口红绿灯识别函数，返回1绿灯，返回0红灯
// 对准后开启
int vision_choose_cross(void){
	//当没读到值时（choose_cross_flag=0）
	delay_ms(100);
	choose_cross_flag = 2;
	uint16_t i=0;
	while(choose_cross_flag == 0){
		i++;
		if(i > 100){tumble();}
		if(i > 135){choose_cross_flag = 2;break;}
		delay_ms(10);
	}
	//红灯返回0（禁止通行）
	if(choose_cross_flag == 1){
		choose_cross_flag = 0;
		return 0;
	}
	//绿灯返回1（允许通行）
	else if(choose_cross_flag == 2){
		choose_cross_flag = 0;
		return 1;
	}
	else {
		// OLED_ShowNum(6,0,choose_cross_flag,3);
	}
	return 0;
}
