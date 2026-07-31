#include "chassis.h"
#include "sensor.h"
#include "DM_Motor.h"
#include "DJI_Motor.h"
#include "pid.h"
#include "jy61p.h"
#include "bsp_can.h"
#include "task.h"
#include "tim.h" // 包含定时器相关头文件
#include "bsp_dwt.h"
#include "motion_control.h"
#include "math.h"
uint32_t time;
uint16_t out_flag_new=0;
float bais=0;
float W_out=0;
float target_bais=0;
//=====================================================================================
//============================角度部分=========================================
//=====================================================================================
void walkspin_dynamic(float target_angle, float speed){
    float wz1=PID_Angle_Position(&PID_YAW, Yaw, target_angle, 3000);
 
        calculate_motor_speeds(speed,-wz1);

    
}
void walkspin_dynamic_behind(float target_angle, float speed){
    float wz1=PID_Angle_Position(&PID_YAW, Yaw, target_angle, 3000);
        calculate_motor_speeds_behind(speed,-wz1);

    
}
void walkspin_track_dynamic_2L(float target_angle, float speed){
    float wz1=PID_Angle_Position(&PID_YAW, Yaw, target_angle, 3000);
	      bais = bais_judgment_2Line();
	      W_out = PID_Position(&PID_sensor2,bais,0,2000);
        calculate_motor_speeds(speed,-wz1+W_out);
}
void walkspin_track_dynamic(float target_angle, float speed){
    float wz1=PID_Angle_Position(&PID_YAW, Yaw, target_angle, 3000);
	      bais = bais_judgment();
	      W_out = PID_Position(&PID_sensor3,bais,0,1000);
        calculate_motor_speeds(speed,-wz1+W_out-200);
}
// 原地自转函数
// 输入：
//    angle：目标角度
// 功能：
//    转向目标角度
//    注意：此函数只需要调用一次，并且输入量是绝对角度
void spin(float angle){
    motor_angle_control(angle);
}

/**
 * @brief 向左转指定角度（逆时针，Yaw 增大）
 * @param deg 需左转的角度（度）
 */
void turn_left(float deg)
{
uint16_t i;
    float result = Yaw + deg;
	PID_Init();
    // if (result > 360) {
    //     result -= 360;
    // }
		motor_angle_control(result);
//		while(abs(calculate_angle_error(Yaw,result))>=2.0f){
//			    float wz1=PID_Angle_Position(&PID_straight, Yaw, result, 3000);
//        if(wz1>0){
//           calculate_motor_speeds(0,-(wz1+front_wz));
//        }
//        else if(wz1<0){
//            calculate_motor_speeds(0,front_wz-wz1);
//        }
//		
////        calculate_motor_speeds(0,-(wz1));
////			walkspin_dynamic(result,3500);
//		}
//		while(abs(calculate_angle_error(Yaw,result))>=2.0f){
//			    float wz1=PID_Angle_Position(&PID_straight, Yaw, result, 3000);
//        if(wz1>0){
//           calculate_motor_speeds(0,-(wz1+front_wz));
//        }
//        else if(wz1<0){
//            calculate_motor_speeds(0,front_wz-wz1);
//        }
//		
//        calculate_motor_speeds(0,-(wz1));
//			walkspin_dynamic(result,3500);
//		}
}

/**
 * @brief 向右转指定角度（顺时针，Yaw 减小）
 * @param deg 需右转的角度（度）
 */
void turn_right(float deg)
{
    // if (deg < 0) deg = -deg;
    // while (deg >= 360.0f) deg -= 360.0f;
    PID_Init();
    uint16_t i;
    float result = Yaw - deg;
    // if (result < 0) {
    //     result += 360;
    // }
		motor_angle_control(result);
//		while(abs(calculate_angle_error(Yaw,result))>=3.0f){
//			    float wz1=PID_Angle_Position(&PID_straight, Yaw, result, 3000);
//        if(wz1>0){
//           calculate_motor_speeds(0,-(wz1+front_wz));
//        }
//        else if(wz1<0){
//            calculate_motor_speeds(0,front_wz-wz1);
//        }
//        calculate_motor_speeds(0,(-wz1));
//			walkspin_dynamic(result,3500);
//		}

}
// 左转函数
// 输入：
//    angle：从当前角度要左转的角度
// 功能：
//    以一定初速度实现左转
void walk_spin_left(float angle)
{
    uint16_t i;
    // 使用累加式角度处理，Yaw已经是连续角度，直接计算目标角度
    float result = Yaw + angle;
    
    // 重置PID控制器，清除积分项和微分项，避免历史状态影响
    PID_Init();
    
		while(fabs(calculate_angle_error_fixed(Yaw,result))>=3.0f){
			    float wz1=PID_Angle_Position(&PID_YAW, Yaw, result, 3000);
 
        calculate_motor_speeds(3500,-(wz1));
//			walkspin_dynamic(result,3500);
		}
    
//    // 添加短延时确保角度完全稳定，避免立即执行下一个动作
//    HAL_Delay(50);
//    for(i=0;i<angle*2.5f;i++){
//		walkspin_dynamic(result,3500);
//	}
}
// 右转函数
// 输入：
//    angle：从当前角度要右转的角度
// 功能：
//    以一定初速度实现右转
void walk_spin_right(float angle)
{
	uint16_t i;
    // 使用累加式角度处理，Yaw已经是连续角度，直接计算目标角度
    float result = Yaw - angle;
    
    // 重置PID控制器，清除积分项和微分项，避免历史状态影响
    PID_Init();
    
		while(fabs(calculate_angle_error_fixed(Yaw,result))>=3.0f){
			float wz1=PID_Angle_Position(&PID_YAW, Yaw, result, 3000);
        
        calculate_motor_speeds(3500,-(wz1));
//			walkspin_dynamic(result,3500);
		}
    
    // 添加短延时确保角度完全稳定，避免立即执行下一个动作
//    HAL_Delay(50);
//	for(i=0;i<angle*2.5f;i++){
//		walkspin_dynamic(result,3500);
//	}  
}
// 原地自转180
// 待测试稳定性
void spin180(void){
	turn_left(179);
	// motor_angle_control(180);
	// motor_angle_control(180);
}

// 正确计算角度误差的函数，处理0/360边界
float calculate_angle_error_fixed(float current, float target) {
    float error = target - current;
    
//    // 处理跨越0/360边界的情况，确保误差在-180到+180之间
//    if (error > 180.0f) {
//        error -= 360.0f;
//    } else if (error < -180.0f) {
//        error += 360.0f;
//    }
    
    return error;
}

float angle_restrict(float angle){
	if(angle<0){angle+=360;}
	if(angle>360){angle-=360;}
	return angle;
}
//=====================================================================================
//============================循迹部分=========================================
//=====================================================================================
void track_dynamic_Speed(float speed){
    bais = bais_judgment();
    W_out = PID_Position(&PID_sensor1,bais,0,5);
   DM_SpeedControl(DM_Chassis1_TX_ID,MOTOR_ENABLE,-(speed+W_out));
   DM_SpeedControl(DM_Chassis2_TX_ID,MOTOR_ENABLE,speed-W_out);

}
void track_dynamic_Speed_2Line(float speed){
    bais = bais_judgment_2Line();
    W_out = PID_Position(&PID_sensor2,bais,target_bais,2000);
//    //脱线次数过多停车
//	if(out_flag_new >= 100){
//		out_flag_new = 0;
//		motor_stop();
//		return;
//	}
//    if (scan_2Line()==1) {
//		out_flag_new++;
//        calculate_motor_speeds(speed,0);
//        return;
//    }
//	else{
//		out_flag_new = 0;
//	}
    calculate_motor_speeds(speed,W_out);
}
// 低速循迹进阶，会自动循迹指定的时间，单位 ms
// 输入：
//    time：循迹的时间（毫秒）
// 功能：
//    循环调用 track_lowSpeed 函数进行指定时间的低速循迹
void track_lowSpeed_count(int time)
{
    uint32_t time_on;

    time_on = read_time();
    while (read_time() <= time_on + time) {
        track_dynamic_Speed(8000);
    }
}
// 高速循迹进阶，会自动循迹指定的时间，单位 ms
// 输入：
//    time：循迹的时间（毫秒）
// 功能：
//    循环调用 track_highSpeed 函数进行指定时间的高速循迹
void track_highSpeed_count(int time)
{
    uint32_t time_on;

    time_on = read_time();
    while (read_time() <= time_on + time) {
        track_dynamic_Speed(13000);
    }
}
void track_2L_count(int time)
{
    uint32_t time_on;

    time_on = read_time();
    while (read_time() <= time_on + time) {
        track_dynamic_Speed_2Line(7000);
    }
}
void tumble(void){
	turn_left(3);
	turn_right(6);
	turn_left(6);
	turn_right(3);
	delay_ms(50);
}
uint8_t upstair_low_flag = 0; 
// 检测俯仰角判断车头是否上翘
// 功能：
//    作为 while 标志使用，检测车头是否上翘，连续 3 次检测到上翘返回 0，否则返回 1
uint8_t scan_stair(void)
{
    // 如果车头上翘
    if (Pitch > 10 && Pitch < 350) {
        upstair_low_flag++;
        // 大于 3 次检测到才会确认为 0
        if (upstair_low_flag >= 3) {
            return 0;
        }
    }
    // 否则清空计数
    else {
        upstair_low_flag = 0;
        return 1;
    }
    return 1;
}
//============================S形曲线变速部分=========================================
//=====================================================================================
// S形曲线差速控制电机函数
// 输入：
//    初始速度，目标速度，总需时间（ms）
// 功能：
//    用于平滑变速
void S_regulate_Ctl(float start_speed, float target_speed,uint32_t total_time){
	// 定义 S 型曲线的斜率参数
    float k = 0.02; 
	uint32_t start_time;
	uint32_t current_time;
    // 计算速度变化到一半时的时间
    float t_mid = total_time / 2;
	current_time = read_time();
	start_time = read_time();
	while(current_time < start_time + total_time){
		current_time = read_time();
		// 计算 S 型曲线的因子
		float s_factor = 1 / (1 + exp(-k * ((current_time - start_time) - t_mid)));

		// 计算当前速度
		float current_speed = start_speed + (target_speed - start_speed) * s_factor;
		DM_SpeedControl(DM_Chassis1_TX_ID,MOTOR_ENABLE,current_speed);
    DM_SpeedControl(DM_Chassis2_TX_ID,MOTOR_ENABLE,current_speed);
		ChassisMotionTime_Update();
		delay_ms(1);
	}
	
}
// S形曲线差速控制循迹函数
// 输入：
//    初始速度，目标速度，总需时间（ms）
// 功能：
//    用于平滑变速下的循迹
void S_regulate_track(float start_speed, float target_speed, uint32_t total_time){
	// 定义 S 型曲线的斜率参数
    float k = 0.02; 
	uint32_t start_time;
	uint32_t current_time;
    // 计算速度变化到一半时的时间
    float t_mid = total_time / 2;
	current_time = read_time();
	start_time = read_time();
	
	while(current_time < start_time + total_time){
		current_time = read_time();
		// 计算 S 型曲线的因子
		float s_factor = 1 / (1 + exp(-k * ((current_time - start_time) - t_mid)));

		// 计算当前速度
		float current_speed = start_speed + (target_speed - start_speed) * s_factor;

        /*
         * Logistic S曲线对时间求导得到目标加速度。
         * k的时间单位为1/ms，因此乘1000转换为rad/s^2；
         * 正值表示向前加速，负值表示向前减速。
         */
        float command_acceleration =
            (target_speed - start_speed) * k *
            s_factor * (1.0f - s_factor) * 1000.0f;
        Task3ChassisCommandAccelerationSet(command_acceleration);
		
		track_dynamic_Speed(current_speed);
		ChassisMotionTime_Update();
		// 以 5 ms 周期更新底盘，避免紧循环持续占满 CAN 发送邮箱。
		delay_ms(5);
		//OLED_ShowNum(0,0,current_speed,4);
	}

    // S曲线结束后进入匀速段，指令加速度应立即回到0。
    Task3ChassisCommandAccelerationSet(0.0f);
	
}
// S形曲线差速控制循迹函数
// 输入：
//    初始速度，目标速度，总需时间（ms）
// 功能：
//    用于平滑变速下的循迹
void S_regulate_track_left(float start_speed, float target_speed, uint32_t total_time){
	// 定义 S 型曲线的斜率参数
    float k = 0.02; 
	uint32_t start_time;
	uint32_t current_time;
    // 计算速度变化到一半时的时间
    float t_mid = total_time / 2;
	current_time = read_time();
	start_time = read_time();
	
	while(current_time < start_time + total_time){
		current_time = read_time();
		// 计算 S 型曲线的因子
		float s_factor = 1 / (1 + exp(-k * ((current_time - start_time) - t_mid)));

		// 计算当前速度
		float current_speed = start_speed + (target_speed - start_speed) * s_factor;
		
		track_dynamic_Speed(current_speed);
		//OLED_ShowNum(0,0,current_speed,4);
	}
	
}
// S形曲线差速控制循迹函数
// 输入：
//    初始速度，目标速度，总需时间（ms）
// 功能：
//    用于平滑变速下的循迹
void S_regulate_track_right(float start_speed, float target_speed, uint32_t total_time){
	// 定义 S 型曲线的斜率参数
    float k = 0.02; 
	uint32_t start_time;
	uint32_t current_time;
    // 计算速度变化到一半时的时间
    float t_mid = total_time / 2;
	current_time = read_time();
	start_time = read_time();
	
	while(current_time < start_time + total_time){
		current_time = read_time();
		// 计算 S 型曲线的因子
		float s_factor = 1 / (1 + exp(-k * ((current_time - start_time) - t_mid)));

		// 计算当前速度
		float current_speed = start_speed + (target_speed - start_speed) * s_factor;
		
		track_dynamic_Speed(current_speed);
		//OLED_ShowNum(0,0,current_speed,4);
	}
	
}
// S形曲线差速控制角度环函数
// 输入：
//    初始速度，目标速度，总需时间（ms）
// 功能：
//    用于平滑变速下的角度环
void S_regulate_walkspin(float start_speed, float target_speed,float angle,uint32_t total_time){
	// 定义 S 型曲线的斜率参数
    float k = 0.02; 
	uint32_t start_time;
	uint32_t current_time;
	
    // 计算速度变化到一半时的时间
    float t_mid = total_time / 2;
	current_time = read_time();
	start_time = read_time();
	while(current_time < start_time + total_time){
		current_time = read_time();
		// 计算 S 型曲线的因子
		float s_factor = 1 / (1 + exp(-k * ((current_time - start_time) - t_mid)));

		// 计算当前速度
		float current_speed = start_speed + (target_speed - start_speed) * s_factor;
		
		walkspin_dynamic(angle,current_speed);
		//OLED_ShowNum(0,0,current_speed,4);
	}
	
}
// S形曲线差速控制跷跷板函数
// 输入：
//    初始速度，目标速度，总需时间（ms）
// 功能：
//    用于平滑变速，带色标校正
void S_regulate_teeterboard(float start_speed, float target_speed,float record_angle,uint32_t total_time){
	// 定义 S 型曲线的斜率参数
    float k = 0.02; 
	uint32_t start_time;
	uint32_t current_time;
	uint16_t angle_limit_flag = 0;
	uint16_t angle_limit = 20;
    // 计算速度变化到一半时的时间
    float t_mid = total_time / 2;
	current_time = read_time();
	start_time = read_time();
	while(current_time < start_time + total_time){
		current_time = read_time();
		// 计算 S 型曲线的因子
		float s_factor = 1 / (1 + exp(-k * ((current_time - start_time) - t_mid)));

		// 计算当前速度
		float current_speed = start_speed + (target_speed - start_speed) * s_factor;
		
		if(current_time > t_mid)
		{
			// if (RF == 0 && angle_limit_flag < angle_limit){
			// 	record_angle += 0.5f;
			// 	angle_limit_flag++;
			// 	if(record_angle > 360){record_angle -= 360;}
			// }
			// else if(LF == 0 && angle_limit_flag < angle_limit){
			// 	record_angle -= 0.5f;
			// 	angle_limit_flag++;
			// 	if(record_angle < 0){record_angle += 360;}
			// }
		}
		
		walkspin_dynamic(record_angle,current_speed);
	}
	
 }
// S形曲线差速控制跷跷板前段函数
// 输入：
//    初始速度，目标速度，总需时间（ms）
// 功能：
//    用于平滑变速，不带色标校正（防止一上来误差累计过大直接偏出去）
void S_regulate_teeterboard_nocorrect(float start_speed, float target_speed,float record_angle,uint32_t total_time){
	// 定义 S 型曲线的斜率参数
    float k = 0.02; 
	uint32_t start_time;
	uint32_t current_time;
	
    // 计算速度变化到一半时的时间
    float t_mid = total_time / 2;
	current_time = read_time();
	start_time = read_time();
	while(current_time < start_time + total_time){
		current_time = read_time();
		// 计算 S 型曲线的因子
		float s_factor = 1 / (1 + exp(-k * ((current_time - start_time) - t_mid)));

		// 计算当前速度
		float current_speed = start_speed + (target_speed - start_speed) * s_factor;
		
		walkspin_dynamic(record_angle,current_speed);
	}
	
}
 // S形曲线差速控制长桥函数
 // 输入：
 //    初始速度，目标速度，总需时间（ms）
 // 功能：
 //    用于平滑变速，带色标校正
 void S_regulate_moutain(float start_speed, float target_speed,float record_angle,uint32_t total_time){
 	// 定义 S 型曲线的斜率参数
     float k = 0.02; 
 	uint32_t start_time;
 	uint32_t current_time;
 	uint16_t angle_limit_flag = 0;
 	uint16_t angle_limit = 20;
     // 计算速度变化到一半时的时间
     float t_mid = total_time / 2;
 	current_time = read_time();
 	start_time = read_time();
 	while(current_time < start_time + total_time){
 		current_time = read_time();
 		// 计算 S 型曲线的因子
 		float s_factor = 1 / (1 + exp(-k * ((current_time - start_time) - t_mid)));

 		// 计算当前速度
 		float current_speed = start_speed + (target_speed - start_speed) * s_factor;
		
		if(record_angle > 360){record_angle -= 360;}	
		else if(record_angle < 0){record_angle += 360;}
// 		if(current_time > t_mid)
// 		{
//			
// 			if (RF == 0 && angle_limit_flag < angle_limit){
// 				record_angle += 0.1f;
// 				angle_limit_flag++;
// 				if(record_angle > 360){record_angle -= 360;}
// 			}
// 			else if(LF == 0 && angle_limit_flag < angle_limit){
// 				record_angle -= 0.1f;
// 				angle_limit_flag++;
// 				if(record_angle < 0){record_angle += 360;}
// 			}
// 		}
//		track_dynamic_Speed_2Line(current_speed);
 		walkspin_dynamic(record_angle,current_speed);
 	}
	
 }
// S形曲线差速控制双线循迹函数
// 输入：
//    初始速度，目标速度，总需时间（ms）
// 功能：
//    用于平滑变速下的循迹
void S_regulate_track2Line(float start_speed, float target_speed, uint32_t total_time){
	// 定义 S 型曲线的斜率参数
    float k = 0.02; 
	uint32_t start_time;
	uint32_t current_time;
    // 计算速度变化到一半时的时间
    float t_mid = total_time / 2;
	current_time = read_time();
	start_time = read_time();
	
	while(current_time < start_time + total_time){
		current_time = read_time();
		// 计算 S 型曲线的因子
		float s_factor = 1 / (1 + exp(-k * ((current_time - start_time) - t_mid)));

		// 计算当前速度
		float current_speed = start_speed + (target_speed - start_speed) * s_factor;

		track_dynamic_Speed_2Line(current_speed);
		//OLED_ShowNum(0,0,current_speed,4);
	}
	
}

