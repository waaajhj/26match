#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "stdint.h"
#define speed_sensor 5000
#define speed_sensor2 5000
#define ad_num 30
//=====================================================================================
//============================角度部分=========================================
//=====================================================================================
float spin_error(float traget_angle);
void stand_spin(float angle);			//自转角度环
void stand_spin_new(float angle,int max_speed);
void stand_spin_dynamic(float angle,int max_speed);
void low_walkspin(float angle);			//低速前进角度环
void low_walk_spin(float angle);
void walkspin(float angle);				//高速前进角度环
void walkspin_old(float angle, float speed);		//动态前进角度环（旧的）
void walkspin_dynamic(float angle, float speed);	//动态前进角度环（新的）
void walkspin_dynamic_behind(float target_angle, float speed);
void fuzzy_pid_init(void);
void fuzzy_angle_pid(float target_angle);
void fuzzy_spin(float angle);
void fuzzy_spin180(void);
void spin(float angle);					//原地自转函数
void spin_fast(float angle);
void turn_left(float angle);			//左转函数
void turn_right(float angle);			//右转函数
void walk_spin_left(float angle);		//从当前角度向左前进转弯指定角度
void walk_spin_right(float angle);		//从当前角度向右前进转弯指定角度
void walk_spin(float angle);			//向指定角度进行前进转弯
void spin180(void);						//自转180度
float angle_restrict(float angle);
//=====================================================================================
//============================循迹部分=========================================
//=====================================================================================
void track_dynamic_newT(float speed);
uint8_t scan(void);						//扫线函数
uint8_t left_scan(void);
uint8_t scan_stair(void);				//台阶扫描函数
uint8_t scan_line(void);				//巡线扫描函数
int scan_cross_old(uint8_t *gray_data);
int scan_cross(uint8_t *gray_data, int start_speed, int delay_stop);	//路口扫描函数
int scan_cross_nostop(uint8_t *gray_data);
int scan_three_cross(uint8_t *gray_data);
int scan_left_cross(uint8_t *gray_data, int start_speed, int delay_stop);
int scan_right_cross(uint8_t *gray_data, int start_speed, int delay_stop);
int scan_cross_early(uint8_t *gray_data);
void walkspin_track_dynamic(float target_angle, float speed);
void track_lowSpeed(void);				//低速PID
void track_highSpeed(void);				//高速PID
void track_lowSpeed_count(int time);	//低速PID进阶，可以循迹指定时间
void track_highSpeed_count(int time);	//高速PID进阶，可以循迹指定时间
void track_dynamic_Speed(float speed);	//动态循迹
void track_dynamic_new(float speed);	//新循迹，脱线不会停
void track_dynamic_left(float speed);
void track_dynamic_right(float speed);
void tumble(void);
void track_dynamic_Speed_2Line(float speed);	// 双线循迹
void track_2L_count(int time);
void walkspin_track_dynamic_2L(float target_angle, float speed);
float calculate_angle_error_fixed(float current, float target);
extern float bais;
extern float W_out;
extern uint32_t time;
extern float target_bais;
//=====================================================================================
//============================S形曲线变速部分=========================================
//=====================================================================================
void S_regulate_Ctl(float start_speed, float target_speed,uint32_t total_time);					//S形曲线差速控制电机函数
void S_regulate_track(float start_speed, float target_speed, uint32_t total_time);
void S_regulate_track_with_slope(float start_speed, float target_speed,
                                 uint32_t total_time, float slope_per_ms);
void S_regulate_track_left(float start_speed, float target_speed, uint32_t total_time);//S形曲线差速控制循迹函数
void S_regulate_track_right(float start_speed, float target_speed, uint32_t total_time);
void S_regulate_walkspin(float start_speed, float target_speed,float angle,uint32_t total_time);	//S形曲线差速控制角度环函数
void S_regulate_teeterboard(float start_speed, float target_speed,float record_angle,uint32_t total_time);			//S形曲线差速控制跷跷板函数
void S_regulate_teeterboard_nocorrect(float start_speed, float target_speed,float record_angle,uint32_t total_time);//S形曲线差速控制跷跷板前段函数
void S_regulate_moutain(float start_speed, float target_speed,float record_angle,uint32_t total_time);				// S形曲线差速控制长桥函数
void S_regulate_track2Line(float start_speed, float target_speed, uint32_t total_time);	// S形曲线差速控制双线循迹函数
#define LF HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)
#define RF HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)
#define Start_flag HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)
#endif
