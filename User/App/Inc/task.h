#ifndef TASK_H_
#define TASK_H_
#include "stm32f4xx_hal.h"


extern uint16_t door_nostop_exit_speed;
/*----------------------外部接口函数---------------------------*/

void updown_low_stair_old(void);		//上下低台（旧的）
void up_low_stair(void);				//上低台
void down_low_stair(void);				//下低台
void updown_low_stair(void);			//上下低台（新的）
void updown_low_stair2(void);			//上下2号低台
void updown_low_stair6(void);
void updown_middle_stair(void);			//上下中台

void updown_high_stair(void);			//上下高台

void climb_moutain(void);	//爬长桥
void obstacle_three_goto(void);					//过三连障碍
void obstacle_three_return(void);					//过三连障碍
void obstacle_stair(void);	//过梯形山
void obstacle_goto(void);
void teeterboard(float angle,float correct_angle);				//过跷跷板
void teeterboard_move(void);				//移动过跷跷板
void teeterboard_90(float correct_angle);
void teeterboard_270(void);
void door(void);							//过门
void door_nostop(void);
void cliff(float left_correct_angle);
void cliff_new(float left_correct_angle);
void slope(void);
float updown_stair_error_2L(void);	// 角度环灰度误差计算函数（双线）
float updown_stair_error_new(void);	// 角度环灰度误差计算函数（新的）
void vision_init(void);
void vision_start(void);
void vision_stop(void);
void updown_low_stair6(void);
int vision_choose_cross(void);
#endif /* TASK_H_ */
