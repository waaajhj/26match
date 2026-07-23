#include "arm.h"
#include "tim.h"
int flag_catch=0;
void Arm_Init(void){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 	//����PWM
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	Arm_up();
}
void Angle1(float angle1){//大臂运动,越小越往前
	float pluse;
	pluse=(angle1/180)*2000+500;
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pluse);
}
void Angle2(float angle2){//小臂运动越小越往后
	float pluse;
	pluse=(angle2/180)*2000+500;
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,pluse);
}
void Arm_up(void){
	Angle1(95);
  Angle2(100);
}
void Arm_down(void){
	Angle1(6);
	Angle2(172);//左边
}