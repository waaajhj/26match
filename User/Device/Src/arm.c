#include "arm.h"
int flag_catch=0;
void Arm_Init(void){
	// 当前版本不再使用舵机，保留空接口以兼容旧路线代码。
}
void Angle1(float angle1){//大臂运动,越小越往前
	// 舵机已移除，不再占用 TIM1。
	(void)angle1;
}
void Angle2(float angle2){//小臂运动越小越往后
	// 舵机已移除，不再占用 TIM2。
	(void)angle2;
}
void Arm_up(void){
	Angle1(95);
  Angle2(100);
}
void Arm_down(void){
	Angle1(6);
	Angle2(172);//左边
}
