#ifndef _ROUTE_H
#define _ROUTE_H

#include "main.h"

/*----------------------引脚定义---------------------------*/



/*----------------------外部接口函数---------------------------*/
void begin_part(void);			// 起始部分函数
void goto_43_part(void);		// 去往4号平台和3号平台函数
void goto_42_part(void);		// 去往4号平台和2号景点函数
void goto_31_part(void);		// 去往3号平台和1号景点函数
void across_door_goto(void);	// 穿门函数（前半程）
void goto_78_part(void);		// 去往78平台的函数
void goto_65_part_new(void);		// 去往65平台的函数
void across_door_return(void);	// 穿门函数（后半程）
void return_part(void);			// 返回函数
#endif