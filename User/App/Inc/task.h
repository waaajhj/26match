#ifndef TASK_H_
#define TASK_H_

/**
 * @brief 任务2运行阶段，用于主循环依次切换+5 cm和-5 cm目标。
 */
typedef enum
{
    TASK_2_STAGE_IDLE = 0,
    TASK_2_STAGE_TO_POSITIVE_5CM,
    TASK_2_STAGE_TO_NEGATIVE_5CM
} Task2Stage_e;

// 任务编排接口；具体底盘和球杆控制实现在motion_control模块中。
void task_1(void);
void task_2(void);
void task_2_special(void);
void task_2_update(void);
void task_3(void);
void task_4(void);
void task_5(void);
void task_switch(void);

// 任务2阶段由主循环更新，保留为全局量以便Keil Watch观察。
extern volatile Task2Stage_e task_2_stage;

#endif /* TASK_H_ */
