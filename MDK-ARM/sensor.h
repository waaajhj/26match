#ifndef SENSOR_H_
#define SENSOR_H_
#include "gpio.h"
#include "main.h"
float bais_judgment(void);
float bais_judgment_side(void);
void judge_sensor(void);
int sensor_F(void);
int sensor_side(void);
void judge_cross(void);
int judge_sensor_M(void);
void line_detect_task(void);
void update_sensor_flag(int *sensor_array, int *index, int *flag); 
void judge_sensor_side(void);
int judge_sensor_M2(void);
float bais_judgment_2Line(void);
uint8_t scan(void);
uint8_t scan_2Line(void);
uint8_t left_scan(void);
int scan_cross(uint8_t *gray_data, int start_speed, int delay_stop);
int scan_cross_nostop(uint8_t *gray_data);
int scan_left_cross(uint8_t *gray_data, int start_speed, int delay_stop);
int scan_right_cross(uint8_t *gray_data, int start_speed, int delay_stop);
int scan_three_cross(uint8_t *gray_data);
int scan_cross_early(uint8_t *gray_data);
uint8_t scan_line(void);
uint8_t scan_right_cross_2L(void);
uint8_t scan_left_cross_2L(void);
int scan_high_bit_single(uint8_t *gray_data);
uint8_t right_scan(void);
typedef enum {
    STATE_NORMAL,     
    STATE_IN_CROSS,   
    STATE_out_MID,     
    STATE_out_sensor, 
} GrabState;
typedef enum {
    mode_front,
    mode_side
} MODE_SENSOR;
extern int scan_flag;
//float state_bais[10]={-85,-50, -25, -10, 0, 0, 10, 25, 50, 85};
#define R1 HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14)
#define R2 HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13)
#define R3 HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12)
#define R4 HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11)
#define R5 HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)
#define M1 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)
#define M2 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)
#define L5 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5)
#define L4 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)
#define L3 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)
#define L2 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)
#define L1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)
#define BIAS_R1 (-30.0f)
#define BIAS_R2 (-15.0f)
#define BIAS_R3 (0.0f)
#define BIAS_R4 (15.0f)
#define BIAS_R5 (30.0f)
#define BIAS_M1 (-5.0f) 
#define BIAS_M2 (5.0f)
#define BIAS_L5 (-30.0f)
#define BIAS_L4 (-15.0f)
#define BIAS_L3 (0.0f)
#define BIAS_L2 (15.0f)
#define BIAS_L1 (30.0f)
#define SENSOR_COUNT 10
#define SENSOR_THRESHOLD 5
typedef struct _Position{
    int16_t x; 
    int16_t y;  
    int dir; 
	GrabState current_state;
    MODE_SENSOR mode_sensor;
    int enable_flag;
    int cross_num;
}Position;
extern Position pos;
extern int sensor_flag;
extern int sensor_num[SENSOR_COUNT];
extern int cross_num[SENSOR_COUNT];
extern int sensor_index;
extern int cross_index;
extern int cross_flag;
extern int angle_flag;
extern int side_flag;
extern int mid_flag;
//PID_Position_Struct pid_sensor={0};
#endif /* SENSOR_H_ */
