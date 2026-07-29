#include "sensor.h"
#include "tim.h"
#include "OLED.h"
#include "pid.h"
#include "jy61p.h"
#include "chassis.h"
#include "DJI_Motor.h"
#include"maixcam.h"
int sensor_num[SENSOR_COUNT] = {0};
int cross_num[SENSOR_COUNT] = {0};
int side_num[SENSOR_COUNT] = {0};
int sensor_index = 0;
int cross_index = 0;
int side_index = 0;
int angle_flag = 0, cross_flag = 0, side_flag = 0, mid_flag = 0, sensor_flag=0;

Position pos = {0, 0, 0, STATE_NORMAL,mode_front,1, 0};

void update_sensor_flag(int *sensor_array, int *index, int *flag) {
    int sum = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sum += sensor_array[i];
        sensor_array[i] = 0;
    }
    *flag = (sum >= SENSOR_THRESHOLD);
    *index = 0;
}
int scan_flag = 1; // 定义全局扫描标志
/**
 * @brief 判断偏差值
 * 
 * 通过读取一系列传感器的状态（R1, R2, R3, R4, M1, M2, L1, L2, L3, L4），
 * 计算并返回一个偏差值，用于表示当前状态的偏离程度。
 * 
 * @return 偏差值，正负表示偏离的方向和程度。
 */
float bais_judgment(void) {
    float bais1 = 0.0f;
    int count = 0;
    int light_indices[12];
    int n = 0;

    // 第一步：记录所有有效传感器的索引（注意 line[0] 特殊）
    for (int i = 0; i < 12; i++) {
        if ((i == 0 && line[0] == 0) || (i > 0 && line[i] == scan_flag)) {
            light_indices[n++] = i;
        }
    }

    // 第二步：累加偏差（保持原逻辑）
    if (line[11] == scan_flag) { bais1 += -75; count++; }
    if (line[10] == scan_flag) { bais1 += -60; count++; }
    if (line[9]  == scan_flag) { bais1 += -45; count++; }
    if (line[8]  == scan_flag) { bais1 += -30; count++; }
    if (line[7]  == scan_flag) { bais1 += -15; count++; }
    if (line[6]  == scan_flag) { bais1 += BIAS_M1; count++; }
    if (line[5]  == scan_flag) { bais1 += BIAS_M2; count++; }
    if (line[4]  == scan_flag) { bais1 += 15; count++; }
    if (line[3]  == scan_flag) { bais1 += 30; count++; }
    if (line[2]  == scan_flag) { bais1 += 45; count++; }
    if (line[1]  == scan_flag) { bais1 += 60; count++; }
    if (line[0]  == scan_flag) { bais1 += 70; count++; }

    // // 第三步：连续性检测（对索引升序判断）
    // if (count >= 2) {
    //     // light_indices 已按 i 从小到大存入（0~11），所以是升序
    //     uint8_t continuous = 1;
    //     for (int i = 1; i < n; i++) {
    //         if (light_indices[i] - light_indices[i-1] > 1) {
    //             continuous = 0;
    //             break;
    //         }
    //     }
    //     if (!continuous) {
    //         return 0.0f; // 不连续，返回0
    //     }
    // }

    // 第四步：原有数量滤波
    if (count > 0 && count < 3) {
        bais1 /= count;
    } else if (count >= 3) {
        bais1 = 0;
    }

    return bais1;
}
//双线检测灰度偏差值计算
float bais_judgment_2Line(void) {
    int count_left = 0, count_right = 0;
    float left_sum = 0.0f, right_sum = 0.0f;

    // --- 检测左侧线（L1~L5）---
    if (L1 == scan_flag) { left_sum += BIAS_L1; count_left++; }
    if (L2 == scan_flag) { left_sum += BIAS_L2; count_left++; }
    if (L3 == scan_flag) { left_sum += BIAS_L3; count_left++; }
    if (L4 == scan_flag) { left_sum += BIAS_L4; count_left++; }
    if (L5 == scan_flag) { left_sum += BIAS_L5; count_left++; }

    // --- 检测右侧线（R1~R5）---
    if (R1 == scan_flag) { right_sum += BIAS_R1; count_right++; }
    if (R2 == scan_flag) { right_sum += BIAS_R2; count_right++; }
    if (R3 == scan_flag) { right_sum += BIAS_R3; count_right++; }
    if (R4 == scan_flag) { right_sum += BIAS_R4; count_right++; }
    if (R5 == scan_flag) { right_sum += BIAS_R5; count_right++; }

    // --- 滤波：单侧超过2个点视为无效（可能是十字路口）---
    if (count_left > 2) { count_left = 0; left_sum = 0; }
    if (count_right > 2) { count_right = 0; right_sum = 0; }

    // --- 核心逻辑：根据有效侧决定模式 ---
    if (count_left > 0 && count_right > 0) {
        // 双线模式：使用左右偏差差值（标准双线循迹）
        float left_avg = left_sum / count_left;
        float right_avg = right_sum / count_right;
        return -left_avg - right_avg;  // 负值：偏右；正值：偏左
    }
    else if (count_left > 0) {
        // 仅左线有效 → 切换为单线循迹，以左线中心为目标
        return -left_sum / count_left;  // 直接使用左线偏差（负值表示车在左线右侧，需左转）
    }
    else if (count_right > 0) {
        // 仅右线有效 → 切换为单线循迹，以右线中心为目标
        return -right_sum / count_right; // 正值表示车在右线左侧，需右转
    }
    else {
        // 两侧都无信号
        return 0.0f;
    }
}
/**
 * @brief 检测前方传感器状态
 * 
 * 检查一系列传感器的状态，
 * 如果任意一个传感器检测到信号，则返回0，否则返回1。
 * 
 * @return int 0表示检测到信号，1表示未检测到信号。
 */
// 扫线函数
// 只要扫到线就返回 0 否则返回 1
uint8_t scan(void)
{
    if (line[0] == scan_flag || line[1] == scan_flag || line[2] == scan_flag || line[3] == scan_flag
        || line[4] == scan_flag || line[5] == scan_flag || line[6] == scan_flag || line[7] == scan_flag
        || line[1] == scan_flag || line[9] == scan_flag || line[10] == scan_flag || line[11] == scan_flag) {
        return 0;    
    } else {
        return 1;
    }
}
// 扫线函数
// 只要扫到线就返回 0 否则返回 1
uint8_t scan_2Line(void)
{
    if ((L1 == scan_flag || L2 == scan_flag || L3 == scan_flag || L4 == scan_flag || L5 == scan_flag )
    || (R1 == scan_flag || R2 == scan_flag || R3 == scan_flag || R4 == scan_flag || R5 == scan_flag )) {
        return 0;    
    } else {
        return 1;
    }
}
// 扫线函数
// 只要扫到线就返回 0 否则返回 1
uint8_t left_scan(void) {
    for (int i = 0; i < 4; i++) {
        if (line[i] == 0) {
            return 1;
        }
    }
    return 0;
}    
uint8_t right_scan(void) {
    for (int i = 9; i < 12; i++) {
        if (line[i] == 0) {
            return 1;
        }
    }
    return 0;
} 
uint8_t scan_line_flag = 0; // 循迹标志
// 检测line数组判断是否寻到线
// 功能：
//    作为 while 标志使用，检测车头是否寻到线，连续 3 次未检测线到返回 0，否则返回 1
uint8_t scan_line(void)
{
    uint8_t i = 0;

    for(i = 0; i < 12; i++)
    {
        if(line[i] == 0){
            //scan_line_flag = 0;
            return 1;
        }
    }
//    scan_line_flag++;
//    if(scan_line_flag >= 4) { // 滤波常数，过小会导致循迹不稳定，过大会导致上坡速度切换不稳定
//        scan_line_flag = 0;
//        return 0;
//    }
    return 0;
}
// 判断是否到达单路口函数（右侧灰度寻双线）
uint8_t scan_right_cross_2L(void){
    int count_right = 0;
    if (R1 == scan_flag) { count_right++; }
    if (R2 == scan_flag) { count_right++; }
    if (R3 == scan_flag) { count_right++; }
    if (R4 == scan_flag) { count_right++; }
    if (R5 == scan_flag) { count_right++; }
    if (count_right > 2) {
        return 0; // 检测到右侧有2个以上元素为0，认为到达路口
    }
    return 1;
}
uint8_t scan_left_cross_2L(void){
    int count_left = 0;
    if (L1 == scan_flag) { count_left++; }
    if (L2 == scan_flag) { count_left++; }
    if (L3 == scan_flag) { count_left++; }
    if (L4 == scan_flag) { count_left++; }
    if (L5 == scan_flag) { count_left++; }
    if (count_left > 2) {
        return 0; // 检测到左侧有2个以上元素为0，认为到达路口
    }
    return 1;
}
/**
 * @brief 判断前方传感器状态
 * 
 * 通过多次检测前方传感器的状态，统计有效信号的次数，并更新前方标志位。
 */
void judge_sensor(void) {
    if (scan() == 0) {
        sensor_num[sensor_index++] = 1; // 记录有效信号
    }
    if (sensor_index == SENSOR_COUNT) {
        update_sensor_flag(sensor_num, &sensor_index, &sensor_flag); // 更新标志位
    }
}

/**
 * @brief 判断是否检测到十字路口
 * 
 * 通过一系列传感器的状态判断是否检测到十字路口，并更新十字路口标志位。
 */
void judge_cross(void) {
    int flag = (R1 + R2 + R3 + R4 + M1 + M2 + L1 + L2 + L3 + L4 >= 5);
    if (flag) {
        cross_num[cross_index++] = 1; // 记录有效信号
    }
    if (cross_index == SENSOR_COUNT) {
        update_sensor_flag(cross_num, &cross_index, &cross_flag); // 更新标志位
    }
}
uint8_t scan_cross_flag = 0;
int scan_cross(uint8_t *gray_data, int start_speed, int delay_stop){
    if (scan_cross_flag >= 1) { // 滤波次数
        scan_cross_flag = 0;
        
		S_regulate_Ctl(start_speed,0,delay_stop);
		
        return 0;
    }

    int light_indices[12] = {0}; // 存储亮起的灰度通道索引
    int count_light = 0;         // 亮起的通道数量

    // 收集所有亮起的通道索引（假设0表示亮起）
    for (int i = 1; i < 11; i++) {
        if (*(gray_data + i) == 0) {
            light_indices[count_light++] = i;
        }
    }

    // 至少需要两路亮起才进行不连续判断
    if (count_light >= 2) {
        uint8_t has_discontinuous = 0;
        // 检查所有两两组合是否存在不连续情况
        for (int j = 0; j < count_light; j++) {
            for (int k = j + 1; k < count_light; k++) {
                if (abs(light_indices[j] - light_indices[k]) > 1) {
                    has_discontinuous = 1;
                    break; // 只要找到一对即可
                }
            }
            if (has_discontinuous) break;
        }

        if (has_discontinuous) {
            scan_cross_flag++;
        } else {
            scan_cross_flag = 0;
        }
    } else {
        scan_cross_flag = 0; // 不足两路直接重置
    }

    return 1;  
}
int scan_cross_nostop(uint8_t *gray_data) 
{
    if (scan_cross_flag >= 1) { // 滤波次数
        scan_cross_flag = 0;
		
        return 0;
    }

    int light_indices[12] = {0}; // 存储亮起的灰度通道索引
    int count_light = 0;         // 亮起的通道数量

    // 收集所有亮起的通道索引（假设0表示亮起）
    for (int i = 2; i < 10; i++) {
        if (*(gray_data + i) == 1) {
            light_indices[count_light++] = i;
        }
    }

    // 至少需要两路亮起才进行不连续判断
    if (count_light >= 2) {
        uint8_t has_discontinuous = 0;
        // 检查所有两两组合是否存在不连续情况
        for (int j = 0; j < count_light; j++) {
            for (int k = j + 1; k < count_light; k++) {
                if (abs(light_indices[j] - light_indices[k]) > 1) {
                    has_discontinuous = 1;
                    break; // 只要找到一对即可
                }
            }
            if (has_discontinuous) break;
        }

        if (has_discontinuous) {
            scan_cross_flag++;
        } else {
            scan_cross_flag = 0;
        }
    } else {
        scan_cross_flag = 0; // 不足两路直接重置
    }
    return 1;  
} 
// 判断是否到达单路口函数
// 输入：
//    gray_data：存储 12 路灰度数据的数组的指针
// 功能：
//    检查左侧是否有 2 个以上元素为 0，若是则认为到达路口
int scan_left_cross(uint8_t *gray_data, int start_speed, int delay_stop) 
{
    if(scan_cross_flag >= 1) { // 滤波次数
        scan_cross_flag = 0;
        
		S_regulate_Ctl(start_speed,0,delay_stop);
		
        return 0;
    }
    int count = 0;
    // 只看中间六个
    for (int i = 0; i < 4; i++) {
        count += (*(gray_data + i) == 0);  // 扫到线时 count 加 1
    }

    if (count > 1) {
        scan_cross_flag++;
    }
    else {
        scan_cross_flag = 0;
    }

    return 1;  
} 
int scan_three_cross(uint8_t *gray_data) 
{
    if(scan_cross_flag >= 3) { // 滤波次数
        scan_cross_flag = 0;
        uint16_t i;
        for(i = 7000; i >= 2000; i -= 300) {
            calculate_motor_speeds(i,0);
        }
        return 0;
    }
    int count = 0;
    // 只看中间六个
    for (int i = 4; i < 12; i++) {
        count += (*(gray_data + i) == 0);  // 扫到线时 count 加 1
    }

    if (count > 2) {
        scan_cross_flag++;
    }
    else {
        scan_cross_flag = 0;
    }

    return 1;  
} 
/**
 * @brief 检测高位两个灰度传感器是否任一亮起（值为0）
 * 
 * 若 gray_data[10] 或 gray_data[11] 中任意一个为0（检测到黑线），
 * 则认为条件满足，返回0；否则返回1。
 * 
 * @param gray_data: 指向12路灰度数据的指针
 * @return int: 0 表示高位有信号，1 表示无信号
 */
int scan_high_bit_single(uint8_t *gray_data)
{
    // 检查高位两个通道（索引10和11）是否任一为0（黑线）
    if (*(gray_data + 10) == 0 || *(gray_data + 11) == 0) {
        return 0; // 任一亮起，返回0
    }
    return 1; // 都未亮起，返回1
}
// 判断是否到达单路口函数
// 输入：
//    gray_data：存储 12 路灰度数据的数组的指针
// 功能：
//    检查左侧是否有 2 个以上元素为 0，若是则认为到达路口
int scan_right_cross(uint8_t *gray_data, int start_speed, int delay_stop) 
{
    if(scan_cross_flag >= 1) { // 滤波次数
        scan_cross_flag = 0;
        
        S_regulate_Ctl(start_speed,0,delay_stop);
		
        return 0;
    }
    int count = 0;
    // 只看中间六个
    for (int i = 8; i < 12; i++) {
        count += (*(gray_data + i) == 0);  // 扫到线时 count 加 1
    }

    if (count > 1) {
        scan_cross_flag++;
    }
    else {
        scan_cross_flag = 0;
    }

    return 1;  
} 
int scan_cross_early(uint8_t *gray_data) 
{
    if(scan_cross_flag >= 1) { // 滤波次数
        scan_cross_flag = 0;
        uint16_t i;
        for(i = 6000; i >= 2000; i -= 150) {
            calculate_motor_speeds(i,0);
        }
        return 0;
    }

    int count = 0;
    // 只看中间六个
    for (int i = 4; i < 12; i++) {
        count += (*(gray_data + i) == 0);  // 扫到线时 count 加 1
    }

    if (count > 2) {
        scan_cross_flag++;
    }
    else {
        scan_cross_flag = 0;
    }

    return 1;  
} 

