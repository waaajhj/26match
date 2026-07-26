// #ifndef __MAIXCAM_H
// #define __MAIXCAM_H
// #include "stm32f4xx_hal.h"
// // 数据包结构定义（总长21字节）
// #pragma pack(push, 1)
// typedef struct {
//     uint8_t header;     // 包头 (0xAA)
//     uint16_t data_len;  // 数据长度 (小端模式，含校验和)
//     uint32_t point1_x;  // 矩形端点1X坐标 (小端模式)
//     uint32_t point1_y;  // 矩形端点1Y坐标
//     uint32_t point2_x;  // 矩形端点2X坐标
//     uint32_t point2_y;  // 矩形端点2Y坐标
//     uint32_t point3_x;  // 矩形端点3坐标 (小端模式)
//     uint32_t point3_y;  // 矩形端点3坐标
//     uint32_t point4_x;  // 矩形端点4坐标
//     uint32_t point4_y;  // 矩形端点4坐标
//     uint8_t checksum;   // 校验和 (前面18字节累加和)
//     uint8_t footer;     // 包尾 (0x55)
// } PointPacket1;
// extern PointPacket1 rx_packet;
// typedef struct {
//     uint8_t header;     // 包头 (0xAA)
//     uint16_t data_len;  // 数据长度 (小端模式，含校验和)
//     uint32_t point_x;  // 矩形端点1X坐标 (小端模式)
//     uint32_t point_y;  // 矩形端点1Y坐标
//     uint8_t checksum;   // 校验和 (前面18字节累加和)
//     uint8_t footer;     // 包尾 (0x55)
// } PointPacket2;
// typedef struct {
//     uint8_t header;     // 包头 (0xAA)
//     uint16_t centerpoint_x;  // 矩形端点1X坐标 (小端模式)
//     uint16_t centerpoint_y;  // 矩形端点1Y坐标
//      uint8_t footer;     // 包尾 (0x55)
// } PointPacket3;
// extern PointPacket3 point_packet;
// extern PointPacket2 laser_packet; // 激光点数据包
// #pragma pack(pop)
// extern int laser_x,laser_y,target_x,target_y;
// extern int center_points[2];
// // 解包状态机
// typedef enum {
//     PKT_ture,
//     PKT_flase,
// } PacketState;
// typedef enum {
//     PKT_rect,//接收矩形坐标
//     PKT_laster, //接收激光点坐标
// } PacketState2;
// extern volatile PacketState pkt_state ;
// extern volatile PacketState2 pkt_state2 ;
// extern uint32_t interrupt_count ;
// extern uint8_t x_arrived;
// void Hal_Uart_Init(void);
// uint8_t CalculateChecksum(PointPacket1 *pkt);
// void ProcessPacket(void);
// extern int point_2[2][2];
// //void jy61p_ReceiveData(uint8_t RxData);
// //void BT24_ReceiveData(uint8_t RxData);
// //void delay_ms(uint32_t ms);
// //void JY61P_BAUD(void);
// //void JY61P_START(void);
// //extern float Roll,Pitch,Yaw;
// //extern int Start_Flag;
// #endif
#ifndef __MAIXCAM_H
#define __MAIXCAM_H

#include "stm32f4xx_hal.h"

// 定义数据包结构（确保与发送端一致）
typedef struct {
    uint8_t header;         // 包头 0xAA
    uint16_t centerpoint_x;  // 中心点X坐标
    uint8_t footer;         // 包尾 0x55
} __attribute__((packed)) PointPacket3;

// 接收状态枚举
typedef enum {
    WAIT_HEADER,      // 等待包头
    RECEIVE_DATA,     // 接收数据
    PACKET_COMPLETE   // 接收完成
} ReceiveState;

// 函数声明
void Hal_Uart_Init(void);
void ProcessPacket(void);
int FindPacket(void) ;
int isCoordinateValid(int x, int y);
// 外部变量声明
// point_packet只在收到包头、包尾正确的完整数据包后更新
extern volatile ReceiveState rx_state;
extern volatile PointPacket3 point_packet;
extern volatile uint8_t point_packet_ready;
extern volatile uint32_t point_packet_rx_count;
extern volatile uint32_t point_packet_error_count;
extern volatile uint32_t uart4_rx_byte_count;
extern volatile uint32_t uart4_last_receive_status;
extern volatile uint32_t uart4_last_error_code;
extern int center_points[2];
extern uint8_t x_arrived;

#define U3_DATASIZE  4
extern uint8_t U3_rx_data[U3_DATASIZE];

#define U2_DATASIZE  5
extern uint8_t U2_rx_data[U2_DATASIZE];
extern uint8_t U2_byte_buf[U2_DATASIZE];
extern uint8_t U2_RxByte;
void usart6_receive(uint8_t data[]);
extern uint8_t line[12];
void usart6_send(uint8_t *str);
void usart2_receive(uint8_t data[]);
void usart2_send(uint8_t *str);
extern uint8_t choose_cross_flag;
extern uint16_t teeterboard_vision_error;
#endif
