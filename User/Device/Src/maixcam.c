#include "maixcam.h"
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "jy61p.h"
#include "tim.h"
// 环形缓冲区
#define RING_BUFFER_SIZE 128
typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} RingBuffer;

static RingBuffer rx_ring_buffer = {0};
PointPacket3 point_packet;

// 添加滤波相关变量
static int last_valid_center_points[2] = {0, 0};  // 上一次有效的坐标
static uint32_t last_valid_time = 0;              // 上一次有效数据的时间
static uint8_t first_valid_data = 0;              // 是否收到第一次有效数据

uint8_t x_arrived = 0;
int center_points[2] = {0};
int point_2[2][2] = {0};
uint32_t interrupt_count = 0;
uint8_t RxData;
uint8_t RxData_101;
uint8_t line[12];	
uint8_t U3_rx_data[U3_DATASIZE];
uint8_t U2_rx_data[U2_DATASIZE];
uint8_t U2_RxByte = 0;
uint8_t U2_byte_buf[U2_DATASIZE];
static volatile uint16_t U2_byte_index = 0;

typedef enum {
    U2_WAIT_AA = 0,
    U2_WAIT_BB,
    U2_RECV_PAYLOAD
} U2_RxState_t;
static volatile U2_RxState_t u2_state = U2_WAIT_AA;
// 蓝牙串口接收缓冲区

// 最大允许的坐标变化值（可根据实际需求调整）
#define MAX_COORDINATE_JUMP 100
// 最大数据间隔时间（毫秒）
#define MAX_DATA_INTERVAL 500

// 环形缓冲区操作函数
static inline void ring_buffer_put(RingBuffer *rb, uint8_t data) {
    if(rb->count < RING_BUFFER_SIZE) {
        rb->buffer[rb->head] = data;
        rb->head = (rb->head + 1) % RING_BUFFER_SIZE;
        rb->count++;
    }
}

static inline int ring_buffer_get(RingBuffer *rb, uint8_t *data) {
    if(rb->count > 0) {
        *data = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
        rb->count--;
        return 1;
    }
    return 0;
}

void Hal_Uart_Init(void){
    // 使用单字节接收，但通过环形缓冲区提高效率
//    HAL_UART_Receive_IT(&huart2, &rx_ring_buffer.buffer[rx_ring_buffer.head], 1);
	  HAL_UART_Receive_IT(&huart3, &RxData, 1);  // 接收1个字节数据
      HAL_UART_Receive_IT(&huart2, &RxData_101, 1);  // 接收1个字节数据
	  HAL_UART_Receive_IT(&huart6, (uint8_t *)U3_rx_data, U3_DATASIZE);
	  HAL_UART_Receive_IT(&huart4, &U2_RxByte, 1);
}

// 检查坐标数据是否合理
int isCoordinateValid(int x, int y) {
    // 检查坐标是否在合理范围内（根据你的摄像头分辨率调整）
    if(x < 0 || x > 320 || y < 0 || y > 240) {
        return 0;
    }
    
    // 如果是第一次收到数据，直接认为有效
    if(!first_valid_data) {
        return 1;
    }
    
    // 检查坐标跳变是否过大
    int x_diff = x - last_valid_center_points[0];
    int y_diff = y - last_valid_center_points[1];
    
    if(x_diff > MAX_COORDINATE_JUMP || x_diff < -MAX_COORDINATE_JUMP ||
       y_diff > MAX_COORDINATE_JUMP || y_diff < -MAX_COORDINATE_JUMP) {
        return 0;
    }
    
    return 1;
}

// 查找数据包
int FindPacket(void) {
    // 确保有足够的数据
    if(rx_ring_buffer.count < sizeof(PointPacket3)) {
        return 0;
    }
    
    // 查找包头
    uint16_t search_pos = rx_ring_buffer.tail;
    uint16_t available_count = rx_ring_buffer.count;
    
    for(int i = 0; i <= available_count - sizeof(PointPacket3); i++) {
        if(rx_ring_buffer.buffer[search_pos] == 0xAA) {
            // 检查是否有完整的数据包
            uint16_t end_pos = (search_pos + sizeof(PointPacket3) - 1) % RING_BUFFER_SIZE;
            if(rx_ring_buffer.buffer[end_pos] == 0x55) {
                // 找到完整数据包
                uint8_t *packet_ptr = (uint8_t*)&point_packet;
                for(int j = 0; j < sizeof(PointPacket3); j++) {
                    uint16_t pos = (search_pos + j) % RING_BUFFER_SIZE;
                    packet_ptr[j] = rx_ring_buffer.buffer[pos];
                }
                
                // 移除已处理的数据
                rx_ring_buffer.tail = (search_pos + sizeof(PointPacket3)) % RING_BUFFER_SIZE;
                rx_ring_buffer.count -= sizeof(PointPacket3);
                
                return 1;
            }
        }
        search_pos = (search_pos + 1) % RING_BUFFER_SIZE;
    }
    
    return 0;
}

// 主解包逻辑
void ProcessPacket(void) {
    interrupt_count++;
    
    // 检查包头和包尾是否正确
    if(point_packet.header == 0xAA && point_packet.footer == 0x55) {
        // 获取当前时间
        uint32_t current_time = HAL_GetTick();
        
        // 提取坐标数据
        int new_center_x = point_packet.centerpoint_x;
        int new_center_y = point_packet.centerpoint_y;
        
        // 检查数据是否有效
        if(isCoordinateValid(new_center_x, new_center_y)) {
            // 数据有效，更新坐标
            center_points[0] = new_center_x;
            center_points[1] = new_center_y;
            
            // 保存当前有效数据和时间
            last_valid_center_points[0] = new_center_x;
            last_valid_center_points[1] = new_center_y;
            last_valid_time = current_time;
            first_valid_data = 1;
        } else {
            // 数据无效，检查是否超时
            if(first_valid_data && (current_time - last_valid_time) < MAX_DATA_INTERVAL) {
                // 未超时，保持上一次的有效数据
                center_points[0] = last_valid_center_points[0];
                center_points[1] = last_valid_center_points[1];
            } else {
                // 超时或第一次数据就无效，可以考虑清零或保持默认值
                // 这里选择保持上一次数据（如果有的话）或使用默认值
                if(!first_valid_data) {
                    center_points[0] = 0;
                    center_points[1] = 0;
                } else {
                    center_points[0] = last_valid_center_points[0];
                    center_points[1] = last_valid_center_points[1];
                }
            }
        }
    }
}

void usart6_receive(uint8_t data[])
{
	uint8_t i;
	
    if(data[0]==0x75 && data[3]==0x03) {
		for (i = 0; i < 8; i++) {
			
		
			line[i] = (data[1] >> i) & 1;
		}
		for (i = 0; i < 4; i++) {
			
			
			line[i+8] = (data[2] >> i) & 1;
		}
		
	}
	  for (i = 0; i < 4; i++) {data[i]=0;}
}
// UART接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // 串口2-视觉相关处理 =================================================
    if (huart->Instance == UART4)
    {
        uint8_t b = U2_RxByte;

        switch(u2_state) {
            case U2_WAIT_AA:
                if (b == 0xAA) {
                    U2_byte_buf[0] = 0xAA;
                    u2_state = U2_WAIT_BB;
                }
                break;

            case U2_WAIT_BB:
                if (b == 0xBB) {
                    U2_byte_buf[1] = 0xBB;
                    U2_byte_index = 2;
                    u2_state = U2_RECV_PAYLOAD;
                } else if (b == 0xAA) {
                    // 连续0xAA的情况，保持在等待第二个头
                    U2_byte_buf[0] = 0xAA;
                    u2_state = U2_WAIT_BB;
                } else {
                    // 非期望字节，回到等待 0xAA
                    u2_state = U2_WAIT_AA;
                }
                break;

            case U2_RECV_PAYLOAD:
                if (U2_byte_index < U2_DATASIZE) {
                    U2_byte_buf[U2_byte_index++] = b;
                } else {
                    // 防护：如果索引越界则重置状态
                    u2_state = U2_WAIT_AA;
                    U2_byte_index = 0;
                }

                // 收齐一个完整帧（固定长度）
                if (U2_byte_index >= U2_DATASIZE) {
                    // 简单校验：尾字节为 0xFF（保持和原代码一致）
                    if (U2_byte_buf[U2_DATASIZE - 1] == 0xFF) {
                        usart2_receive(U2_byte_buf);
                    }
                    // 无论是否有效，都重置状态机准备下一个包
                    u2_state = U2_WAIT_AA;
                    U2_byte_index = 0;
                    // 可选择清空缓冲区（非必要）
                    // memset(U2_byte_buf, 0, sizeof(U2_byte_buf));
                }
                break;
        }

        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart4, &U2_RxByte, 1);
        return;
    }
	if(huart->Instance == USART3){//如果是jy61p的数据
		jy61p_ReceiveData(RxData);
		HAL_UART_Receive_IT(&huart3 ,&RxData, 1);
//		HAL_UART_Transmit_IT(&huart1,&RxData,1);
    }
	if(huart->Instance == USART2){//如果是jy61p的数据
		WT101_ReceiveData(RxData_101);
		HAL_UART_Receive_IT(&huart2 ,&RxData_101, 1);
//		HAL_UART_Transmit_IT(&huart1,&RxData,1);
	}
    
    
	if(huart->Instance == USART6){//如果是灰度数据
		HAL_UART_Receive_IT(&huart6, (uint8_t *)U3_rx_data, U3_DATASIZE);
    usart6_receive(U3_rx_data);
	}
}
void usart6_send(uint8_t *str)
{
    uint8_t len = strlen((char *)str); // 获取字符串长度
    unsigned short count = 0;
    
    for (; count < len; count++)
    {
        HAL_UART_Transmit(&huart6, &str[count], 1, 0xffff); // 逐个发送字符
        // 等待发送完成
        // while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY); // 可以根据具体情况选择合适的等待方式
    }
}
//串口2发送函数
void usart2_send(uint8_t *str)
{
    uint8_t len = strlen((char *)str); // 获取字符串长度
    unsigned short count = 0;
    
    for (; count < len; count++)
    {
        HAL_UART_Transmit(&huart4, &str[count], 1, 0xffff); // 逐个发送字符
        // 等待发送完成
        // while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY); // 可以根据具体情况选择合适的等待方式
    }
}
//串口2数据包解包函数
uint8_t choose_cross_flag = 0;
uint16_t teeterboard_vision_error = 0;
void usart2_receive(uint8_t data[])
{
	uint8_t i;

	if(data[0]==0xAA && data[1]==0xBB && data[U2_DATASIZE-1]==0xFF) {
		//红0绿1
		switch (data[2]) {
			case 0x01: {//红灯
				choose_cross_flag = 1;
				//OLED_ShowNum(0,0,choose_cross_flag,3);
			}
			  break;
			case 0x02: {//绿灯
				choose_cross_flag = 2;
				//OLED_ShowNum(0,0,choose_cross_flag,3);
			}
		}
		
	}

	for (i = 0; i < U2_DATASIZE; ++i) {data[i]=0;}
		
}
uint8_t sign[]="\x57\x01";
uint8_t color_flag=1;
//定时器中断读取灰度,间隔10ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim3){
		usart6_send(sign);
	}
	
}
// UART错误回调函数
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2) {
        // 清除错误标志
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        
        // 重新启动接收
        HAL_UART_Receive_IT(&huart1, &rx_ring_buffer.buffer[rx_ring_buffer.head], 1);
    }
}

// 获取数据是否超时的函数（可选）
uint8_t isDataTimeout(void) {
    if(!first_valid_data) {
        return 1; // 还没有收到有效数据
    }
    uint32_t current_time = HAL_GetTick();
    if((current_time - last_valid_time) > MAX_DATA_INTERVAL) {
        return 1; // 数据超时
    }
    
    return 0; // 数据正常
}