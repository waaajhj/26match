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
volatile PointPacket3 point_packet;
volatile ReceiveState rx_state = WAIT_HEADER;
volatile uint8_t point_packet_ready = 0;
volatile uint32_t point_packet_rx_count = 0;
volatile uint32_t point_packet_error_count = 0;
volatile uint32_t uart4_rx_byte_count = 0;
volatile uint32_t uart4_last_receive_status = HAL_ERROR;
volatile uint32_t uart4_last_error_code = HAL_UART_ERROR_NONE;

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
static uint8_t uart4_rx_byte = 0;
static uint8_t uart4_point_packet_index = 0;
static PointPacket3 uart4_point_packet_buffer;
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
      rx_state = WAIT_HEADER;
      uart4_point_packet_index = 0;
      point_packet_ready = 0;
      uart4_last_receive_status =
          HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);
      if (uart4_last_receive_status != HAL_OK) {
          point_packet_error_count++;
      }
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

        
    }
}

void usart6_receive(uint8_t data[])
{
	uint8_t i;

    /*
     * 新灰度模块返回 16 字节：
     * [0..1] 为帧头 0x77、0x88，[2..13] 依次为 D1~D12，
     * [14..15] 为 CRC16。为尽量少改原代码，本次按说明书允许的方式忽略 CRC。
     */
    if (data[0] == 0x77 && data[1] == 0x88) {
        for (i = 0; i < 12; i++) {
            // 保持原 line[12] 接口：line[0] 对应 D1，line[11] 对应 D12。
            line[i] = (data[i + 2] != 0U) ? 1U : 0U;
        }
    }

    // // 清空本帧缓冲，下一次仍由 USART6 中断接收完整的 16 字节数据帧。
    // for (i = 0; i < U3_DATASIZE; i++) {
    //     data[i] = 0U;
    // }
}
// UART接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // UART4视觉数据接收 =================================================
    if (huart->Instance == UART4)
    {
        uint8_t received_byte = uart4_rx_byte;
        uint8_t *packet_bytes = (uint8_t *)&uart4_point_packet_buffer;
        HAL_StatusTypeDef receive_status;

        uart4_rx_byte_count++;

        /*
         * 先保存本次字节并立刻挂接下一次单字节接收，尽量缩短UART4
         * 未处于接收状态的时间，避免连续数据导致丢字节和包错位。
         */
        receive_status = HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);
        uart4_last_receive_status = receive_status;
        if (receive_status != HAL_OK) {
            point_packet_error_count++;
        }

        /*
         * PACKET_COMPLETE 表示 point_packet 中保存着一个完整有效包。
         * 下一个字节到来时再开始接收新包，期间不进行数据处理。
         */
        if (rx_state == PACKET_COMPLETE) {
            rx_state = WAIT_HEADER;
            uart4_point_packet_index = 0;
        }

        switch (rx_state) {
            case WAIT_HEADER:
                if (received_byte == 0xAA) {
                    packet_bytes[0] = received_byte;
                    uart4_point_packet_index = 1;
                    rx_state = RECEIVE_DATA;
                }
                break;

            case RECEIVE_DATA:
                if (uart4_point_packet_index < sizeof(PointPacket3)) {
                    packet_bytes[uart4_point_packet_index++] = received_byte;
                }

                if (uart4_point_packet_index >= sizeof(PointPacket3)) {
                    if (uart4_point_packet_buffer.header == 0xAA &&
                        uart4_point_packet_buffer.footer == 0x55) {
                        point_packet.header = uart4_point_packet_buffer.header;
                        point_packet.centerpoint_x =
                            uart4_point_packet_buffer.centerpoint_x;
                        point_packet.footer = uart4_point_packet_buffer.footer;
                        point_packet_rx_count++;
                        point_packet_ready = 1;
                        rx_state = PACKET_COMPLETE;
                        uart4_point_packet_index = 0;
                    } else if (received_byte == 0xAA) {
                        point_packet_error_count++;
                        packet_bytes[0] = received_byte;
                        uart4_point_packet_index = 1;
                        rx_state = RECEIVE_DATA;
                    } else {
                        point_packet_error_count++;
                        uart4_point_packet_index = 0;
                        rx_state = WAIT_HEADER;
                    }
                }
                break;

            default:
                uart4_point_packet_index = 0;
                rx_state = WAIT_HEADER;
                break;
        }
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
// 新灰度模块查询命令是单字节 0x61；字符串结尾的 0 仅供 strlen() 计算长度。
uint8_t sign[] = "\x61";
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
    if (huart->Instance == UART4) {
        uart4_last_error_code = huart->ErrorCode;
        point_packet_error_count++;

        /*
         * 溢出错误会终止HAL的中断接收，丢弃当前半包后重新启动
         * 单字节接收。其他非阻塞错误由HAL继续原接收过程。
         */
        if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U) {
            uart4_point_packet_index = 0;
            rx_state = WAIT_HEADER;
            __HAL_UART_CLEAR_OREFLAG(huart);
            uart4_last_receive_status =
                HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);
        }
        return;
    }

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
