#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
// #include "seasky_protocol.h"

#define VISION_RECV_SIZE 16u // 当前为固定值,36字节
#define VISION_SEND_SIZE 16u
#define ACTION_DATA_LENGTH 16
#define SYN_DATA_LENGTH 16
#define CV_SEND_LENGTH 16
#define PACKET_HEADER 0xFF
#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;


typedef enum
{
    CRC_RIGHT=0,
    CRC_WRONG=1
}CRC_STATE;

// typedef struct
// {
// 	struct 
// 	{
//     	char   sof             ;
//     	int8_t fire_times      ;
//     	float abs_pitch    ;
//     	float abs_yaw      ;
//     	int16_t reserved_slot  ;
//     	uint32_t crc_check     ;
// 	}ACTION_DATA;
// } Vision_Recv_s;
typedef struct
{	struct{
    uint8_t  sof;          // [0] 帧头 (0xA5)
    uint8_t  fire_advice;  // [1] 开火建议 (0:不火, 1:火)
    float    pitch;        // [2-5] 目标Pitch
    float    yaw;          // [6-9] 目标Yaw
    float    distance;     // [10-13] 目标距离 (新增)
    uint8_t  tail[2];      // [14-15] 帧尾/校验占位
}ACTION_DATA;
} Vision_Recv_s;
typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

// typedef struct
// {
//     char sof;
// 	int8_t fire_times;
//     float present_pitch;
//     float present_yaw;
//     int16_t reserved_slot;
//     uint32_t crc_value;
// } 
// Vision_Send_s;
typedef struct
{
    uint8_t  sof;          // [0] 帧头 (0xA5)
    uint8_t  mode;         // [1] 当前模式 (自瞄/大符等)
    float    roll;         // [2-5] 当前 Roll (必须加这个，上位机需要)
    float    pitch;        // [6-9] 当前 Pitch
    float    yaw;          // [10-13] 当前 Yaw
    uint8_t  tail[2];      // [14-15] 帧尾/校验占位
} Vision_Send_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
// void VisionSend();
void VisionSend(Vision_Send_s *tx_data);
// /**
//  * @brief 设置视觉发送标志位
//  *
//  * @param enemy_color
//  * @param work_mode
//  * @param bullet_speed
//  */
// void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

// /**
//  * @brief 设置发送数据的姿态部分
//  *
//  * @param yaw
//  * @param pitch
//  */
// void VisionSetAltitude(float yaw, float pitch, float roll);

extern void get_protocol_send_data(
							uint8_t *tx_buf,			 // 待发送的原始数据	
							Vision_Send_s *tx_data			 // 待发送的数据
							);	 // 待发送的数据帧长度

/*接收数据处理*/
void get_protocol_info(uint8_t *rx_buf,			 // 接收到的原始数据
						   Vision_Recv_s *rx_data);			 // 接收的float数据存储地址
extern Vision_Recv_s recv_data; 
#endif // !MASTER_PROCESS_H