#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
// #include "seasky_protocol.h"

#define VISION_RECV_SIZE 32u // 当前为固定值,36字节
#define VISION_SEND_SIZE 32u
#define ACTION_DATA_LENGTH 16
#define SYN_DATA_LENGTH 16
#define CV_SEND_LENGTH 16

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

typedef struct
{
	struct{
		uint8_t fire_cmd;         //[1]
		uint8_t roll_cmd;         //[2]
		uint8_t nav_cmd;          //[3]
		float pitch;     //[4-7]
		float yaw;	     //[8-11]
		float distance;	 //[12-15]
		float vx ;	     //[16-19]
		float vy ;	      //[20-23]		
		float wz ;	      //[24-27]
		uint16_t control_mode; //[28-29] 0x0001为控制模式,0x0002为导航模式,0x0003为陀螺模式
	}ACTION_DATA;
}Vision_Recv_s;

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
//     uint8_t enemy_color;
// 	float pitch;
// 	float yaw;
// 	float robot_HP;
// 	float last_time;
// 	float outpost_HP;
// 	float vx;
// 	float vy;
// 	float vz;
// 	uint8_t attack_cmd;	
// 	uint8_t retreat_cmd;
// 	uint8_t avoid_cmd;
// 	uint8_t game_state;
// } 
// Vision_Send_s;
typedef struct
{
	float pitch;
	float yaw;
	float roll;
	float chassis_angle;
	float x;
	float y;
	uint8_t enemy_color; // 颜色
	// uint8_t retreat_cmd;
	// uint8_t avoid_cmd;
	// uint8_t game_state;
} 
Vision_Send_s;
// typedef struct
// {
// 	float pitch;
// 	float yaw;
// 	float roll;
// 	float robot_HP;
// 	float last_time;
// 	float vx;
// 	float vy;
// 	float vz;
// 	// uint8_t retreat_cmd;
// 	// uint8_t avoid_cmd;
// 	// uint8_t game_state;
// } 
// Vision_Send_s;


// typedef struct
// {
// 	char sof;
// 	float vx;
// 	float vy;
// 	uint32_t crc_value;
// } 
// Chassis_Send_s;
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
void VisionSend();
// void ChassisSend();

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

// extern void get_protocol_send_chassis_data(
// 							uint8_t *tx_buf,			 // 待发送的 chassis 原始数据	
// 							Chassis_Send_s *tx_data			 // 待发送的数据
// 							);	 // 待发送的数据帧长度

/*接收数据处理*/
void get_protocol_info(uint8_t *rx_buf,			 // 接收到的原始数据
						   Vision_Recv_s *rx_data);			 // 接收的 float 数据存储地址

// 新增：底盘数据发送接口
// void ChassisSend(Chassis_Send_s *tx_data);
// extern Vision_Recv_s recv_data
#endif // !MASTER_PROCESS_H