/**
 * @file referee_protocol.h
 * @author kidneygood (you@domain.com) / Updated for RM2026 V1.9.0
 * @version 0.3
 * @date 2026-03-10
 *
 * @copyright Copyright (c) HNU YueLu EC 2022-2026 all rights reserved
 *
 */

#ifndef referee_protocol_H
#define referee_protocol_H

#include "stdint.h"

/****************************宏定义部分****************************/

#define REFEREE_SOF 0xA5 // 起始字节,协议固定为0xA5
#define Robot_Red 0
#define Robot_Blue 1
// #define Communicate_Data_LEN 5 // 自定义交互数据长度（交互数据已注释，暂不使用）

#pragma pack(1)

/****************************通信协议格式****************************/

/* 通信协议格式偏移 */
typedef enum
{
    FRAME_HEADER_Offset = 0,
    CMD_ID_Offset = 5,
    DATA_Offset = 7,
} JudgeFrameOffset_e;

/* 通信协议长度 */
typedef enum
{
    LEN_HEADER = 5, // 帧头长
    LEN_CMDID = 2,	// 命令码长度
    LEN_TAIL = 2,	// 帧尾CRC16

    LEN_CRC8 = 4,   // 帧头CRC8校验长度=帧头+数据长+包序号
} JudgeFrameLength_e;

/****************************帧头****************************/

/* 帧头偏移 */
typedef enum
{
    SOF = 0,		 // 起始位
    DATA_LENGTH = 1, // 帧内数据长度
    SEQ = 3,		 // 包序号
    CRC8 = 4		 // CRC8
} FrameHeaderOffset_e;

/* 帧头定义 */
typedef struct
{
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t CRC8;
} xFrameHeader;

/****************************cmd_id命令码说明****************************/

/* 命令码ID,用来判断接收的是什么数据 */
typedef enum
{
    ID_game_state = 0x0001,				   // 比赛状态数据
    ID_game_result = 0x0002,			   // 比赛结果数据
    ID_game_robot_HP = 0x0003,	           // 比赛机器人血量数据
    ID_event_data = 0x0101,				   // 场地事件数据
    ID_referee_warning = 0x0104,           // 裁判系统警告数据
    ID_dart_info = 0x0105,				   // 飞镖发射相关数据
    ID_game_robot_state = 0x0201,		   // 机器人性能体系数据
    ID_power_heat_data = 0x0202,		   // 实时底盘缓冲能量和射击热量数据
    ID_game_robot_pos = 0x0203,			   // 机器人位置数据
    ID_buff_musk = 0x0204,				   // 机器人增益和底盘能量数据
    ID_robot_hurt = 0x0206,				   // 伤害状态数据
    ID_shoot_data = 0x0207,				   // 实时射击数据
    ID_shoot_num = 0x0208,				   // 允许发弹量数据
    ID_rfid_status = 0x0209,			   // 机器人RFID模块状态
    ID_dart_client_cmd = 0x020A,           // 飞镖选手端指令数据
    ID_ground_robot_pos = 0x020B,          // 地面机器人位置数据
    ID_radar_mark_data = 0x020C,           // 雷达标记进度数据
    ID_sentry_info = 0x020D,               // 哨兵自主决策信息同步
    ID_radar_info = 0x020E,                // 雷达自主决策信息同步
    
    // 以下为交互类数据
    // ID_sentry_dee_data = 0x0301,		   // 机器人交互数据
    // ID_custom_robot_data = 0x0302,		   // 自定义控制器与机器人交互数据
    // ID_map_command = 0x0303,               // 选手端小地图交互数据
    // ID_custom_client_data = 0x0306,		   // 自定义控制器与选手端交互数据
    // ID_robot_custom_data = 0x0309,		   // 自定义控制器接收机器人数据
    // ID_robot_custom_data_2 = 0x0310,	   // 机器人发送给自定义客户端的数据
    // ID_robot_custom_data_3 = 0x0311,	   // 自定义客户端发送给机器人的自定义指令
} CmdID_e;

/* 命令码数据段长 */
typedef enum
{
    LEN_game_state = 11,				   // 0x0001
    LEN_game_result = 1,				   // 0x0002
    LEN_game_robot_HP = 32,				   // 0x0003
    LEN_event_data = 4,					   // 0x0101
    LEN_referee_warning = 3,               // 0x0104
    LEN_dart_info = 3,					   // 0x0105
    LEN_game_robot_state = 13,			   // 0x0201
    LEN_power_heat_data = 16,			   // 0x0202
    LEN_game_robot_pos = 16,			   // 0x0203
    LEN_buff_musk = 7,					   // 0x0204 
    LEN_robot_hurt = 1,					   // 0x0206
    LEN_shoot_data = 7,					   // 0x0207
    LEN_shoot_num = 8,					   // 0x0208 (协议加入了堡垒增益，长8字节)
    LEN_rfid_status = 4,				   // 0x0209
    LEN_dart_client_cmd = 6,               // 0x020A
    LEN_ground_robot_pos = 40,             // 0x020B
    LEN_radar_mark_data = 1,               // 0x020C
    LEN_sentry_info = 6,                   // 0x020D
    LEN_radar_info = 1,                    // 0x020E
    
    // LEN_sentry_dee_data = 127,			   // 0x0301
    // LEN_custom_robot_data = 30,			   // 0x0302
    // LEN_map_command = 15,                  // 0x0303
    // LEN_custom_client_data = 8,			   // 0x0306
    // LEN_robot_custom_data = 30,			   // 0x0309
    // LEN_robot_custom_data_2 = 300,		   // 0x0310
    // LEN_robot_custom_data_3 = 30,		   // 0x0311
} JudgeDataLength_e;

/****************************接收数据的详细说明****************************/

/* ID: 0x0001  Byte:  11    比赛状态数据 */
typedef struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef struct
{
    uint8_t winner;
} game_result_t;

/* ID: 0x0003  Byte:  32    比赛机器人血量数据 */
typedef struct
{
    uint16_t red_1_robot_HP; 
    uint16_t red_2_robot_HP; 
    uint16_t red_3_robot_HP; 
    uint16_t red_4_robot_HP; 
    uint16_t reserved1; 
    uint16_t red_7_robot_HP; 
    uint16_t red_outpost_HP; 
    uint16_t red_base_HP; 
    uint16_t blue_1_robot_HP; 
    uint16_t blue_2_robot_HP; 
    uint16_t blue_3_robot_HP; 
    uint16_t blue_4_robot_HP; 
    uint16_t reserved2; 
    uint16_t blue_7_robot_HP; 
    uint16_t blue_outpost_HP; 
    uint16_t blue_base_HP;
} game_robot_HP_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef struct
{
    uint32_t event_data;
} event_data_t;

/* ID：0x0104  Byte:  3     裁判警告数据*/
typedef struct 
{ 
    uint8_t level; 
    uint8_t offending_robot_id; 
    uint8_t count; 
} referee_warning_t;

/* ID: 0x0105  Byte:  3    飞镖发射数据*/
typedef struct 
{ 
    uint8_t dart_remaining_time; 
    uint16_t dart_info; 
} dart_info_t;

/* ID: 0X0201  Byte: 13    机器人性能体系数据 */
typedef struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP; 
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1; 
    uint8_t power_management_shooter_output : 1;
} robot_status_t;

/* ID: 0X0202  Byte: 16    实时底盘缓冲能量和射击热量数据 */
typedef struct 
{ 
    uint16_t reserved1; 
    uint16_t reserved2; 
    float reserved3; 
    uint16_t buffer_energy; 
    uint16_t shooter_17mm_1_barrel_heat; 
    uint16_t shooter_17mm_2_barrel_heat; 
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef struct
{
    float x;
    float y;
    float angle;
} robot_pos_t;

/* ID: 0x0204  Byte:  7    机器人增益和底盘能量数据 */
typedef struct
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
} buff_t;

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef struct
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} hurt_data_t;

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef  struct
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} shoot_data_t;

/* ID: 0x0208  Byte:  8    允许发弹量数据 */
typedef struct
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} projectile_allowance_t;

/* ID: 0x0209  Byte:  4    RFID模块状态数据 */
typedef struct
{
    uint32_t rfid_status;
} rfid_status_t;

/* ID: 0x020A  Byte:  6    飞镖选手端指令数据 */
typedef struct
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;

/* ID: 0x020B  Byte: 40    地面机器人位置数据 */
typedef struct
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved1;
    float reserved2;
} ground_robot_position_t;

/* ID: 0x020C  Byte:  1    雷达标记进度数据 */
typedef struct
{
    uint8_t mark_progress;
} radar_mark_data_t;

/* ID: 0x020D  Byte:  6    哨兵自主决策信息同步 */
typedef struct
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
} sentry_info_t;

/* ID: 0x020E  Byte:  1    雷达自主决策信息同步 */
typedef struct
{
    uint8_t radar_info;
} radar_info_t;


/* ===========================================================================
 * UI交互数据定义（referee_UI 模块使用）
 * =========================================================================== */

/////////////////////////////机器人交互数据/////////////////////////////

// 交互数据头结构
typedef struct
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

// 机器人id
typedef enum
{
    // 红方机器人ID
    RobotID_RHero = 1,
    RobotID_REngineer = 2,
    RobotID_RStandard1 = 3,
    RobotID_RStandard2 = 4,
    RobotID_RStandard3 = 5,
    RobotID_RAerial = 6,
    RobotID_RSentry = 7,
    RobotID_RRadar = 9,
    // 蓝方机器人ID
    RobotID_BHero = 101,
    RobotID_BEngineer = 102,
    RobotID_BStandard1 = 103,
    RobotID_BStandard2 = 104,
    RobotID_BStandard3 = 105,
    RobotID_BAerial = 106,
    RobotID_BSentry = 107,
    RobotID_BRadar = 109,
} Robot_ID_e;

// 交互数据ID
typedef enum
{
    UI_Data_ID_Del      = 0x0100,
    UI_Data_ID_Draw1    = 0x0101,
    UI_Data_ID_Draw2    = 0x0102,
    UI_Data_ID_Draw5    = 0x0103,
    UI_Data_ID_Draw7    = 0x0104,
    UI_Data_ID_DrawChar = 0x0110,
    Communicate_Data_ID = 0x0200,
} Interactive_Data_ID_e;

// 交互数据长度
typedef enum
{
    Interactive_Data_LEN_Head  = 6,
    UI_Operate_LEN_Del         = 2,
    UI_Operate_LEN_PerDraw     = 15,
    UI_Operate_LEN_DrawChar    = 15 + 30,
} Interactive_Data_Length_e;

/////////////////////////////UI交互数据/////////////////////////////

// 图形数据
typedef struct
{
    uint8_t figure_name[3];
    uint32_t operate_tpye : 3;
    uint32_t figure_tpye  : 3;
    uint32_t layer        : 4;
    uint32_t color        : 4;
    uint32_t details_a    : 9;
    uint32_t details_b    : 9;
    uint32_t width        : 10;
    uint32_t start_x      : 11;
    uint32_t start_y      : 11;
    uint32_t details_c    : 10;
    uint32_t details_d    : 11;
    uint32_t details_e    : 11;
} Graph_Data_t;

// 打印字符串数据
typedef struct
{
    Graph_Data_t grapic_data_struct;
    uint8_t data[30];
} String_Data_t;

typedef enum { UI_Data_Del_NoOperate=0, UI_Data_Del_Layer=1, UI_Data_Del_ALL=2 } UI_Delete_Operate_e;
typedef enum { UI_Graph_ADD=1, UI_Graph_Change=2, UI_Graph_Del=3 } UI_Graph_Operate_e;
typedef enum { UI_Graph_Line=0, UI_Graph_Rectangle=1, UI_Graph_Circle=2,
               UI_Graph_Ellipse=3, UI_Graph_Arc=4, UI_Graph_Float=5,
               UI_Graph_Int=6, UI_Graph_Char=7 } UI_Graph_Type_e;
typedef enum { UI_Color_Main=0, UI_Color_Yellow=1, UI_Color_Green=2,
               UI_Color_Orange=3, UI_Color_Purplish_red=4, UI_Color_Pink=5,
               UI_Color_Cyan=6, UI_Color_Black=7, UI_Color_White=8 } UI_Graph_Color_e;

/* ===========================================================================
 * 以下机器人间通信数据暂不使用，已注释
 * ===========================================================================

// 学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF，上限 30Hz
// #define Communicate_Data_LEN 5
// typedef struct { uint8_t data[Communicate_Data_LEN]; } robot_interactive_data_t;
// typedef struct { xFrameHeader FrameHeader; uint16_t CmdID;
//     ext_student_interactive_header_data_t datahead;
//     robot_interactive_data_t Data; uint16_t frametail; } Communicate_SendData_t;
// typedef struct { ext_student_interactive_header_data_t datahead;
//     robot_interactive_data_t Data; } Communicate_ReceiveData_t;

 * =========================================================================== */

#pragma pack()

#endif