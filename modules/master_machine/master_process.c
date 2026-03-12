/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
// #include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "crc.h"
#include "bsp_dwt.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
static USARTInstance *vision_usart_instance;
// void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
// {
//     send_data.enemy_color = enemy_color;
//     send_data.work_mode = work_mode;
//     send_data.bullet_speed = bullet_speed;
// }

// void VisionSetAltitude(float yaw, float pitch, float roll)
// {
//     send_data.yaw = yaw;
//     send_data.pitch = pitch;
//     send_data.roll = roll;
// }

//硬件crc32校验
CRC_STATE check_data4_crc32(uint8_t *pbuffer,uint8_t length_4multi)
{
    // uint32_t *p32;
    // uint32_t crc_cal = 0;
    
    // p32 = (uint32_t*)&pbuffer[0];
    // crc_cal = HAL_CRC_Calculate(&hcrc,p32,(uint32_t)(length_4multi/4-1));

    // p32 = (uint32_t*)&pbuffer[length_4multi-4];
    // if (*p32 == crc_cal)
    // {
    //   return CRC_RIGHT;
    // }
    // else
    // {
    //   return CRC_WRONG;
    // }
}

/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
void get_protocol_send_data(
                            uint8_t *tx_buf,         // 待发送的数据帧
                            Vision_Send_s *tx_data          // 待发送的float数据
                
                            )    // 待发送的数据帧长度
{
    // float *pf;
    // int8_t *pi8;
    // int16_t *pi16;
    // uint32_t *p32;
    // uint32_t crc_val;
    // tx_buf[0] = tx_data->sof;
    // pi8 = (int8_t*)&tx_buf[1];
    // *pi8 = tx_data->fire_times;

    // pf = (float*)&tx_buf[2];
    // *pf = tx_data->present_pitch;

    // pf = (float*)&tx_buf[6];
    // *pf = tx_data->present_yaw;

    // pi16 = (int16_t*)&tx_buf[10];
    // *pi16 = tx_data->reserved_slot;

    // p32 = (uint32_t*)&tx_buf[0];
    // crc_val=HAL_CRC_Calculate(&hcrc,p32,3);

    // p32 = (uint32_t*)&tx_buf[12];
    // *p32 = crc_val;
    // 1. 帧头


    // 2. 模式 Mode [1]
    tx_buf[0] = 0xFF; // 帧头

    // 3. 数据段 (Roll, Pitch, Yaw)
    // 使用 memcpy 防止字节对齐问题，比直接指针转换更安全
    memcpy(&tx_buf[1],  &tx_data->pitch, 4); // [1-4]
    memcpy(&tx_buf[5], &tx_data->yaw,   4); // [5-8]
    memcpy(&tx_buf[9], &tx_data->roll,  4); // [9-12]
    memcpy(&tx_buf[13], &tx_data->chassis_angle,  4); // [13-16]
    memcpy(&tx_buf[17], &tx_data->x, 4); // [17-20]
    // memcpy(&tx_buf[21], &tx_data->outpost_HP,   4); // [21-24]
    memcpy(&tx_buf[21], &tx_data->y,   4); // [21-24]
    // memcpy(&tx_buf[25], &tx_data->vz,   4); // [25-28]
    // memcpy(&tx_buf[29], &tx_data->vz,   4); // [29-32]
    // tx_buf[33] = tx_data->attack_cmd; // [33]
    // tx_buf[34] = tx_data->retreat_cmd; // [34]
    // tx_buf[35] = tx_data->avoid_cmd; // [35]
    // tx_buf[36] = tx_data->game_state; // [36]
    // tx_buf[33] = 0; // [32]
    // tx_buf[34] = 0; // [33]
    // tx_buf[35] = 0; // [34]
    // tx_buf[36] = 0; // [35]
    // 4. 帧尾/校验
    tx_buf[25] = 0; // CRC 占位，暂时填 
    tx_buf[26] = 0; // CRC 占位，暂时填 0 或者固定值，防止数组越界
    tx_buf[27] = 0; // CRC 占位，暂时填 0 或者固定值，防止数组越界
    tx_buf[28] = 0; // CRC 占位，暂时填 0 或者固定值，防止数组越界
    tx_buf[29] = 0; // CRC 占位，暂时填 0 或者固定值，防止数组越界
    uint8_t checksum = 0;
    for (int i = 0; i < 30; i++) {
        checksum ^= tx_buf[i];
    }
    tx_buf[30] = checksum;
    tx_buf[31] = 0x0D; // 帧尾
    // 上位机没明确 CRC 算法，暂时填 0 或者固定值，防止数组越界

}

// void get_protocol_send_chassis_data(
//                             uint8_t *tx_buf,         // 待发送的数据帧
//                             Chassis_Send_s *tx_data          // 待发送的 float 数据
                
//                             )    // 待发送的数据帧长度
// {
//     uint32_t *p32;
//     uint32_t crc_val;
    
//     // 1. 帧头
//     tx_buf[0] = tx_data->sof;
    
//     // 2. 数据段 (vx, vy, wz, power 等)
//     memcpy(&tx_buf[1], &tx_data->vx, 4);           // [1-4]   vx
//     memcpy(&tx_buf[5], &tx_data->vy, 4);           // [5-8]   vy
//     memcpy(&tx_buf[9], &tx_data->wz, 4);           // [9-12]  wz
//     memcpy(&tx_buf[13], &tx_data->chassis_power, 4);    // [13-16] chassis_power
//     memcpy(&tx_buf[17], &tx_data->buffer_energy, 4);    // [17-20] buffer_energy
//     memcpy(&tx_buf[21], &tx_data->shoot_heat, 4);       // [21-24] shoot_heat
//     memcpy(&tx_buf[25], &tx_data->shoot_heat_limit, 4); // [25-28] shoot_heat_limit
//     memcpy(&tx_buf[29], &tx_data->chassis_power_limit, 4); // [29-32] chassis_power_limit
//     memcpy(&tx_buf[33], &tx_data->initial_speed, 4);    // [33-36] initial_speed
//     memcpy(&tx_buf[37], &tx_data->robot_HP, 2);         // [37-38] robot_HP
    
//     // 3. CRC32 校验
//     p32 = (uint32_t*)&tx_buf[0];
//     crc_val = HAL_CRC_Calculate(&hcrc, p32, 10);  // 计算前 40 字节的 CRC
    
//     memcpy(&tx_buf[39], &crc_val, 4);  // [39-42] crc_value
    
//     // 总长度 43 字节
// }

/*
    此函数用于处理接收数据，
    返回数据内容的id
*/
void get_protocol_info(uint8_t *rx_buf, Vision_Recv_s *rx_data)         // 接收的float数据存储地址
{
    // // 1. 严格检查帧头 (0xFF)
    if (rx_buf[0] != 0xFF) {
        return; // 帧头不对，直接丢弃，防止错位解析
    }
    // if (rx_buf[35] != 0x1E) {
    //     return; // 帧尾不对，说明数据可能损坏或长度错位
    // }
    // // 直接解析，无需 CRC32 (除非你确定上位机发了 CRC)
    // rx_data->ACTION_DATA.fire_cmd = rx_buf[0];
    
    // [1-3] 开火,roll,nav 指令
    if (rx_buf[28] == 0x00) { //底盘
        rx_data->ACTION_DATA.roll_cmd = rx_buf[2];
        rx_data->ACTION_DATA.nav_cmd = rx_buf[3];
        memcpy(&rx_data->ACTION_DATA.vx,      &rx_buf[16],  4);
        memcpy(&rx_data->ACTION_DATA.vy,     &rx_buf[20], 4);
        memcpy(&rx_data->ACTION_DATA.wz,     &rx_buf[24], 4);
    }
    else if (rx_buf[28] == 0x01) { //云台
        rx_data->ACTION_DATA.fire_cmd = rx_buf[1];
        memcpy(&rx_data->ACTION_DATA.pitch,    &rx_buf[4],  4);
        memcpy(&rx_data->ACTION_DATA.yaw,      &rx_buf[8],  4);
        memcpy(&rx_data->ACTION_DATA.distance,    &rx_buf[12],  4);
    }
    else
    {
        return; // 未知控制模式，丢弃数据

    }
}



/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"



/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    // uint16_t flag_register;
    DaemonReload(vision_daemon_instance); // 喂狗
    get_protocol_info(vision_usart_instance->recv_buff, &recv_data);
    // TODO: code to resolve flag_register;
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend(Vision_Send_s *tx_data)
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    // static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    // static uint16_t tx_len;
    // TODO: code to set flag_register
    // flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(send_buff, tx_data);
    USARTSend(vision_usart_instance, send_buff, sizeof(Vision_Send_s), USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    //uint16_t flag_register;
    //get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    
    get_protocol_info(vis_recv_buff, &recv_data);  // ← 改為新接口
    // TODO: code to resolve flag_register;
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);
  
    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend(Vision_Send_s *tx_data)  // ← 添加參數
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    
    // 使用新版打包函數
    get_protocol_send_data(send_buff, tx_data);  // ← 只需要兩個參數
    
    // 發送固定長度
    USBTransmit(send_buff, 32);  // ← 固定長度
}

// void ChassisSend(Chassis_Send_s *tx_data)  // ← 添加參數
// {
//     static uint8_t send_buff[43];  // 底盘数据帧长度 43 字节
    
//     // 使用打包函数将数据结构转换为协议帧
//     get_protocol_send_chassis_data(send_buff, tx_data);
    
//     // 通过 USB CDC 发送到底盘
//     USBTransmit(send_buff, 43);  // 发送 43 字节
// }

#endif // VISION_USE_VCP
