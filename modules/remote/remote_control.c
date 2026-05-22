#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

#define REMOTE_CONTROL_FRAME_SIZE 25u // 遥控器接收的buffer大小

uint16_t CH[16];

// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    uint16_t ch_val;
    // 通道 0
    ch_val = (sbus_buf[1] >> 0 | sbus_buf[2] << 8) & 0x07FF;
    CH[0] = ch_val;

    // 通道 1
    ch_val = (sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF;
    CH[1] = ch_val;

    // 通道 2
    ch_val = (sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF;
    CH[2] = ch_val;

    // 通道 3
    ch_val = (sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF;
    CH[3] = ch_val;

    // 通道 4
    ch_val = (sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF;
    CH[4] = ch_val;

    // 通道 5
    ch_val = (sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF;
    CH[5] = ch_val;

    // 通道 6
    ch_val = (sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF;
    CH[6] = ch_val;

    // 通道 7
    ch_val = (sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF;
    CH[7] = ch_val;

    // 通道 8
    ch_val = (sbus_buf[12] >> 0 | sbus_buf[13] << 8) & 0x07FF;
    CH[8] = ch_val;

    // 通道 9
    ch_val = (sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF;
    CH[9] = ch_val;

    // 通道 10
    ch_val = (sbus_buf[14] >> 6 | sbus_buf[15] << 2 | sbus_buf[16] << 10) & 0x07FF;
    CH[10] = ch_val;

    // 通道 11
    ch_val = (sbus_buf[16] >> 1 | sbus_buf[17] << 7) & 0x07FF;
    CH[11] = ch_val;

    // 通道 12
    ch_val = (sbus_buf[17] >> 4 | sbus_buf[18] << 4) & 0x07FF;
    CH[12] = ch_val;

    // 通道 13
    ch_val = (sbus_buf[18] >> 7 | sbus_buf[19] << 1 | sbus_buf[20] << 9) & 0x07FF;
    CH[13] = ch_val;

    // 通道 14
    ch_val = (sbus_buf[20] >> 2 | sbus_buf[21] << 6) & 0x07FF;
    CH[14] = ch_val;

    // 通道 15
    ch_val = (sbus_buf[21] >> 5 | sbus_buf[22] << 3) & 0x07FF;
    CH[15] = ch_val;
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance);         // 先喂狗
    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    rc_ctrl[TEMP].lost_flag = 1;         // 遥控器离线标志位
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}