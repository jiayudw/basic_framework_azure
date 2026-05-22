#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"
#include <stdint.h>

#define REMOTE_CONTROL_FRAME_SIZE 25u // 遥控器接收的buffer大小

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
    // 右水平摇杆 -393 ~ +382
    rc_ctrl[0].rc.rocker_r_ = ((sbus_buf[1] >> 0 | sbus_buf[2] << 8) & 0x07FF) - 1033;

    // 右垂直摇杆 -388 ~ +392
    rc_ctrl[0].rc.rocker_r1 = ((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF) - 1020;

    // 左边水平摇杆 -388 ~ +379
    rc_ctrl[0].rc.rocker_l_ = ((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF) - 1020;

    // 左边垂直摇杆 -780 ~ +787
    rc_ctrl[0].rc.rocker_l1 = ((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF) - 1028;

    // VRA -784 ~ +783 顺时针增加
    rc_ctrl[0].rc.dial = ((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF) - 1024;

    uint16_t switch_ch;

    // SWA 下:0 上:1
    switch_ch = (sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF;
    rc_ctrl[0].rc.switch_left = switch_ch==240? 1 : 0;

    // SWB 下:0 上:1
    switch_ch = (sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF;
    rc_ctrl[0].rc.switch_middle_left = switch_ch==240? 1 : 0;

    // SWC 下:0 中:1 上:2
    switch_ch = (sbus_buf[12] >> 0 | sbus_buf[13] << 8) & 0x07FF;
    rc_ctrl[0].rc.switch_middle_right = switch_ch==240? 2 : switch_ch==1023? 1 : 0; 

    // SWD 下:0 上:1
    switch_ch = (sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF;
    rc_ctrl[0].rc.switch_right = switch_ch==240? 1 : 0;

    rc_ctrl[TEMP].lost_flag = 0; // 收到数据,清除掉线标志
    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据.待用
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