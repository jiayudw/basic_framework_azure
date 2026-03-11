#ifndef VIDEO_TRANS_H
#define VIDEO_TRANS_H

#include <stdint.h>
#include "usart.h" // 包含STM32的串口头文件，如果你工程里叫别的请自行修改

#define VTM_RECV_SIZE 21u
#define VIDEO_CH_VALUE_OFFSET 1024 // 减去中位值，让摇杆变为 -660~+660

// 缓冲数组索引
#define TEMP 0
#define LAST 1

// 极简版键盘位域结构体（所有按键地位平等）
typedef union {
    uint16_t keys;
    struct {
        uint16_t w:1;       // bit 0
        uint16_t s:1;       // bit 1
        uint16_t a:1;       // bit 2
        uint16_t d:1;       // bit 3
        uint16_t shift:1;   // bit 4 
        uint16_t ctrl:1;    // bit 5 
        uint16_t q:1;       // bit 6
        uint16_t e:1;       // bit 7
        uint16_t r:1;       // bit 8
        uint16_t f:1;       // bit 9
        uint16_t g:1;       // bit 10
        uint16_t z:1;       // bit 11
        uint16_t x:1;       // bit 12
        uint16_t c:1;       // bit 13
        uint16_t v:1;       // bit 14
        uint16_t b:1;       // bit 15
    };
}  VTM_Key_t;;

// 图传数据总结构体
typedef struct {
    struct {
        int16_t ch0;            // 右摇杆左右
        int16_t ch1;            // 右摇杆上下
        int16_t ch2;            // 左摇杆上下
        int16_t ch3;            // 左摇杆左右
        uint8_t mode_switch;    // 模式开关 (C:0, N:1, S:2)
        uint8_t pause_btn;      // 暂停按键 (0或1)
        uint8_t custom_l;       // 自定义按键左
        uint8_t custom_r;       // 自定义按键右
        int16_t dial;           // 拨轮 (-660 ~ 660)
        uint8_t trigger;        // 扳机
    } rc;

    struct {
        int16_t x;              // 鼠标X轴
        int16_t y;              // 鼠标Y轴
        int16_t z;              // 鼠标滚轮
        uint8_t press_l;        // 鼠标左键
        uint8_t press_r;        // 鼠标右键
        uint8_t press_m;        // 鼠标中键
    } mouse;

    VTM_Key_t key;                  // 当前所有16个按键的直观状态(持续按压)
    uint16_t key_count[16];     // 16个按键各自独立的上升沿触发计数(按一次加一)

    uint8_t lost_flag;          // 图传掉线标志位
} Video_Trans_Data_t;

// 对外提供的初始化和获取指针接口
Video_Trans_Data_t* VideoTransInit(UART_HandleTypeDef *huart);

#endif
