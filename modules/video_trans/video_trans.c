#include "video_trans.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_usart.h" 
#include "crc_ref.h"
#include "daemon.h"
#include "bsp_log.h"
// 双缓冲全局静态变量
static Video_Trans_Data_t video_ctrl[2]; // [0]:接收缓存与计算(TEMP), [1]:稳定对外数据(LAST)
static USARTInstance *vtm_usart_instance;
static DaemonInstance *vtm_daemon_instance;
// ROM查表数组：空间换时间，极速解算 CRC16-CCITT-FALSE
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};
static void VTMFeedDog(void) {
    // 调用你系统的喂狗函数，告诉守护进程“我没掉线，重新开始倒计时”
    DaemonReload(vtm_daemon_instance); 
}
static void VTMLostCallback(void *id)
{
    // 1. 断联瞬间，清空所有的遥控摇杆和按键数据，防止机器人疯跑！
    memset(video_ctrl, 0, sizeof(video_ctrl)); 
    
    // 2. 将标志位强制置为 1，告知整个系统图传死了
    video_ctrl[TEMP].lost_flag = 1;         
    video_ctrl[LAST].lost_flag = 1;
    
    // 3. 尝试重启串口接收，万一等下图传又连上了呢
    USARTServiceInit(vtm_usart_instance); 
    
    LOGWARNING("[vtm] video transmission lost!");
}

// 极速版CRC计算函数
static uint16_t FastCRC16(const uint8_t *data, uint32_t length) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < length; i++) {
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

// 摇杆与拨盘安全限幅
static void RectifyVideoJoystick() {
    if (abs(video_ctrl[TEMP].rc.ch0) > 660) video_ctrl[TEMP].rc.ch0 = 0;
    if (abs(video_ctrl[TEMP].rc.ch1) > 660) video_ctrl[TEMP].rc.ch1 = 0;
    if (abs(video_ctrl[TEMP].rc.ch2) > 660) video_ctrl[TEMP].rc.ch2 = 0;
    if (abs(video_ctrl[TEMP].rc.ch3) > 660) video_ctrl[TEMP].rc.ch3 = 0;
    if (abs(video_ctrl[TEMP].rc.dial) > 660) video_ctrl[TEMP].rc.dial = 0;
}

/**
 * @brief 这个函数会被挂载在 BSP_USART 的回调里，收到21字节后触发
 */
static void DecodeVideoTrans(void) {
    uint8_t *rx_buf = vtm_usart_instance->recv_buff;

    // 1. 检验帧头 (说明书固定值 0xA9 和 0x53)
    if (rx_buf[0] != 0xA9 || rx_buf[1] != 0x53) return;

    // 2. 使用官方函数进行 CRC 校验
    // 传入整个缓冲区的指针，以及图传说明书规定的总长度 21
    if (Verify_CRC16_Check_Sum(rx_buf, VTM_RECV_SIZE) == 0) { 
        return; // 如果校验返回 FALSE (0)，说明数据错乱，直接丢弃
    }

    // ============== 开始安全解包至 TEMP 中转层 ==============
    
    // 摇杆与拨轮解析（并减去1024偏置）
    video_ctrl[TEMP].rc.ch0 = (((rx_buf[2] | (rx_buf[3] << 8)) & 0x07FF)) - VIDEO_CH_VALUE_OFFSET;
    video_ctrl[TEMP].rc.ch1 = ((((rx_buf[3] >> 3) | (rx_buf[4] << 5)) & 0x07FF)) - VIDEO_CH_VALUE_OFFSET;
    video_ctrl[TEMP].rc.ch2 = ((((rx_buf[4] >> 6) | (rx_buf[5] << 2) | (rx_buf[6] << 10)) & 0x07FF)) - VIDEO_CH_VALUE_OFFSET;
    video_ctrl[TEMP].rc.ch3 = ((((rx_buf[6] >> 1) | (rx_buf[7] << 7)) & 0x07FF)) - VIDEO_CH_VALUE_OFFSET;
    video_ctrl[TEMP].rc.dial = ((((rx_buf[8] >> 1) | (rx_buf[9] << 7)) & 0x07FF)) - VIDEO_CH_VALUE_OFFSET;

    RectifyVideoJoystick();

    // 手柄按键及杂项解析
    video_ctrl[TEMP].rc.mode_switch = (rx_buf[7] >> 4) & 0x03; 
    video_ctrl[TEMP].rc.pause_btn   = (rx_buf[7] >> 6) & 0x01; 
    video_ctrl[TEMP].rc.custom_l    = (rx_buf[7] >> 7) & 0x01; 
    video_ctrl[TEMP].rc.custom_r    = (rx_buf[8]) & 0x01;      
    video_ctrl[TEMP].rc.trigger     = (rx_buf[9] >> 4) & 0x01; 

    // 鼠标位解析
    video_ctrl[TEMP].mouse.x = (int16_t)(rx_buf[10] | (rx_buf[11] << 8));
    video_ctrl[TEMP].mouse.y = (int16_t)(rx_buf[12] | (rx_buf[13] << 8));
    video_ctrl[TEMP].mouse.z = (int16_t)(rx_buf[14] | (rx_buf[15] << 8));
    video_ctrl[TEMP].mouse.press_l = (rx_buf[16]) & 0x03;
    video_ctrl[TEMP].mouse.press_r = (rx_buf[16] >> 2) & 0x03;
    video_ctrl[TEMP].mouse.press_m = (rx_buf[16] >> 4) & 0x03;

    // ====== 极简无冲突按键解析 ======
    // 直接取出 16 bit 键值赋值给 TEMP 状态
    video_ctrl[TEMP].key.keys = (uint16_t)(rx_buf[17] | (rx_buf[18] << 8));

    uint16_t key_now  = video_ctrl[TEMP].key.keys;                   
    uint16_t key_last = video_ctrl[LAST].key.keys;                       

    // 遍历16位，纯净计算每个键的上升沿（按一次算一次）
    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++) {
        if ((key_now & j) && !(key_last & j)) {
            video_ctrl[TEMP].key_count[i]++;
        }
    }

    video_ctrl[TEMP].lost_flag = 0; 
    
    // ============== 数据验证无误，整体拷贝至 LAST 供外部使用 ==============
    memcpy(&video_ctrl[LAST], &video_ctrl[TEMP], sizeof(Video_Trans_Data_t)); 
    VTMFeedDog(); 
}

/**
 * @brief 注册并开启图传接收
 */
Video_Trans_Data_t* VideoTransInit(UART_HandleTypeDef *huart) {
    memset(video_ctrl, 0, sizeof(video_ctrl));
    
    USART_Init_Config_s conf;
    conf.usart_handle = huart;               
    conf.recv_buff_size = VTM_RECV_SIZE;     
    conf.module_callback = DecodeVideoTrans; 
    
    vtm_usart_instance = USARTRegister(&conf);
        // 2. ======= 新增：注册图传的守护进程 =======
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10,  // 超时阈值，比如100ms没收到数据就认为断联
        .callback = VTMLostCallback, // 绑定上面的掉线回调函数
        .owner_id = NULL, 
    };
    vtm_daemon_instance = DaemonRegister(&daemon_conf);
    // 返回 LAST 稳定层数据指针，底层任务读取这个指针即可
    return &video_ctrl[LAST];
}
