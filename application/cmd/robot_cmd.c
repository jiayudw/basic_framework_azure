// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360
#define shoot_frequency 8 //射击频率
/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
// static Video_Trans_Data_t *vtm_data;
static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
// static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
static int16_t curr_dial = 0;  // 拨轮值, 范围 -660 ~ +660

static int pitch_way=0;



void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart1); // 视觉通信串口
    // vtm_data = VideoTransInit(&huart1);
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;
    // vision_recv_data->ACTION_DATA.fire_times = 0;
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.lid_mode = LID_CLOSE;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.bullet_speed = BULLET_SPEED_NONE;
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入

}

//用于转换电机的真实角度
// int16_t map_value(float value, float *ori_scope, float *target_scope) {

//     float from_range = ori_scope[1] - ori_scope[0];
//     float to_range = target_scope[1] - target_scope[0];

//     float scaled_value = (value - ori_scope[0]) / from_range;
//     float result = target_scope[0] + scaled_value * to_range;

//     return result;
// }

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
// static void CalcOffsetAngle()
// {
//     // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
//     static float angle;
//     angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
// #if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
//     if (angle > YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
// #else // 小于180度
//     if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle > 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
// #endif
// }
static void CalcOffsetAngle()
{
    // 获取云台相对于自身的单圈绝对角度 (0-360)
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; 

    // 1. 计算原始角度差
    float angle_diff = angle - YAW_ALIGN_ANGLE;

    // 2. 最短路处理：如果差值超过 180 度，说明反向转动距离更短
    while (angle_diff > 180.0f) {
        angle_diff -= 360.0f;
    }
    while (angle_diff <= -180.0f) {
        angle_diff += 360.0f;
    }

    // 3. 更新输出值
    chassis_cmd_send.offset_angle = angle_diff;
}


// int16_t CalcNowYawDirection (){
//         // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
//     static float angle;
//     angle = gimbal_fetch_data. _single_round_angle; // 从云台获取的当前yaw电机单圈角度
// #if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
//     if (angle > YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
// #else // 小于180度
//     if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle > 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
// #endif
//     return (chassis_cmd_send.offset_angle*10000)/0.0174533;
// }

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */

static void RemoteControlSet()
{
    
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘跟随云台
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    }
    if (switch_is_up(rc_data[TEMP].rc.switch_right))//右侧开关为上为小陀螺模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    // 左侧开关状态为[下],遥控器控制下启动视觉调试
    //后续启动视觉，先到当前位置，如果识别到才瞄准        chassis_cmd_send.vy = 50.0f * (float)rc_data[TEMP].rc.rocker_r_; // 右侧摇杆水平方向控制y方向速度

    if (switch_is_down(rc_data[TEMP].rc.switch_left))
    {   
        chassis_cmd_send.vx = 50.0f * (float)rc_data[TEMP].rc.rocker_r1; // 右侧摇杆竖直方向控制x方向速度
        chassis_cmd_send.vy = -50.0f * (float)rc_data[TEMP].rc.rocker_r_; // 右侧摇杆水平方向控制y方向速度
        gimbal_cmd_send.yaw -= 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
        // chassis_cmd_send.wz = (float)rc_data[TEMP].rc.rocker_l_;

        // gimbal_cmd_send.yaw = gimbal_fetch_data.yaw_motor_single_round_angle;
        gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
        int16_t curr_dial = rc_data[TEMP].rc.dial;  // 拨轮值, 范围 -660 ~ +660

        if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE)
        {
            gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
        }
        if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE)
        {
            gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
        }

    }
    //左中间为视觉控制,直接给定速度和角度增量,不需要遥控器的输入
    else if (switch_is_mid(rc_data[TEMP].rc.switch_left)){
        chassis_cmd_send.vx = vision_recv_data->ACTION_DATA.vx * 14000;
        chassis_cmd_send.vy = vision_recv_data->ACTION_DATA.vy * 18620;
        int16_t curr_dial = rc_data[TEMP].rc.dial;  // 拨轮值, 范围 -660 ~ +660

        // chassis_cmd_send.wz = vision_recv_data->ACTION_DATA.wz * 183.0f * 4.62;
        if(vision_recv_data->ACTION_DATA.distance >0 || vision_recv_data->ACTION_DATA.distance == -1) //大于等于0意味着处于自瞄状态
        {
            curr_dial = -110;
            shoot_cmd_send.friction_mode = FRICTION_ON;
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            
            double send_yaw_;
            if(vision_recv_data->ACTION_DATA.distance > 0){
                // gimbal_cmd_send.yaw = vision_recv_data->ACTION_DATA.yaw;// ACTION_DATA.yaw和gimbal_cmd_send.yaw的坐标系一致，都是上电那一刻确定的坐标系 
                
                
                float d= vision_recv_data->ACTION_DATA.yaw - gimbal_cmd_send.yaw;
                int d_n=(int)(d/360.0);
                float angle_proc = vision_recv_data->ACTION_DATA.yaw - d_n*360; //abs<360
                if (angle_proc-gimbal_cmd_send.yaw>180)
                    angle_proc-=360;
                else if(angle_proc-gimbal_cmd_send.yaw<-180)
                    angle_proc+=360;
                gimbal_cmd_send.yaw=angle_proc;
                
                gimbal_cmd_send.pitch = vision_recv_data->ACTION_DATA.pitch;
            }
            else if (vision_recv_data->ACTION_DATA.distance == -1){
                gimbal_cmd_send.yaw += 0.05; //锁敌yaw//0.144
                gimbal_cmd_send.pitch += 0.05*pitch_way; //0.2
                if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE){
                    gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
                    pitch_way=-1;
                }
                if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE/2){
                    gimbal_cmd_send.pitch = PITCH_MIN_ANGLE/2;
                    pitch_way=1;
                }
                // if (gimbal_cmd_send.yaw > 360){
                //     gimbal_cmd_send.yaw -= 360;
                // }
                // else if (gimbal_cmd_send.yaw <= 0 ){
                //     gimbal_cmd_send.yaw += 360;
                // }
            // double d=send_yaw_-(int)(send_yaw_/360.0)
            // gimbal_cmd_send.yaw=gimbal_cmd_send.yaw+d;
            }
            if (vision_recv_data->ACTION_DATA.fire_cmd == 1 ){
                curr_dial = -600;
            }
            else if(vision_recv_data->ACTION_DATA.fire_cmd == 0 ){
                curr_dial = -110;
            }

            // gimbal_cmd_send.yaw = gimbal_fetch_data.yaw_motor_single_round_angle; // 会抖动的分离模式
            
            // yaw_motor-

            // gimbal_cmd_send.yaw = YAW_ALIGN_ANGLE //wth
        }
        else if(vision_recv_data->ACTION_DATA.distance == -2)//等于-2意味着处于导航状态，此时只响应ACTION_DATA.yaw，不响应ACTION_DATA.wz
        {
            curr_dial = 0;
            gimbal_cmd_send.yaw -= -vision_recv_data->ACTION_DATA.wz*28.5/100;// 原汁原味的分离模式, wz是速度，对速度积分,这个是给跟随云台yaw的参数
            
        }
        if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE)
        {
            gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
        }
        if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE)
        {
            gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
        }
    }

    // if (switch_is_down(rc_data[TEMP].rc.switch_left))
    // {   if( vision_recv_data->ACTION_DATA.distance != -1){
    //     gimbal_cmd_send.yaw = vision_recv_data->ACTION_DATA.yaw;
    //     gimbal_cmd_send.pitch =vision_recv_data->ACTION_DATA.pitch;
    // }else{
    //     chassis_cmd_send.vx = 20.0f * (float)rc_data[TEMP].rc.rocker_r1; // 右侧摇杆竖直方向控制x方向速度
    //     chassis_cmd_send.vy = -20.0f * (float)rc_data[TEMP].rc.rocker_r_; // 右侧摇杆水平方向控制y方向速度
    //     gimbal_cmd_send.yaw -= 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
    //     gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
    //     if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE)
    //     {
    //         gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    //     }
    //     if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE)
    //     {
    //         gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
    //     }
    // }

        // shoot_cmd_send.shoot_num = vision_recv_data->ACTION_DATA.fire_times;
        // if (shoot_cmd_send.shoot_num == 1)
        // {
        //     shoot_cmd_send.load_mode = LOAD_VISION;
        // }else if (shoot_cmd_send.shoot_num == 0)
        // {
        //     shoot_cmd_send.load_mode = LOAD_STOP;
        // }
        
        // if (vision_recv_data->ACTION_DATA.reserved_slot / 10 == 2)
        // {
        //     shoot_cmd_send.load_mode = LOAD_REVERSE;
        //     shoot_cmd_send.shoot_rate = 4;
        //     shoot_cmd_send.shoot_num = 0;
        // }
        // if (vision_recv_data->ACTION_DATA.reserved_slot / 10 == 2)
        // {
        //     shoot_cmd_send.load_mode = LOAD_REVERSE;
        //     shoot_cmd_send.shoot_rate = 4;
        //     shoot_cmd_send.shoot_num = 0;
        // }

        // if (vision_recv_data->ACTION_DATA.reserved_slot % 10 == 2)
        // {
        //     chassis_cmd_send.vy = 0;
        //      chassiuint8_t_cmd_send.vy = 0;
        //     chassis_cmd_send.wz = 0;
        // }else if (vision_recv_data->ACTION_DATA.reserved_slot % 10 == 1)
        // {
        //     chassis_cmd_send.vy = -0;
        //     chassis_cmd_send.wz = 0;
        // }
    // } else {
    //     gimbal_cmd_send.yaw -= 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
    //     gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
    //     if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE)
    //     {
    //         gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    //     }
    //     if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE)
    //     {
    //         gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
    //     }
    //     // 按照摇杆的输出大小进行角度增量,增益系数需调整

    //     // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    //     chassis_cmd_send.vx = 20.0f * (float)rc_data[TEMP].rc.rocker_r1; // 右侧摇杆竖直方向控制x方向速度
    //     chassis_cmd_send.vy = -20.0f * (float)rc_data[TEMP].rc.rocker_r_; // 右侧摇杆水平方向控制y方向速度

    // }
// // ================= 3. 弹速切换逻辑 (拨轮向下) =================
    static uint8_t bullet_speed_index = 1;  // 默认18m/s (index=1)
    static int16_t last_dial = 0;           // 上一次拨轮值
    
    // int16_t curr_dial = rc_data[TEMP].rc.dial;  // 拨轮值, 范围 -660 ~ +660
    
shoot_cmd_send.lid_mode = LID_OPEN;

    // 2. 拨轮发射逻辑：采用互斥结构防止覆盖
    if (curr_dial < -500) { 
        // 【拨轮向上推满】：连发模式
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        
    } 
    else if (curr_dial < -100) { 
        // 【拨轮向上微推】：仅开启摩擦轮预热
        shoot_cmd_send.friction_mode = FRICTION_ON;

        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    else if (curr_dial > 500) {
        // 【拨轮向下推满】：反转模式（用于清弹）
        // shoot_cmd_send.friction_mode = FRICTION_ON;
        // shoot_cmd_send.shoot_mode = SHOOT_ON;
        // shoot_cmd_send.load_mode = LOAD_REVERSE;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.load_mode = LOAD_REVERSE;
    }
    else if (curr_dial > 100) {
        // 【拨轮向下微推】：单发模式
        // shoot_cmd_send.friction_mode = FRICTION_ON;
        // shoot_cmd_send.shoot_mode = SHOOT_ON;
        // shoot_cmd_send.load_mode = LOAD_1_BULLET;
        // 如果需要射击完成标志位控制，可以在这里添加 shoot_num 逻辑
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    else {
        // 【拨轮中位】：停止发射逻辑，关闭摩擦轮
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    
    // 3. 设定固定弹速（代码一风格）
    shoot_cmd_send.bullet_speed = SMALL_AMU_15; 
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{   
//     /**
//  * @brief 输入为键鼠（图传链路）时，底盘、云台、发射机构的控制模式和控制量设置
//  */
//     // ================= 1. 状态机与工作模式切换 (按键状态边沿触发) =================
//     // 按下 R 键：切换底盘的小陀螺模式 / 正常跟随模式
//     // 注意：用 key_count[4] (对应R键的索引) 结合 % 2 实现翻转开关逻辑
//     if (vtm_data->key_count[8] % 2 == 1) 
//     {
//         chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;          // 开启小陀螺
//         gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;        // 云台陀螺仪跟随
//     }
//     else 
//     {
//         chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; // 正常跟随云台
//         gimbal_cmd_send.gimbal_mode   = GIMBAL_GYRO_MODE;
//     }

//     // 按下 F 键：开关摩擦轮 (key_count[9] 是 F 键)
//     if (vtm_data->key_count[9] % 2 == 1) 
//     {
//         shoot_cmd_send.friction_mode = FRICTION_ON;
//     } 
//     else 
//     {
//         shoot_cmd_send.friction_mode = FRICTION_OFF;
//     }


//     // ================= 2. 云台控制 (由鼠标移动控制) =================
//     // 当按下鼠标右键时，启动视觉自瞄（接管云台控制权）
//     if (vtm_data->mouse.press_r == 1) 
//     {
//         if (vision_recv_data->ACTION_DATA.distance != -1) 
//         {
//             // 如果视觉识别到目标，直接给定视觉计算的绝对角度或偏移量
//             gimbal_cmd_send.yaw = vision_recv_data->ACTION_DATA.yaw;
//             gimbal_cmd_send.pitch = vision_recv_data->ACTION_DATA.pitch;
//         }
//     }
//     else 
//     {
//         // 如果没按右键，则使用鼠标移动量控制云台增量
//         // 鼠标的 X 对应 Yaw 轴 (左右)，Y 对应 Pitch 轴 (上下)
//         gimbal_cmd_send.yaw   -= 0.001f * (float)vtm_data->mouse.x; 
//         gimbal_cmd_send.pitch += 0.001f * (float)vtm_data->mouse.y; 

//         // 软件限幅，防止云台抬头或低头卡死
//         if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE)
//             gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
//         if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE)
//             gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
//     }


//     // ================= 3. 发射机构控制 (鼠标左键控制) =================
//     shoot_cmd_send.lid_mode = LID_OPEN; // 默认打开弹舱盖

//     // 定义一个静态变量，用来记录按住左键的时间（调用次数）
//     static uint16_t left_mouse_press_time = 0;
    
//     // 设定阈值：由于 CMD 任务假设是 200Hz（5ms执行一次）
//     // 比如：设定按住超过 300 毫秒 (300 / 5 = 60次) 就算长按连发
//     const uint16_t LONG_PRESS_THRESHOLD = 60; 
//     // ===【Q键反转逻辑：优先级最高，按下Q直接反转清弹】===
//     if (vtm_data->key.q == 1)
//     {
//         // Q键按住期间，强制覆盖所有射击状态为反转
//         left_mouse_press_time = 0; // 同时清零左键计时器，防止松开Q后立刻误触发连发
//         shoot_cmd_send.friction_mode = FRICTION_ON;
//         shoot_cmd_send.shoot_mode    = SHOOT_ON;
//         shoot_cmd_send.load_mode     = LOAD_REVERSE;
//         shoot_cmd_send.shoot_rate    = 4; // 反转速度，可调
//         shoot_cmd_send.bullet_speed  = SMALL_AMU_15;
//     }
//     // ===【F键摩擦轮已开启，才允许正向射击逻辑】===
//     else if (shoot_cmd_send.friction_mode == FRICTION_ON)
//     {
//         if (vtm_data->mouse.press_l == 1)
//         {
//             if (left_mouse_press_time < 60000) {
//                 left_mouse_press_time++;
//             }

//             if (left_mouse_press_time < LONG_PRESS_THRESHOLD)
//             {
//                 // 【短按】单发
//                 shoot_cmd_send.shoot_mode = SHOOT_ON;
//                 shoot_cmd_send.load_mode  = LOAD_1_BULLET;
//                 shoot_cmd_send.bullet_speed = SMALL_AMU_15;
//             }
//             else
//             {
//                 // 【长按】连发
//                 shoot_cmd_send.shoot_mode = SHOOT_ON;
//                 shoot_cmd_send.load_mode  = LOAD_BURSTFIRE;
//                 shoot_cmd_send.shoot_rate = shoot_frequency;
//                 shoot_cmd_send.bullet_speed = SMALL_AMU_15;
//             }
//         }
//         else
//         {
//             // 松开左键，清零并停止
//             left_mouse_press_time = 0;
//             shoot_cmd_send.shoot_mode = SHOOT_OFF;
//             shoot_cmd_send.load_mode  = LOAD_STOP;
//         }
//     }
//     else
//     {
//         // 摩擦轮没开，强制停止
//         left_mouse_press_time = 0;
//         shoot_cmd_send.shoot_mode = SHOOT_OFF;
//         shoot_cmd_send.load_mode  = LOAD_STOP;
//     }
//     // ================= 4. 底盘控制 (由键盘 W, A, S, D 控制) =================
//     float vx_set = 0.0f;
//     float vy_set = 0.0f;
    
//     // 设置底盘基础移动速度
//     float chassis_base_speed = 5000.0f; 

//     // 如果按下 Shift 键，实现加速（冲刺模式）
//     if (vtm_data->key.shift == 1) {
//         chassis_base_speed = 5000.0f; // 冲刺速度
//     }

//     // 前后平移控制 (W / S)
//     if (vtm_data->key.w == 1) 
//         vx_set = chassis_base_speed;
//     else if (vtm_data->key.s == 1) 
//         vx_set = -chassis_base_speed;

//     // 左右平移控制 (A / D)
//     if (vtm_data->key.d == 1) 
//         vy_set = chassis_base_speed;
//     else if (vtm_data->key.a == 1) 
//         vy_set = -chassis_base_speed;

//     // 赋值给底盘发送结构体
//     chassis_cmd_send.vx = vx_set;
//     chassis_cmd_send.vy = -vy_set;
}



/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    if (rc_data[TEMP].lost_flag == 1  || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }

    if (rc_data[TEMP].lost_flag == 0 ) 
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
                LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    EmergencyHandler();
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // if (rc_data[TEMP].lost_flag == 0){}
    RemoteControlSet();
    
    // if(vtm_data->lost_flag == 0){
    // MouseKeySet();
    // }
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    // vision_send_data.sof = 'P';
    // int send_pitch =  (int)(gimbal_fetch_data.gimbal_imu_data.Pitch*DEGREE_2_RAD*10000);
    // int send_yaw =  (int)(gimbal_fetch_data.gimbal_imu_data.Yaw*DEGREE_2_RAD*10000);
    // vision_send_data.present_pitch = (int16_t)(send_pitch>>16);
    // vision_send_data.present_yaw = (int16_t)(send_yaw>>16);
    // vision_send_data.present_debug_value = 0;
    // vision_send_data.null_byte = 0;
    // VisionSend(&vision_send_data);
}