#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_log.h"
#include "bmi088.h"
#include "master_process.h"
#include "cmsis_os.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
DJIMotorInstance *yaw_motor, *pitch_motor; // wth
static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;     // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static Subscriber_t *chassis_sub;     // 底盘裁判系统数据订阅者
static Chassis_Upload_Data_s chassis_refe_data; // 底盘裁判系统数据
static Vision_Send_s vision_send_data; // 云台视觉数据 

static float last_time,time_delta; // 计时变量
// 专门用于 Ozone 调试的全局变量
volatile float debug_speed_target = 0.0f;
// YAW轴调试参数
volatile float debug_yaw_angle_Kp = 30.0f;
volatile float debug_yaw_angle_Ki = 0.2f;
volatile float debug_yaw_angle_Kd = 0.5f;

volatile float debug_yaw_speed_Kp = 50.0f;
volatile float debug_yaw_speed_Ki = 200.0f;
volatile float debug_yaw_speed_Kd = 0.0f;

// PITCH轴调试参数
volatile float debug_pitch_angle_Kp = 30.0f;
volatile float debug_pitch_angle_Ki = 0.1f;
volatile float debug_pitch_angle_Kd = 0.8f;

volatile float debug_pitch_speed_Kp = 50.0f;
volatile float debug_pitch_speed_Ki = 350.0f;
volatile float debug_pitch_speed_Kd = 0.0f;
static float x,y;
void GimbalInit()
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 5,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 20, // 8
                .Ki = 0.1,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 200, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 20, // 10
                .Ki = 0,
                .Kd = 0.5,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 350, // 350
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    chassis_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}
// void gimbal_absolute_angle_c(){
//     static float yaw_motor_encoder_angle = 0.0f;
//     // 云台相对角度 = 云台绝对角度 (IMU) - 底盘角度 (电机编码器)
//     static float gimbal_relative_to_chassis = 0.0f;
//     if (yaw_motor != NULL) {
//         // 从电机控制器中读取编码器反馈角度
//         yaw_motor_encoder_angle = yaw_motor->motor_controller.angle_feedback;
//     }
//     // 计算云台相对于底盘的旋转角度
//     gimbal_relative_to_chassis = gimba_IMU_data->YawTotalAngle - yaw_motor_encoder_angle;
    
//     // 角度归一化到 [-180, 180]
//     while (gimbal_relative_to_chassis > 180.0f) {
//         gimbal_relative_to_chassis -= 360.0f;
//     }
//     while (gimbal_relative_to_chassis < -180.0f) {
//         gimbal_relative_to_chassis += 360.0f;
//     }
    
//     // 将相对角度添加到反馈数据中
//     gimbal_feedback_data.gimbal_relative_angle = gimbal_relative_to_chassis;
//     gimbal_feedback_data.yaw_motor_encoder_angle = yaw_motor_encoder_angle;
// }

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{// ============================================
    // 【新增】Ozone 实时调参覆盖逻辑
    // ============================================
    if (yaw_motor != NULL) {
        // YAW 角度环
        yaw_motor->motor_controller.angle_PID.Kp = debug_yaw_angle_Kp;
        yaw_motor->motor_controller.angle_PID.Ki = debug_yaw_angle_Ki;
        yaw_motor->motor_controller.angle_PID.Kd = debug_yaw_angle_Kd;
        // YAW 速度环
        yaw_motor->motor_controller.speed_PID.Kp = debug_yaw_speed_Kp;
        yaw_motor->motor_controller.speed_PID.Ki = debug_yaw_speed_Ki;
        yaw_motor->motor_controller.speed_PID.Kd = debug_yaw_speed_Kd;
    }

    if (pitch_motor != NULL) {
        // PITCH 角度环
        pitch_motor->motor_controller.angle_PID.Kp = debug_pitch_angle_Kp;
        pitch_motor->motor_controller.angle_PID.Ki = debug_pitch_angle_Ki;
        pitch_motor->motor_controller.angle_PID.Kd = debug_pitch_angle_Kd;
        // PITCH 速度环
        pitch_motor->motor_controller.speed_PID.Kp = debug_pitch_speed_Kp;
        pitch_motor->motor_controller.speed_PID.Ki = debug_pitch_speed_Ki;
        pitch_motor->motor_controller.speed_PID.Kd = debug_pitch_speed_Kd;
    }
    // ============================================
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    SubGetMessage(chassis_sub, &chassis_refe_data);
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);  
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    static float cos_yaw,sin_yaw;
    float time_delta = DWT_GetTimeline_ms() - last_time;
    last_time = DWT_GetTimeline_ms();

    cos_yaw = arm_cos_f32(gimbal_feedback_data.gimbal_imu_data.Yaw * DEGREE_2_RAD);
    sin_yaw = arm_sin_f32(gimbal_feedback_data.gimbal_imu_data.Yaw * DEGREE_2_RAD);
    float chassis_send_vx = cos_yaw * chassis_refe_data.speed_vx + sin_yaw * chassis_refe_data.speed_vy;
    float chassis_send_vy = -sin_yaw * chassis_refe_data.speed_vx + cos_yaw * chassis_refe_data.speed_vy;

    x +=  chassis_send_vx*time_delta/100.0f; // 目前没有多传感器融合的需求,frame_id暂时没什么用,先固定为0
    y +=  chassis_send_vy*time_delta*10.8/100.0f;
    vision_send_data.x = x;
    vision_send_data.y = y;
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    vision_send_data.pitch = gimbal_feedback_data.gimbal_imu_data.Pitch;
    vision_send_data.yaw = gimbal_feedback_data.gimbal_imu_data.Yaw;   
    vision_send_data.roll = gimbal_feedback_data.gimbal_imu_data.Roll;
    // vision_send_data.chassis_angle = 0.0f;

    // vision_send_data.vx = chassis_refe_data.speed_vx;
    // vision_send_data.vy = chassis_refe_data.speed_vy;
    // vision_send_data.present_roll = chassis_refe_data.speed_wz;
    // vision_send_data.present_pitch = chassis_refe_data.speed_vx;
    // vision_send_data.present_yaw = chassis_refe_data.speed_vy;   
    // vision_send_data.reserved_slot = 0;
    // void gimbal_absolute_angle_c();
    VisionSend(&vision_send_data);
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
