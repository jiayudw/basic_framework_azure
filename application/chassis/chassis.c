/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
// #include "referee_task.h"
#include "rm_referee.h"
#include "general_def.h"
#include "bsp_dwt.h"
// #include "referee_UI.h"
#include "arm_math.h"
#include "bsp_log.h"
//添加功率控制头文件
#include "power_meter.h"
PIDInstance power_pid;
PID_Init_Config_s power_pid_config;


/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
static Subscriber_t *gimbal_sub;    //获取云台角度
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
// static Chassis_Send_s chassis_send_data;               // 发送给视觉的底盘数据

static referee_info_t* referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
static float real_vx, real_vy;   // 真实系的速度
static float time, robot_start; // 计时器相关变量

static float sin_theta, cos_theta;

void ChassisInit()
{
    
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 4.void Chassis5
                .Ki = 1,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.4, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&chassis_motor_config);    

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorInit(&chassis_motor_config);
    
    referee_data = RefereeInit(&huart6); // 裁判系统初始化（已移除UI）

    // referee_data = UITaskInit(&huart6,&ui_data); // 裁判系统初始化,会同时初始化UI

    //添加功率计初始化
    // power_meter_init(); // 功率计初始化
    // power_pid_config.Kp = 0.05; // 功率PID参数
    // power_pid_config.Ki = 0;
    // power_pid_config.Kd = 0;
    // power_pid_config.MaxOut = 1000;
    // PIDInit(&power_pid, &power_pid_config); // 功率PID初始化

    // SuperCap_Init_Config_s cap_conf = {
    //     .can_config = {
    //         .can_handle = &hcan2,
    //         .tx_id = 0x302, // 超级电容默认接收id
    //         .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
    //     }};
    // cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf =  chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf =  chassis_vx + chassis_vy + chassis_cmd_recv.wz * RF_CENTER;
    vt_lb =  chassis_vx + chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb =  chassis_vx - chassis_vy + chassis_cmd_recv.wz * RB_CENTER;
    // vt_lf =  chassis_vx - chassis_vy -;
    // vt_rf =  chassis_vx + chassis_vy ;
    // vt_lb =  chassis_vx + chassis_vy ;
    // vt_rb =  chassis_vx - chassis_vy ;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
        // =========================================================
    // 1. 参数调优区 (常量定义，便于赛场快速修改)
    // =========================================================
    const float MAX_WHEEL_SPEED = 21200.0f; // M3508电机的最大安全设定转速(根据实际PID整定修改)
    
    // 缓冲能量阈值
    const float BUF_WARN_THRES   = 60.0f;  // 开始轻微限制的阈值
    const float BUF_DANGER_THRES = 30.0f;  // 开始中等限制的阈值
    const float BUF_EXTREME_THRES= 10.0f;  // 极度危险限制的阈值
    
    // 对应的功率缩放系数
    const float SCALE_SAFE       = 1.0f;   // 100% 输出
    const float SCALE_WARN       = 0.6f;   // 60%  输出
    const float SCALE_DANGER     = 0.4f;   // 40%  输出
    const float SCALE_EXTREME    = 0.2f;   // 20%  输出

    // =========================================================
    // 2. 状态获取与裁判系统离线保护
    // =========================================================
    float power_scale    = SCALE_SAFE; 
    float current_buffer = chassis_feedback_data.buffer_energy;
    float power_limit    = chassis_feedback_data.chassis_power_limit; 

    // 判断如果power_limit大于0（表明插了裁判系统且正在通讯）才限制功率
    if (power_limit > 0.0f) 
    {
        if (current_buffer < BUF_EXTREME_THRES) 
        {
            power_scale = SCALE_EXTREME;
        } 
        else if (current_buffer < BUF_DANGER_THRES) 
        {
            power_scale = SCALE_DANGER;
        } 
        else if (current_buffer < BUF_WARN_THRES) 
        {
            power_scale = SCALE_WARN;
        } 
        else 
        {
            power_scale = SCALE_SAFE;
        }
    }

    // =========================================================
    // 3. 施加功率衰减 (等比例缩放，保证麦轮受力方向不变)
    // =========================================================
    vt_lf *= power_scale;
    vt_rf *= power_scale;
    vt_lb *= power_scale;
    vt_rb *= power_scale;

    // =========================================================
    // 4. 运动学最大转速限幅防爆冲
    // =========================================================
    // 找出四个轮子解算速度的绝对值最大者
    float max_speed = fabsf(vt_lf);
    if (fabsf(vt_rf) > max_speed) { max_speed = fabsf(vt_rf); }
    if (fabsf(vt_lb) > max_speed) { max_speed = fabsf(vt_lb); }
    if (fabsf(vt_rb) > max_speed) { max_speed = fabsf(vt_rb); }

    // 如果最大值超出了底盘物理极限，则进行二次等比例压缩
    if (max_speed > MAX_WHEEL_SPEED)
    {
        float rate = MAX_WHEEL_SPEED / max_speed;
        vt_lf *= rate;
        vt_rf *= rate;
        vt_lb *= rate;
        vt_rb *= rate;
    }

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...

    float wheel_speed_lf_dps = motor_lf->measure.speed_aps;
    float wheel_speed_rf_dps = motor_rf->measure.speed_aps;
    float wheel_speed_lb_dps = motor_lb->measure.speed_aps;
    float wheel_speed_rb_dps = motor_rb->measure.speed_aps;
    //逆运动学解算
    float dps_to_mps = (PERIMETER_WHEEL / REDUCTION_RATIO_WHEEL) / 360.0f;
    //转换为轮子线速度 
    float v_lf = wheel_speed_lf_dps * dps_to_mps;
    float v_rf = wheel_speed_rf_dps * dps_to_mps;
    float v_lb = wheel_speed_lb_dps * dps_to_mps;
    float v_rb = wheel_speed_rb_dps * dps_to_mps;

    float vx = (( -v_lf - v_lb + v_rf + v_rb) / 4.0f)/9.0f;  // X方向速度（前后）
    float vy = ((v_lf - v_lb + v_rf - v_rb) / 4.0f)/9.0f;  // Y方向速度（左右）
    //float vy = (v_lf + v_rf + v_lb + v_rb) / 4.0f;
    //float vx = (v_lf - v_rf - v_lb + v_rb) / 4.0f;

    // real_vx = vx*cos_theta + vy*sin_theta;
    // real_vy = -vx*sin_theta + vy*cos_theta;

    chassis_feedback_data.speed_vx = vx;
    chassis_feedback_data.speed_vy = -vy;




    // chassis_feedback_data.speed_wz = chassis_cmd_recv.wz; // 由于没有陀螺仪数据,暂时用命令中的角速度作为反馈,后续增加IMU数据后再修改

}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{

    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

    robot_start = DWT_GetTimeline_ms();
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);

    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);

    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        // chassis_cmd_recv.wz = 0.0f;// v = 0
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
        chassis_cmd_recv.wz = 2.0f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
        break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加规则的变速策略
        chassis_cmd_recv.wz = 1000;
        if (chassis_cmd_recv.wz > 2000){
            chassis_cmd_recv.wz == 2000;
        }
        else if (chassis_cmd_recv.wz < 1000){
            chassis_cmd_recv.wz == 1000;
        }
        else if (chassis_cmd_recv.wz == 1000)
        {
            chassis_cmd_recv.wz += 100;
        }
        else if(chassis_cmd_recv.wz == 2000 ){
            chassis_cmd_recv.wz -= 100; 
        }
        break;
    default:
        break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    // chassis_vy = chassis_cmd_recv.vy;
    // chassis_vx = chassis_cmd_recv.vx;
    // chassis_vx = chassis_cmd_recv.vx;//消除了云台速度对底盘速度的影响
    // chassis_vy = chassis_cmd_recv.vy;
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;  
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    //这


    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 推送反馈消息
#ifdef ONE_BOARD
    //chassis_feedback_data.chassis_power = referee_data->PowerHeatData.chassis_power;
    chassis_feedback_data.chassis_power_limit = (float)referee_data->GameRobotState.chassis_power_limit;
    chassis_feedback_data.buffer_energy = (float)referee_data->PowerHeatData.buffer_energy;

    // chassis_feedback_data.shoot_heat = (float)referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    chassis_feedback_data.shoot_heat = (float)referee_data->PowerHeatData.shooter_17mm_barrel_heat;

    chassis_feedback_data.shoot_heat_limit = (float)referee_data->GameRobotState.shooter_barrel_heat_limit;
    
    chassis_feedback_data.robot_HP = referee_data->GameRobotState.current_HP;

    //添加弹速反馈
    chassis_feedback_data.initial_speed = referee_data->ShootData.initial_speed;
    // 添加敌方颜色反馈: Robot_Red=0 Robot_Blue=1, 敌方颜色与我方相反
    chassis_feedback_data.enemy_color = (uint8_t)(referee_data->referee_id.Robot_Color == Robot_Red) ? Robot_Blue : Robot_Red;
    
    // 添加底盘速度控制指令反馈代码
    // chassis_feedback_data.speed_vx = chassis_cmd_recv.vx;
    // chassis_feedback_data.speed_vy = chassis_cmd_recv.vy;
    chassis_feedback_data.speed_wz = chassis_cmd_recv.wz;
    
    // 发送底盘数据到上位机
    // ChassisSend(&chassis_send_data);

    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}
