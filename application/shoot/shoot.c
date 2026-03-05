#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 14, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 16384,
            },
            .current_PID = {
                .Kp = 0.38, // 0.7
                .Ki = 0.12, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 16384,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 1,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 4; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 10, // 10
                .Ki = 1.2,
                .Kd = 0,
                .MaxOut = 15000,
            },
            .speed_PID = {
                .Kp = 8, // 10
                .Ki = 0.8, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.08, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP, 
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M3508 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    static uint8_t last_load_mode = LOAD_STOP; 
    static float snapshot_angle = 0;
    static float target_angle = 0;
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
        last_load_mode = LOAD_STOP; 
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;
    // 定义静态变量
    static uint8_t reverse_trigger = 0; 
    // 记录上一次的目标角度，用于在锁定时保持位置
    static float reverse_target_angle = 0; 
    float current_time = DWT_GetTimeline_ms();
    uint8_t is_cooling_down = (current_time < hibernate_time + dead_time);


    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        reverse_trigger = 0; 
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    // case LOAD_1_BULLET:
    // if (last_load_mode != LOAD_1_BULLET && !is_cooling_down) 
    //     {                                                                     // 激活能量机关/干扰对方用,英雄用.
    //     DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
    //     DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
    //     hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
    //     dead_time = 100;
    //     reverse_trigger = 0;
    //     }                                                                     // 完成1发弹丸发射的时间
    //     break;
    // 三连发,如果不需要后续可能删除
    case LOAD_1_BULLET:
    {
        DJIMotorOuterLoop(loader, ANGLE_LOOP);

        // 冷却中：保持上一次目标角度，不再新增
        if (is_cooling_down)
        {
            DJIMotorSetRef(loader, target_angle);
            break;
        }

        // 只在“从其它模式切到单发”的那一帧触发一次
        if (last_load_mode != LOAD_1_BULLET)
        {
            // 以当前测得的总角度为基准，每次多转固定角度
            float snapshot_angle = loader->measure.total_angle;
            // 注意正负方向，和你实际出弹方向保持一致
            target_angle = snapshot_angle - ONE_BULLET_DELTA_ANGLE;

            DJIMotorSetRef(loader, target_angle);

            hibernate_time = current_time;
            dead_time      = 150.0f;   // 按机械情况再调
            reverse_trigger = 0;
        }
        else
        {
            // 按住不松的过程中，只维持目标
            DJIMotorSetRef(loader, target_angle);
        }
    }
    break;


    case LOAD_3_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                  // 切换到速度环
        DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
        hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
        dead_time = 300;
        reverse_trigger = 0;                                                                         // 完成3发弹丸发射的时间
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
    {
        
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        
        // 检查：确保 shoot_rate 不为 0
        // 如果 robot_cmd 没发具体的 shoot_rate，这里最好给个默认保底值
        float current_shoot_rate = shoot_cmd_recv.shoot_rate;
        if (current_shoot_rate == 0) {current_shoot_rate = 8;} // 默认8发/秒
        
        DJIMotorSetRef(loader, -current_shoot_rate * (360.0f/6.0f) * REDUCTION_RATIO_LOADER );
        reverse_trigger = 0; 
       }   break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    //   case LOAD_REVERSE:
    //     DJIMotorOuterLoop(loader, ANGLE_LOOP); // 切角度环

    //     // 如果是第一次进入反转逻辑（trigger == 0）
    //     if (reverse_trigger == 0)
    //     {
    //         // 1. 设定目标：当前角度往回退 60 度
    //         reverse_target_angle = loader->measure.total_angle - 60;
    //         DJIMotorSetRef(loader, reverse_target_angle);
            
    //         // 2. 标记已触发，防止下一帧循环再次减角度
    //         reverse_trigger = 1;
    //     }
    //     else
    //     {
    //         // 3. 如果一直按着 E 键（trigger 已经是 1），就保持在这个目标位置不动
    //         // 相当于“锁住”在回退后的位置
    //         DJIMotorSetRef(loader, reverse_target_angle);
    //     }
    //     break;
    // case LOAD_REVERSE:
    //     DJIMotorOuterLoop(loader, SPEED_LOOP);
    //     // ...
    //     break;
    case LOAD_REVERSE:{
    DJIMotorOuterLoop(loader, SPEED_LOOP);
    DJIMotorSetRef(loader, 1500);  // 负速度反转
    break;}

    default:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, 0);
        reverse_trigger = 0;
        break;
        
        // while (1)
        //     ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }
     last_load_mode = shoot_cmd_recv.load_mode;

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_l, -10000);
            DJIMotorSetRef(friction_r, 10000);
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(friction_l, 36000);
            DJIMotorSetRef(friction_r, -36000);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_l, 60000);
            DJIMotorSetRef(friction_r, -60000);
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}