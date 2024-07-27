#include "stretch.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "encoder.h"

#include "bmi088.h"

static DJIMotorInstance *left_motor, *right_motor;
static Publisher_t *stretch_pub;                          // 二级应用消息发布者(二级反馈给cmd)
static Subscriber_t *stretch_sub;                         // cmd控制消息订阅者
static Stretch_Upload_Data_s stretch_feedback_data; // 回传给cmd的二级状态信息
static Stretch_Ctrl_Cmd_s stretch_cmd_recv;         // 来自cmd的控制信息

void Stretch_Init()
{
    Motor_Init_Config_s stretch_config = {
        .can_init_config = {
            .can_handle = &hfdcan3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 200,
                .Ki            = 0,
                .Kd            = 0,
                // .DeadBand      = 0.1f,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 10000,
            },
            .speed_PID = {
                .Kp            = 0.5,
                .Ki            = 0,
                .Kd            = 0.005,//0.0065,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut        = 7000,
            },
        },
        .controller_setting_init_config = {                                                                                                                                                                                                                              
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = M3508};
    
    stretch_config.can_init_config.tx_id                             =5;
    stretch_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    left_motor                                                            = DJIMotorInit(&stretch_config);

    stretch_config.can_init_config.tx_id                             =1;
    stretch_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    right_motor                                                            = DJIMotorInit(&stretch_config);

    stretch_pub = PubRegister("stretch_feed", sizeof(Stretch_Upload_Data_s));
    stretch_sub = SubRegister("stretch_cmd", sizeof(Stretch_Ctrl_Cmd_s));
    
}


void Stretch_Task()
{
    SubGetMessage(stretch_sub, &stretch_cmd_recv);
    switch (stretch_cmd_recv.stretch_mode) {
        // 停止
        case STRETCH_STOP:
            DJIMotorStop(left_motor);
            DJIMotorStop(right_motor);
            break;
        case STRETCH:
            DJIMotorEnable(left_motor);
            DJIMotorEnable(right_motor);
            break;
        default:
            break;
    }
    DJIMotorSetRef(left_motor, stretch_cmd_recv.left_angle);
    DJIMotorSetRef(right_motor, stretch_cmd_recv.right_angle);


    // 设置反馈数据,主要是imu和yaw的ecd
    stretch_feedback_data.now_left_angle  = left_motor->measure.total_angle;
    stretch_feedback_data.now_right_angle = right_motor->measure.total_angle;

    stretch_feedback_data.now_left_current = left_motor->measure.real_current;
    stretch_feedback_data.now_right_current = right_motor->measure.real_current;

    stretch_feedback_data.now_left_speed = left_motor->measure.speed_aps;
    stretch_feedback_data.now_right_speed = right_motor->measure.speed_aps;


    // 推送消息
    PubPushMessage(stretch_pub, (void *)&stretch_feedback_data);
}